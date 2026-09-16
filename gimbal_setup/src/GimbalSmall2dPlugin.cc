/*
 * Copyright (C) 2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <string>
#include <vector>

#include "gazebo/common/PID.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "GimbalSmall2dPlugin.hh"

using namespace gazebo;
using namespace std;

GZ_REGISTER_MODEL_PLUGIN(GimbalSmall2dPlugin)

/// \brief Private data class
class gazebo::GimbalSmall2dPluginPrivate
{
  /// \brief Callback when a command string is received.
  /// \param[in] _msg Mesage containing the command string
  public: void OnStringMsg(ConstGzStringPtr &_msg);

  /// \brief A list of event connections
  public: std::vector<event::ConnectionPtr> connections;

  /// \brief Subscriber to the gimbal command topic
  public: transport::SubscriberPtr sub;

  /// \brief Publisher to the gimbal status topic
  public: transport::PublisherPtr pub;

  /// \brief Parent model of this plugin
  public: physics::ModelPtr model;

  /// \brief Joint for tilting the gimbal
  public: physics::JointPtr tiltJoint;

  // / \brief Command that updates the gimbal tilt angle / (original was IGN_PI_2; 0 to prevent the
  // camera from swinging during startup)
  public: double command = 0.0;

  /// \brief Pointer to the transport node
  public: transport::NodePtr node;

  // / \brief SPEED-SERVO control (NOT torque-PID). Torque-PID small joint / was either slow in its
  // inertia or was oscillating and shaking the carrier body / with the reaction torque (measured on the
  // swinging platform). / ODE joint motor: vel = clamp(kv*error), torque limit fmax.
  public: double servoKv = 150.0;      // rad/s / rad
  public: double servoVelMax = 6.0;    // rad/s (~344 deg/s)
  public: double servoFmax = 0.15;     // N*m: sufficient for camera inertia ~1e-5,
                                       // its reaction to the body is really negligible / \brief dead band: it prevents the measurement jitter
                                       // from rest turning into a continuous speed command / and causing vibration to the body (in SITL, it
                                       // even prevented take-off with "Gyros not / calibrated")
  public: double servoDeadband = 0.003; // rad (~0.17 deg)

  // If / \brief is true, the command is the camera's WORLD pitch (body pitch is / compensated); If
  // false, old behavior: joint angle is kept relative to the body.
  public: bool stabilize = false;

  // / \brief The direction of the camera optical axis in the tilt_link LOCAL frame / (in stabilized mode
  // the world pitch is measured along this axis).
  public: ignition::math::Vector3d cameraAxis{0, 1, 0};

  // / \brief Current value of the controlled quantity (published in status): / camera world pitch in
  // stabilized mode, otherwise joint angle.
  public: double measured = 0.0;

  // / \brief status broadcast counter (originally static — 5 gimbal shared the same / counter and
  // diluted each other's broadcast)
  public: int pubCounter = 1000;
};

/////////////////////////////////////////////////
GimbalSmall2dPlugin::GimbalSmall2dPlugin()
  : dataPtr(new GimbalSmall2dPluginPrivate)
{
}

/////////////////////////////////////////////////
void GimbalSmall2dPlugin::Load(physics::ModelPtr _model,
  sdf::ElementPtr _sdf)
{
  this->dataPtr->model = _model;

  std::string jointName = "tilt_joint";
  if (_sdf->HasElement("joint"))
  {
    jointName = _sdf->Get<std::string>("joint");
  }
  if (_sdf->HasElement("initial_angle"))
  {
    this->dataPtr->command = _sdf->Get<double>("initial_angle");
  }
  if (_sdf->HasElement("servo_kv"))
    this->dataPtr->servoKv = _sdf->Get<double>("servo_kv");
  if (_sdf->HasElement("servo_vel_max"))
    this->dataPtr->servoVelMax = _sdf->Get<double>("servo_vel_max");
  if (_sdf->HasElement("servo_fmax"))
    this->dataPtr->servoFmax = _sdf->Get<double>("servo_fmax");
  if (_sdf->HasElement("servo_deadband"))
    this->dataPtr->servoDeadband = _sdf->Get<double>("servo_deadband");
  if (_sdf->HasElement("stabilize"))
    this->dataPtr->stabilize = _sdf->Get<bool>("stabilize");
  if (_sdf->HasElement("camera_axis"))
  {
    this->dataPtr->cameraAxis =
      _sdf->Get<ignition::math::Vector3d>("camera_axis");
    this->dataPtr->cameraAxis.Normalize();
  }
  this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(jointName);
  if (!this->dataPtr->tiltJoint)
  {
    std::string scopedJointName = _model->GetScopedName() + "::" + jointName;
    gzwarn << "joint [" << jointName
           << "] not found, trying again with scoped joint name ["
           << scopedJointName << "]\n";
    this->dataPtr->tiltJoint = this->dataPtr->model->GetJoint(scopedJointName);
  }
  if (!this->dataPtr->tiltJoint)
  {
    gzerr << "GimbalSmall2dPlugin::Load ERROR! Can't get joint '"
          << jointName << "' " << endl;
  }
}

/////////////////////////////////////////////////
void GimbalSmall2dPlugin::Init()
{
  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->model->GetWorld()->Name());

  if (this->dataPtr->tiltJoint)
  {
    // ODE joint motor: torque limit is set once, speed every step
    this->dataPtr->tiltJoint->SetParam("fmax", 0, this->dataPtr->servoFmax);
  }

  std::string topic = std::string("~/") +  this->dataPtr->model->GetName() +
    "/gimbal_tilt_cmd";
  this->dataPtr->sub = this->dataPtr->node->Subscribe(topic,
      &GimbalSmall2dPluginPrivate::OnStringMsg, this->dataPtr.get());

  this->dataPtr->connections.push_back(event::Events::ConnectWorldUpdateBegin(
          std::bind(&GimbalSmall2dPlugin::OnUpdate, this)));

  topic = std::string("~/") +
    this->dataPtr->model->GetName() + "/gimbal_tilt_status";

  this->dataPtr->pub =
    this->dataPtr->node->Advertise<gazebo::msgs::GzString>(topic);
}

/////////////////////////////////////////////////
void GimbalSmall2dPluginPrivate::OnStringMsg(ConstGzStringPtr &_msg)
{
  this->command = atof(_msg->data().c_str());
}

/////////////////////////////////////////////////
void GimbalSmall2dPlugin::OnUpdate()
{
  if (!this->dataPtr->tiltJoint)
    return;

  double angle;
  if (this->dataPtr->stabilize)
  {
    // world pitch of camera axis (positive = up); body pitch is automatically compensated because the
    // error is measured in the world frame
    ignition::math::Vector3d axisWorld =
      this->dataPtr->tiltJoint->GetChild()->WorldPose().Rot()
        .RotateVector(this->dataPtr->cameraAxis);
    angle = atan2(axisWorld.Z(),
      sqrt(axisWorld.X()*axisWorld.X() + axisWorld.Y()*axisWorld.Y()));
  }
  else
  {
    angle = this->dataPtr->tiltJoint->Position(0);
  }
  this->dataPtr->measured = angle;

  // speed-servo: positive joint direction = camera up = positive pitch
  double err = this->dataPtr->command - angle;
  double vel = 0.0;
  if (fabs(err) > this->dataPtr->servoDeadband)
  {
    vel = this->dataPtr->servoKv * err;
    if (vel > this->dataPtr->servoVelMax) vel = this->dataPtr->servoVelMax;
    if (vel < -this->dataPtr->servoVelMax) vel = -this->dataPtr->servoVelMax;
  }
  this->dataPtr->tiltJoint->SetParam("vel", 0, vel);

  if (++this->dataPtr->pubCounter > 20)
  {
    this->dataPtr->pubCounter = 0;
    std::stringstream ss;
    ss << angle;
    gazebo::msgs::GzString m;
    m.set_data(ss.str());
    this->dataPtr->pub->Publish(m);
  }
}
