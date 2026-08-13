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

  /// \brief Command that updates the gimbal tilt angle
  /// (orijinali IGN_PI_2 idi; acilista kamera savrulmasin diye 0)
  public: double command = 0.0;

  /// \brief Pointer to the transport node
  public: transport::NodePtr node;

  /// \brief HIZ-SERVO kontrol (tork-PID DEGIL). Tork-PID kucuk eklem
  /// ataletinde ya yavas kaliyor ya da tepki torkuyla tasiyici govdeyi
  /// sallayip birlikte osilasyona giriyordu (sallanan platformda olculdu).
  /// ODE joint motoru: vel = clamp(kv*hata), tork siniri fmax.
  public: double servoKv = 150.0;      // rad/s / rad
  public: double servoVelMax = 6.0;    // rad/s (~344 deg/s)
  public: double servoFmax = 0.15;     // N*m — kamera ataleti ~1e-5 icin bol,
                                       // govdeye tepkisi gercekten ihmal edilebilir
  /// \brief olu bant: durgun halde olcum jitter'inin surekli hiz komutuna
  /// donusup govdeye titresim basmasini engeller (SITL'de "Gyros not
  /// calibrated" ile kalkisi bile engelliyordu)
  public: double servoDeadband = 0.003; // rad (~0.17 deg)

  /// \brief true ise komut kameranin DUNYA pitch'idir (govde pitch'i telafi
  /// edilir); false ise eski davranis: eklem acisi govdeye gore tutulur.
  public: bool stabilize = false;

  /// \brief Kamera optik ekseninin tilt_link YEREL cercevesindeki yonu
  /// (stabilize modunda dunya pitch'i bu eksen uzerinden olculur).
  public: ignition::math::Vector3d cameraAxis{0, 1, 0};

  /// \brief Kontrol edilen buyuklugun guncel degeri (status'ta yayinlanir):
  /// stabilize modunda kamera dunya pitch'i, degilse eklem acisi.
  public: double measured = 0.0;

  /// \brief status yayin sayaci (orijinalde static idi — 5 gimbal ayni
  /// sayaci paylasip birbirinin yayinini seyreltiyordu)
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
    // ODE eklem motoru: tork siniri bir kez ayarlanir, hiz her adimda
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
    // kamera ekseninin dunya pitch'i (pozitif = yukari); govde pitch'i
    // otomatik telafi edilir cunku hata dunya cercevesinde olculuyor
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

  // hiz-servo: pozitif eklem yonu = kamera yukari = pozitif pitch
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
