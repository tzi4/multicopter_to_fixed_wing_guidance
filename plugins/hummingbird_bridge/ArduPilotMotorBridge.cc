/*
 * Adapter between the legacy ArduPilot Gazebo joint-control interface and
 * PX4 Gazebo Classic's CommandMotorSpeed transport message.
 *
 * ArduPilotPlugin drives four command-only joints. This plugin samples those
 * joint velocities and republishes them without changing scale or ordering.
 * The upstream gazebo_motor_model plugins remain solely responsible for the
 * physical rotor dynamics.
 */
#include <array>
#include <memory>
#include <string>

#include <boost/bind.hpp>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>

#include "CommandMotorSpeed.pb.h"

namespace gazebo {

class ArduPilotMotorBridge final : public ModelPlugin {
 public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = std::move(model);
    std::string target_model = "iris-1";
    std::string command_topic = "/gazebo/command/motor_speed";
    if (sdf->HasElement("targetModelName")) {
      target_model = sdf->Get<std::string>("targetModelName");
    }
    if (sdf->HasElement("commandSubTopic")) {
      command_topic = sdf->Get<std::string>("commandSubTopic");
    }

    auto joint_elem = sdf->GetElement("jointName");
    std::size_t index = 0;
    while (joint_elem && index < joints_.size()) {
      const std::string name = joint_elem->Get<std::string>();
      joints_[index] = model_->GetJoint(name);
      if (!joints_[index]) {
        gzthrow("[ArduPilotMotorBridge] Joint not found: " << name);
      }
      ++index;
      joint_elem = joint_elem->GetNextElement("jointName");
    }
    if (index != joints_.size()) {
      gzthrow("[ArduPilotMotorBridge] Exactly four jointName entries are required");
    }

    node_.reset(new transport::Node());
    node_->Init();
    const std::string topic = "~/" + target_model + command_topic;
    publisher_ = node_->Advertise<mav_msgs::msgs::CommandMotorSpeed>(topic, 1);
    update_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&ArduPilotMotorBridge::OnUpdate, this));
    gzmsg << "[ArduPilotMotorBridge] Publishing motor commands on "
          << topic << "\n";
  }

 private:
  void OnUpdate() {
    mav_msgs::msgs::CommandMotorSpeed message;
    for (const auto &joint : joints_) {
      message.add_motor_speed(static_cast<float>(joint->GetVelocity(0)));
    }
    publisher_->Publish(message);
  }

  physics::ModelPtr model_;
  std::array<physics::JointPtr, 4> joints_;
  transport::NodePtr node_;
  transport::PublisherPtr publisher_;
  event::ConnectionPtr update_;
};

GZ_REGISTER_MODEL_PLUGIN(ArduPilotMotorBridge)
}  // namespace gazebo
