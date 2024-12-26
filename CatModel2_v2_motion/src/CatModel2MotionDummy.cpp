//
// Created by Timon Kaufmann in June 2021
//

#include "CatModel2_v2_motion/CatModel2MotionDummy.h"

#include "rclcpp/rclcpp.hpp"
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_interface/ros_msg_conversions/RosMsgConversions.h>

#include "CatModel2_v2_dummy/gait/ModeSequenceTemplateRos.h"

namespace ocs2 {
namespace legged_robot {

CatModel2MotionDummy::CatModel2MotionDummy(const rclcpp::Node::SharedPtr &node, const std::string &configFile, const std::string &robotName)
    : CatModel2MotionInterface(configFile), node_(node){
  // Publishers
  targetTrajectoryPublisher_ = node->create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(robotName + "_mpc_target", 1);
  gaitSequencePublisher_ =
        node->create_publisher<ocs2_msgs::msg::ModeSchedule>(robotName + "_mpc_gait_schedule", 1);
  
  // Subsribers
  observationSubscriber_ = node->create_subscription<ocs2_msgs::msg::MpcObservation>(robotName + "_mpc_observation", 1,
                                                                                       std::bind(&CatModel2MotionDummy::observationCallback, this, std::placeholders::_1));
}

void CatModel2MotionDummy::observationCallback(const ocs2_msgs::msg::MpcObservation::ConstSharedPtr &msg) {
  std::lock_guard<std::mutex> lock(observationMutex_);
  observationPtr_.reset(new ocs2::SystemObservation(ocs2::ros_msg_conversions::readObservationMsg(*msg)));
}

void CatModel2MotionDummy::publishMotion(const std::pair<ocs2::TargetTrajectories, ModeSequenceTemplate>& motion) {
  rclcpp::spin_some(node_); // Trigger callback
  ocs2::SystemObservation observation;
  {
    std::lock_guard<std::mutex> lock(observationMutex_);
    if (observationPtr_) {
      observation = *observationPtr_;
    } else {
      std::cout << "No observation is received from the MPC node. Make sure the MPC node is running!"
                << "\n";
      return;
    }
  }
  const scalar_t startTime = observation.time + 1.0;

  
  auto mpcTargetTrajectoriesMsg = ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(motion.first);
  for (auto& t : mpcTargetTrajectoriesMsg.time_trajectory) {
    t += startTime;
  }

  std::vector<size_t> modeSequence = {9, 6};
  std::vector<scalar_t> switchingTimes = {0.0, 0.3,0.6};
  ModeSequenceTemplate trot(switchingTimes, modeSequence);
  gaitSequencePublisher_->publish(createModeSequenceTemplateMsg(trot));
  targetTrajectoryPublisher_->publish(mpcTargetTrajectoriesMsg);
}

} // legged_robot
}  // ocs2