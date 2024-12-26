//
// Created by rgrandia on 14.10.21.
//
#pragma once

#include "CatModel2_v2_motion/CatModel2MotionInterface.h"
#include "CatModel2_v2_interface/gait/ModeSequenceTemplate.h"


#include <visualization_msgs/msg/marker.hpp>
#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

namespace ocs2 {
namespace legged_robot {

class CatModel2MotionDummy : public CatModel2MotionInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CatModel2MotionDummy(const rclcpp::Node::SharedPtr& node,
                     const std::string& configFile,
                     const std::string& robotName);
  ~CatModel2MotionDummy() override = default;

  void publishMotion(const std::pair<ocs2::TargetTrajectories, ModeSequenceTemplate>& motion) override;

 private:
  void observationCallback(
      const ocs2_msgs::msg::MpcObservation::ConstSharedPtr& msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr
      targetTrajectoryPublisher_;
  rclcpp::Publisher<ocs2_msgs::msg::ModeSchedule>::
      SharedPtr gaitSequencePublisher_;

  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observationSubscriber_;
  std::mutex observationMutex_;
  std::unique_ptr<ocs2::SystemObservation> observationPtr_;
};

} // legged_robot
}  // ocs2