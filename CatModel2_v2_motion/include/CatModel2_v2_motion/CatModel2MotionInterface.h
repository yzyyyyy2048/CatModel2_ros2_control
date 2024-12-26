//
// Created by Timon Kaufmann in June 2021
//
#pragma once

#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"

#include <ocs2_core/reference/TargetTrajectories.h>
#include "CatModel2_v2_interface/gait/ModeSequenceTemplate.h"

namespace ocs2 {
namespace legged_robot {

class CatModel2MotionInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit CatModel2MotionInterface(const std::string& configFile);
  virtual ~CatModel2MotionInterface() = default;

  virtual void publishMotion(const std::pair<ocs2::TargetTrajectories, ModeSequenceTemplate>& motion) = 0;

  void getKeyboardCommand();

 private:
  void printAnimationList() const;

  std::unordered_map<std::string, std::pair<ocs2::TargetTrajectories, ModeSequenceTemplate>> motionData_;
};

} // legged_robot
}  // ocs2