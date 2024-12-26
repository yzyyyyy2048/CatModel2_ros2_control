
#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "CatModel2_v2_motion/CatModel2MotionDummy.h"

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char* argv[]) {
  const std::string robotName = "legged_robot";
  std::string motionFile = ament_index_cpp::get_package_share_directory("CatModel2_v2_motion") + "/config/motions.info";
  std::cerr << "Loading motion file: " << motionFile << std::endl;

  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared(robotName + "_mpc_motion_command");

  std::unique_ptr<CatModel2MotionInterface> motionCommandInterface;
  motionCommandInterface.reset(new CatModel2MotionDummy(node, motionFile, robotName));

  rclcpp::Rate rate(10);
  while (rclcpp::ok())
  {
    motionCommandInterface->getKeyboardCommand();
    rate.sleep();
  }

  // Successful exit
  return 0;
}