//
// Created by Timon Kaufmann in June 2021
//

#include "rclcpp/rclcpp.hpp"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "CatModel2_v2_motion/LoadMotions.h"
#include "CatModel2_v2_motion/CatModel2MotionInterface.h"


namespace ocs2 {
namespace legged_robot {

CatModel2MotionInterface::CatModel2MotionInterface(const std::string& configFile) {
  bool verbose = false;

  scalar_t dt = 0.0;
  ocs2::loadData::loadCppDataType(configFile, "dt", dt);

  std::vector<std::string> motionList;
  ocs2::loadData::loadStdVector(configFile, "motionList", motionList, verbose);

  const std::string motionFilesPath = ament_index_cpp::get_package_share_directory("CatModel2_v2_motion") + "/config/motions/";
  for (const auto& motionName : motionList) {
    const auto csvData = readCsv(motionFilesPath + motionName + ".txt");
    motionData_.insert({motionName, readMotion(csvData, dt)});
  }
  printAnimationList();
}

void CatModel2MotionInterface::getKeyboardCommand() {
  const std::string commandMsg = "Enter the desired motion, for the list of available motions enter \"list\"";
  std::cout << commandMsg << ": ";

  auto shouldTerminate = []() { return !rclcpp::ok(); };
  const std::string motionCommand = ocs2::getCommandLineString(shouldTerminate);

  if (motionCommand.empty()) {
    return;
  }

  if (motionCommand == "list") {
    printAnimationList();
    return;
  }

  try {
    const auto& motion = motionData_.at(motionCommand);
    std::cout << "Executing \"" << motionCommand << "\" \n";
    publishMotion(motion);
  } catch (const std::out_of_range& e) {
    std::cout << "Motion \"" << motionCommand << "\" not found.\n";
    printAnimationList();
  }
}

void CatModel2MotionInterface::printAnimationList() const {
  std::cout << "\nList of available motions:\n";
  for (const auto& s : motionData_) {
    std::cout << "  * " << s.first << "\n";
  }
  std::cout << std::endl;
}

} // legged_robot
}  // ocs2