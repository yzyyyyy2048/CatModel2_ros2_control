#include <gtest/gtest.h>
#include <experimental/filesystem>

#include "CatModel2_v2_motion/LoadMotions.h"
#include "CatModel2_v2_interface/gait/ModeSequenceTemplate.h"

using namespace ocs2::legged_robot;

inline std::string getAbsolutePathToConfigurationFile(const std::string& fileName) {
  const std::experimental::filesystem::path pathToTest = std::experimental::filesystem::path(__FILE__);
  return std::string(pathToTest.parent_path()) + "/" + fileName;
}

TEST(testLoadMotions, loadcsv) {
  const auto file = getAbsolutePathToConfigurationFile("data/testCsv.txt");
  const auto result = readCsv(file);
  const auto& header = result.header;
  ASSERT_EQ(header[0], "t");
  ASSERT_EQ(header[1], "x0");
  ASSERT_EQ(header[2], "x1");

  const auto& data = result.data;
  ASSERT_EQ(data.size(), 2);
  ASSERT_EQ(data[0].size(), 3);
  ASSERT_EQ(data[1].size(), 3);
  ASSERT_DOUBLE_EQ(data[0][0], 0.0);
  ASSERT_DOUBLE_EQ(data[0][1], 1.0);
  ASSERT_DOUBLE_EQ(data[0][2], 2.0);
  ASSERT_DOUBLE_EQ(data[1][0], 1.0);
  ASSERT_DOUBLE_EQ(data[1][1], 2.0);
  ASSERT_DOUBLE_EQ(data[1][2], 3.0);
}

TEST(testLoadMotions, loadmotion) {
  const auto file = getAbsolutePathToConfigurationFile("data/testMotion.txt");
  const auto csvData = readCsv(file);
  const auto result = readMotion(csvData);
  const auto& modeseq = result.second;
  const auto& gait = toGait(modeseq);

  ASSERT_TRUE(isValidGait(gait));
}