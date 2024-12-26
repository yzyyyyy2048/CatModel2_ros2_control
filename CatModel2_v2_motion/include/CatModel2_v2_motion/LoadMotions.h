//
// Created by rgrandia on 05.10.21.
//
#pragma once
#include <ocs2_core/reference/TargetTrajectories.h>
#include "CatModel2_v2_interface/gait/Gait.h"
#include "CatModel2_v2_interface/gait/ModeSequenceTemplate.h"

namespace ocs2 {
namespace legged_robot {

// Some Dimension Settingss
constexpr size_t NUM_CONTACT_POINTS = 4;
constexpr size_t BASE_COORDINATE_SIZE = 6;
constexpr size_t JOINT_COORDINATE_SIZE = 14;  
constexpr size_t STATE_DIM = 26;  // 26
constexpr size_t INPUT_DIM = 26;    // 26

// Some Type Definitions
template <typename T>
using joint_coordinate_s_t = Eigen::Matrix<T, JOINT_COORDINATE_SIZE, 1>;
using joint_coordinate_t = joint_coordinate_s_t<scalar_t>;

// Struct to store the result of reading a csv file with 1 header and lines of floating point data
struct CsvData {
  std::vector<std::string> header;
  std::vector<vector_t> data;
};

// Read a csv file with 1 header and lines of floating point data
CsvData readCsv(const std::string& fileName);

/**
 * Convert csv data into a motion reference and gait
 *
 * Expects the following header:
 * time, contactflag_LF, contactflag_RF, contactflag_LH, contactflag_RH, 
 * base_positionInWorld_x, base_positionInWorld_y, base_positionInWorld_z, 
 * base_euler_x, base_euler_y, base_euler_z, 
 * jointAngle_Base, jointAngle_Chest
 * jointAngle_LF_HAA, jointAngle_LF_HFE, jointAngle_LF_KFE, jointAngle_RF_HAA, jointAngle_RF_HFE, jointAngle_RF_KFE, 
 * jointAngle_LH_HAA, jointAngle_LH_HFE, jointAngle_LH_KFE, jointAngle_RH_HAA, jointAngle_RH_HFE, jointAngle_RH_KFE,
 *
 * @param fileName : absolute path of the file
 * @param dt : approximate sampling interval. Reference points closer than this dt will be dropped. Set to negative to load all points.
 * @return reference motion and gait
 */
std::pair<ocs2::TargetTrajectories, ModeSequenceTemplate> readMotion(const CsvData& csvData, scalar_t dt = -1.0);

void verifyHeader(const std::vector<std::string>& provided);

void verifyHeaderImpl(const std::vector<std::string>& expected, const std::vector<std::string>& provided);

} // namespace legged_robot
} // namespace ocs2