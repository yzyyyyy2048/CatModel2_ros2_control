#include "CatModel2_v2_motion/LoadMotions.h"
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace legged_robot {

CsvData readCsv(const std::string& fileName) {
  CsvData result;

  // Create an input filestream
  std::ifstream myFile(fileName);
  std::string line;

  // Make sure the file is open
  if (!myFile.is_open() || myFile.bad()) {
    throw std::runtime_error("Could not open file: " + fileName);
  }

  {  // Read the column names
    if (std::getline(myFile, line)) {
      std::istringstream ss(line);

      std::string colname;
      // Extract each column name
      while (std::getline(ss, colname, ',')) {
        // Remove additional endline marker generated in python or windows.
        colname.erase(std::remove(colname.begin(), colname.end(), '\r'), colname.end());

        result.header.push_back(colname);
        ss >> std::ws;  // remove white space
      }
    }
  }
  const size_t numCols = result.header.size();

  while (std::getline(myFile, line)) {
    std::istringstream ss(line);
    result.data.emplace_back(vector_t::Zero(numCols));

    int colIdx = 0;
    scalar_t val;
    while (ss >> val) {
      // Add current value to the right column
      result.data.back()[colIdx] = val;
      colIdx++;

      // If the next token is a comma, ignore it and move on
      if (ss.peek() == ',') {
        ss.ignore();
        ss >> std::ws;  // remove white space
      }
    }
  }

  myFile.close();

  return result;
}

std::pair<ocs2::TargetTrajectories, ModeSequenceTemplate> readMotion(const CsvData& csvData, scalar_t dt) {
  verifyHeader(csvData.header);

  const auto numDataPoints = csvData.data.size();
  const scalar_t t0 = csvData.data.front()[0];
  const scalar_t duration = csvData.data.back()[0] - t0;

  const auto getMode = [](const vector_t dataLine) -> size_t {
    const contact_flag_t contactFlags{dataLine[1] > 0.5, dataLine[2] > 0.5, dataLine[3] > 0.5, dataLine[4] > 0.5};
    return stanceLeg2ModeNumber(contactFlags);
  };

  ocs2::TargetTrajectories targetTrajectories;
  targetTrajectories.timeTrajectory.reserve(numDataPoints);
  targetTrajectories.stateTrajectory.reserve(numDataPoints);
  targetTrajectories.inputTrajectory.reserve(numDataPoints);

  std::vector<size_t> modeSequence;
  std::vector<scalar_t> switchingTimes;
  modeSequence.push_back(getMode(csvData.data.front()));
  switchingTimes.push_back(0.0);
  for (const auto& dataLine : csvData.data) {
    const scalar_t t = dataLine[0];

    // Extend gait if the mode changes
    const size_t mode = getMode(dataLine);
    if (mode != modeSequence.back()) {
      switchingTimes.push_back(t - t0);
      modeSequence.push_back(mode);
    } else {
      // Drop a point if dt is smaller than desired
      if (!targetTrajectories.empty() && t < targetTrajectories.timeTrajectory.back() + dt) {
        continue;
      }
    }

    // Time trajectory
    targetTrajectories.timeTrajectory.push_back(t);

    size_t colId = 5;  // after time and 4 contact flags
    const vector3_t basePositionInWorld = dataLine.segment(colId, 3);
    colId += 3;
    const vector3_t eulerXYZ(dataLine[colId], dataLine[colId + 1], dataLine[colId + 2]);
    colId += 3;
    const joint_coordinate_t jointPositions = dataLine.segment(colId, JOINT_COORDINATE_SIZE);
    colId += JOINT_COORDINATE_SIZE;

    // State trajectory
    targetTrajectories.stateTrajectory.push_back(vector_t(STATE_DIM));
    targetTrajectories.stateTrajectory.back() << vector_t::Zero(6), basePositionInWorld, eulerXYZ[2], 0, 0, -0.10, 0.72, -1.44, 0.10, 0.72, -1.44, 0, 0, -0.10, 0.72, -1.44, 0.1, 0.72, -1.44;

    // Input trajectory
    targetTrajectories.inputTrajectory.push_back(vector_t(INPUT_DIM));
  }

  // 删除一开始的stance
  modeSequence.erase(modeSequence.begin());
  ModeSequenceTemplate gait(switchingTimes, modeSequence);
  return {targetTrajectories, gait};
}

void verifyHeader(const std::vector<std::string>& provided) {
  const std::vector<std::string> expectedHeader = {"time",
                                                   "contactflag_LF",
                                                   "contactflag_RF",
                                                   "contactflag_LH",
                                                   "contactflag_RH",
                                                   "base_positionInWorld_x",
                                                   "base_positionInWorld_y",
                                                   "base_positionInWorld_z",
                                                   "base_euler_x",
                                                   "base_euler_y",
                                                   "base_euler_z",
                                                   "jointAngle_LF_HAA",
                                                   "jointAngle_LF_HFE",
                                                   "jointAngle_LF_KFE",
                                                   "jointAngle_RF_HAA",
                                                   "jointAngle_RF_HFE",
                                                   "jointAngle_RF_KFE",
                                                   "jointAngle_pitch",
                                                   "jointAngle_yaw",
                                                   "jointAngle_LH_HAA",
                                                   "jointAngle_LH_HFE",
                                                   "jointAngle_LH_KFE",
                                                   "jointAngle_RH_HAA",
                                                   "jointAngle_RH_HFE",
                                                   "jointAngle_RH_KFE",
                                                  };
  verifyHeaderImpl(expectedHeader, provided);
}

void verifyHeaderImpl(const std::vector<std::string>& expected, const std::vector<std::string>& provided) {
  // Check header
  if (provided.size() != expected.size()) {
    throw std::runtime_error("Incorrect amount of columns. Expected: " + std::to_string(expected.size()) + ", but got " +
                             std::to_string(provided.size()));
  }
  for (size_t i = 0; i < expected.size(); ++i) {
    if (provided[i] != expected[i]) {
      throw std::runtime_error("Incorrect header of column " + std::to_string(i) + ", expected: " + expected[i] + ", but got " +
                               provided[i]);
    }
  }
}


} // namespace legged_robot
} // namespace ocs2

