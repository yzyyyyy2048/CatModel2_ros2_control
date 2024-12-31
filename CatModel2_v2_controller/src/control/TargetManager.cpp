//
// Created by tlab-uav on 24-9-30.
//

#include "CatModel2_v2_controller/control/TargetManager.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "CatModel2_v2_controller/control/CtrlComponent.h"



namespace ocs2::legged_robot {
    TargetManager::TargetManager(CtrlComponent &ctrl_component,
                                 const std::shared_ptr<ReferenceManagerInterface> &referenceManagerPtr,
                                 const std::string &task_file,
                                 const std::string &reference_file)
        : ctrl_component_(ctrl_component),
          referenceManagerPtr_(referenceManagerPtr) {
        default_joint_state_ = vector_t::Zero(12);
        loadData::loadCppDataType(reference_file, "comHeight", command_height_);
        loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joint_state_);
        loadData::loadCppDataType(task_file, "mpc.timeHorizon", time_to_target_);
        loadData::loadCppDataType(reference_file, "targetRotationVelocity", target_rotation_velocity_);
        loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", target_displacement_velocity_);
        
    //     if(flag_setup_){  // Setup
    //     if (z_coeff_(0) < 1.0){
    //       z_coeff_(0) += 0.001; //Will take 200 loops to stand up 
    //       const auto trajectories = setup_behavior_(z_coeff_, latestObservation_);
    //     //   targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    //     }
    //     else {
    //       flag_setup_ = false;  // Allow command by cmd_vel & goal
    //     //   ROS_INFO("--> Robot is up");
    //     }
    //   }
    // };
    }
    
    // TargetTrajectories setupBehavior(const vector_t& z_coeff, const SystemObservation& observation) 
    // {
    // const vector_t current_pose = observation.state.segment<6>(6);
    // const vector_t target_pose = [&]() {
    //     vector_t target(6);
    //     target(0) = current_pose(0);
    //     target(1) = current_pose(1);
    //     target(2) = command_height_*z_coeff(0);
    //     target(3) = current_pose(3);
    //     target(4) = 0;
    //     target(5) = 0;
    //     return target;
    // }();
    // // Increase target_reaching_time didn't work.
    // const scalar_t target_reaching_time = observation.time + estimateTimeToTarget(target_pose - current_pose);
    // // ROS_INFO("z_target = %.4f\n", COM_HEIGHT*z_coeff(0)); // Print z value each time robot goes up to stand
    // return targetPoseToTargetTrajectories(target_pose, observation, target_reaching_time);
    // }





void TargetManager::update() {
    vector_t cmdGoal = vector_t::Zero(6);
    cmdGoal[0] = ctrl_component_.control_inputs_.ly * target_displacement_velocity_;
    cmdGoal[1] = -ctrl_component_.control_inputs_.lx * target_displacement_velocity_;
    cmdGoal[2] = ctrl_component_.control_inputs_.ry;
    cmdGoal[3] = -ctrl_component_.control_inputs_.rx * target_rotation_velocity_;

    const vector_t currentPose = ctrl_component_.observation_.state.segment<6>(6);

    const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
    vector_t cmd_vel_rot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdGoal.head(3);

    vector_t targetPose(6);

    if (first_start_) {
        // 记录初始高度和启动时间
        ramp_start_time_ = ctrl_component_.observation_.time;
        initial_height_ = currentPose(2);
        // 计算高度变化的速度
        target_height_speed_ = (command_height_ - initial_height_) / ramp_duration_;

        first_start_ = false;

        std::cout << "Starting ramp-up: Initial height = " << initial_height_
          << ", Target height = " << command_height_
          << ", Ramp duration = " << ramp_duration_
          << ", Ramp_start_time_ = " << ramp_start_time_
          << " seconds, Target height speed = " << target_height_speed_
          << " m/s" << std::endl;
    }

    double elapsed_time = ctrl_component_.observation_.time - ramp_start_time_;
    if (elapsed_time < ramp_duration_) {
        // 计算目标高度：初始高度 + 速度 * 经过的时间
        double target_height = initial_height_ + target_height_speed_ * elapsed_time;
       
       std::cout << "Target Height: " << target_height << std::endl;

        targetPose(0) = currentPose(0);
        targetPose(1) = currentPose(1);
        targetPose(2) = target_height;
        targetPose(3) = currentPose(3);
        targetPose(4) = 0;
        targetPose(5) = 0;
    }
    else {
        // std::cout << "Ramp-up finished, setting target height to " << command_height_ << std::endl;
        // 达到缓慢启动时间后，设置为正常目标高度
        targetPose = [&]() {
            vector_t target(6);
            target(0) = currentPose(0) + cmd_vel_rot(0) * time_to_target_;
            target(1) = currentPose(1) + cmd_vel_rot(1) * time_to_target_;
            target(2) = command_height_ + ctrl_component_.control_inputs_.ry * command_height_ * 3.0;
            target(3) = currentPose(3) + cmdGoal(3) * time_to_target_;
            target(4) = 0;
            target(5) = 0;

            // std::cout << "Z方向期望位置: " << target(2) << std::endl;

            return target;
        }();
    }

    // std::cout << "Z方向的位置: " << currentPose(2) << std::endl;

    const scalar_t targetReachingTime = ctrl_component_.observation_.time + time_to_target_;
    auto trajectories =
            targetPoseToTargetTrajectories(targetPose, ctrl_component_.observation_, targetReachingTime);
    trajectories.stateTrajectory[0].head(3) = cmd_vel_rot;
    trajectories.stateTrajectory[1].head(3) = cmd_vel_rot;

    referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
}

}
