//
// Created by tlab-uav on 24-9-30.
//

#include "CatModel2_v2_controller/control/TargetManager.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include "CatModel2_v2_controller/control/CtrlComponent.h"

namespace ocs2::legged_robot {
    TargetManager::TargetManager(CtrlComponent &ctrl_component,
                                 const std::shared_ptr<ReferenceManagerInterface> &referenceManagerPtr,
                                 const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                                 const std::string &task_file,
                                 const std::string &reference_file)
        : ctrl_component_(ctrl_component),
          referenceManagerPtr_(referenceManagerPtr) {
        default_joint_state_ = vector_t::Zero(14);
        loadData::loadCppDataType(reference_file, "comHeight", command_height_);
        loadData::loadEigenMatrix(reference_file, "defaultJointState", default_joint_state_);
        loadData::loadCppDataType(task_file, "mpc.timeHorizon", time_to_target_);
        loadData::loadCppDataType(reference_file, "targetRotationVelocity", target_rotation_velocity_);
        loadData::loadCppDataType(reference_file, "targetDisplacementVelocity", target_displacement_velocity_);

        target_pub = node->create_publisher<std_msgs::msg::Float32MultiArray>("target_pose", 10);

        
        auto motionTrajectoriesCallback =
                [this](const ocs2_msgs::msg::MpcTargetTrajectories &msg) {
            std::lock_guard<std::mutex> lock(motionTrajectoryMutex_);
            motionTrajectoryPtr_.reset(new TargetTrajectories(ros_msg_conversions::readTargetTrajectoriesMsg(msg)));
            RCLCPP_INFO(rclcpp::get_logger("target_manager"), "Motion Trajectory is received.!!!!!");
        };

        motion_trajectories_subscriber_ =
                node->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
                    "legged_robot_mpc_target", 1, motionTrajectoriesCallback);

        RCLCPP_INFO(rclcpp::get_logger("target_manager"), "target_manager is ready.");
    }

    void TargetManager::update() {
        TargetTrajectories motion_targetTrajectories;
        {
            std::lock_guard<std::mutex> lock(motionTrajectoryMutex_);
            if (motionTrajectoryPtr_) {
                motion_targetTrajectories = *motionTrajectoryPtr_;
                motionTrajectoryPtr_.reset();
            }else if (motionTrajectoryRunning_) {
                return;
            }else {
                defalut_update();
                return;
            }
        }

        referenceManagerPtr_->setTargetTrajectories(std::move(motion_targetTrajectories));
        motionTrajectoryRunning_ = true;
        
    }

    void TargetManager::defalut_update() {
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
        
        // std::cout << "Target Height: " << target_height << std::endl;

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
            if(std::abs(cmdGoal[0]) <= 1e-4 && std::abs(cmdGoal[1]) <= 1e-4 && std::abs(cmdGoal[2]) <= 1e-4 && std::abs(cmdGoal[3]) <= 1e-4){
            targetPose = lasttargetPose;
            // std::cout << "not change" << std::endl;
            // std::cout << targetPose << std::endl;
        }


    }
    lasttargetPose = targetPose;

        std_msgs::msg::Float32MultiArray target_msg;

    
        target_msg.data = std::vector<float>(targetPose.data(), targetPose.data() + targetPose.size());

        target_pub->publish(target_msg);

        // std::cout << "Z方向的位置: " << currentPose(2) << std::endl;

        const scalar_t targetReachingTime = ctrl_component_.observation_.time + time_to_target_;
        auto trajectories =
                targetPoseToTargetTrajectories(targetPose, ctrl_component_.observation_, targetReachingTime);
        trajectories.stateTrajectory[0].head(3) = cmd_vel_rot;
        trajectories.stateTrajectory[1].head(3) = cmd_vel_rot;

        referenceManagerPtr_->setTargetTrajectories(std::move(trajectories));
    }
}
