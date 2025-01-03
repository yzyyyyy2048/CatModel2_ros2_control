//
// Created by tlab-uav on 24-9-30.
//

#ifndef TARGETMANAGER_H
#define TARGETMANAGER_H

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <memory>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>

struct CtrlComponent;

namespace ocs2::legged_robot {
    class TargetManager {
    public:
        TargetManager(CtrlComponent &ctrl_component,
                      const std::shared_ptr<ReferenceManagerInterface> &referenceManagerPtr,
                      const rclcpp_lifecycle::LifecycleNode::SharedPtr &node,
                      const std::string& task_file,
                      const std::string& reference_file);

        ~TargetManager() = default;

        void update();
        void defalut_update();

    private:
        TargetTrajectories targetPoseToTargetTrajectories(const vector_t &targetPose,
                                                           const SystemObservation &observation,
                                                           const scalar_t &targetReachingTime) {
            // desired time trajectory
            const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

            // desired state trajectory
            vector_t currentPose = observation.state.segment<6>(6);
            currentPose(2) = command_height_;
            currentPose(4) = 0;
            currentPose(5) = 0;
            vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
            stateTrajectory[0] << vector_t::Zero(6), currentPose, default_joint_state_;
            stateTrajectory[1] << vector_t::Zero(6), targetPose, default_joint_state_;

            // desired input trajectory (just right dimensions, they are not used)
            const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

            return {timeTrajectory, stateTrajectory, inputTrajectory};
        }
        CtrlComponent &ctrl_component_;
        std::shared_ptr<ReferenceManagerInterface> referenceManagerPtr_;

        vector_t default_joint_state_{};
        scalar_t command_height_{};
        scalar_t time_to_target_{};
        scalar_t target_displacement_velocity_;
        scalar_t target_rotation_velocity_;
        
        // 新增成员变量
        bool first_start_ = true;              // 标志是否为第一次启动
        double ramp_duration_ = 10.0;          // 缓慢达到目标高度的持续时间（秒），从5.0增加到10.0
        double ramp_start_time_ = 0.0;         // 缓慢启动的开始时间
        double initial_height_ = 0.0;          // 启动时的初始高度
        double target_height_speed_ = 0.0;     // 高度变化的速度（m/s）

        bool motionTrajectoryRunning_{false};
        std::mutex motionTrajectoryMutex_;
        std::unique_ptr<TargetTrajectories> motionTrajectoryPtr_;
        rclcpp::Subscription<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr motion_trajectories_subscriber_;
    };
}

#endif //TARGETMANAGER_H
