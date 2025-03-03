//
// Created by qiayuan on 2021/11/15.
//

#include "CatModel2_v2_controller/estimator/StateEstimateBase.h"

#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <CatModel2_v2_interface/common/Types.h>
#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

#include <memory>
#include "CatModel2_v2_controller/control/CtrlComponent.h"

namespace ocs2::legged_robot {

    StateEstimateBase::StateEstimateBase(PinocchioInterface pinocchio_interface, CentroidalModelInfo info,
                                         const PinocchioEndEffectorKinematics &ee_kinematics,
                                         CtrlComponent &ctrl_component,
                                         rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ctrl_component_(ctrl_component),
          pinocchio_interface_(std::move(pinocchio_interface)),
          info_(std::move(info)),
          ee_kinematics_(ee_kinematics.clone()),
          rbd_state_(vector_t::Zero(2 * info_.generalizedCoordinatesNum)), node_(std::move(node)) {
        odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        pose_pub_ = node_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("pose", 10);

        ground_truth_pos_pub = node_->create_publisher<std_msgs::msg::Float32MultiArray>("ground_truth_pos", 10);
        ground_truth_vel_pub = node_->create_publisher<std_msgs::msg::Float32MultiArray>("ground_truth_vel", 10);
    }

    void StateEstimateBase::updateJointStates() {
        const size_t size = ctrl_component_.joint_effort_state_interface_.size();
        vector_t joint_pos(size), joint_vel(size);

        for (int i = 0; i < size; i++) {
            joint_pos(i) = ctrl_component_.joint_position_state_interface_[i].get().get_value();
            joint_vel(i) = ctrl_component_.joint_velocity_state_interface_[i].get().get_value();
        }

        rbd_state_.segment(6, info_.actuatedDofNum) = joint_pos;
        rbd_state_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum) = joint_vel;
    }

    void StateEstimateBase::updateContact() {
        const size_t size = ctrl_component_.foot_force_state_interface_.size();
        for (int i = 0; i < size; i++) {
            // contact_flag_[i] = ctrl_component_.foot_force_state_interface_[i].get().get_value() > 0.1;
            // std::cout << "contact_flag_[" << i << "]: " << contact_flag_[i]<< std::endl;
        }
    }

    void StateEstimateBase::updateImu() {
        quat_ = {
            ctrl_component_.imu_state_interface_[0].get().get_value(),
            ctrl_component_.imu_state_interface_[1].get().get_value(),
            ctrl_component_.imu_state_interface_[2].get().get_value(),
            ctrl_component_.imu_state_interface_[3].get().get_value()
        };

        angular_vel_local_ = {
            ctrl_component_.imu_state_interface_[4].get().get_value(),
            ctrl_component_.imu_state_interface_[5].get().get_value(),
            ctrl_component_.imu_state_interface_[6].get().get_value()
        };

        linear_accel_local_ = {
            ctrl_component_.imu_state_interface_[7].get().get_value(),
            ctrl_component_.imu_state_interface_[8].get().get_value(),
            ctrl_component_.imu_state_interface_[9].get().get_value()
        };

        groundTruth_pos = {
            ctrl_component_.odom_state_interface_[0].get().get_value(),
            ctrl_component_.odom_state_interface_[1].get().get_value(),
            ctrl_component_.odom_state_interface_[2].get().get_value()
        };

        groundTruth_vel = {
            ctrl_component_.odom_state_interface_[3].get().get_value(),
            ctrl_component_.odom_state_interface_[4].get().get_value(),
            ctrl_component_.odom_state_interface_[5].get().get_value()
        };

        std_msgs::msg::Float32MultiArray groundTruth_pos_msg;
        groundTruth_pos_msg.data = std::vector<float>(groundTruth_pos.begin(), groundTruth_pos.end());
        ground_truth_pos_pub->publish(groundTruth_pos_msg);

        std_msgs::msg::Float32MultiArray groundTruth_vel_msg;
        groundTruth_vel_msg.data = std::vector<float>(groundTruth_vel.begin(), groundTruth_vel.end());
        ground_truth_vel_pub->publish(groundTruth_vel_msg);
        

        // orientationCovariance_ = orientationCovariance;
        // angularVelCovariance_ = angularVelCovariance;
        // linearAccelCovariance_ = linearAccelCovariance;

        zyx_offset_(0) = imuBiasYaw_;
        zyx_offset_(1) = imuBiasPitch_;
        zyx_offset_(2) = imuBiasRoll_;

        const vector3_t zyx = quatToZyx(quat_) + zyx_offset_;
        // std::cout << "zyx 的值是: " << zyx << std::endl;
        const vector3_t angularVelGlobal = getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(
            zyx, getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(quatToZyx(quat_), angular_vel_local_));
        updateAngular(zyx, angularVelGlobal);
    }

    void StateEstimateBase::updateAngular(const vector3_t &zyx, const vector_t &angularVel) {
        rbd_state_.segment<3>(0) = zyx;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum) = angularVel;
    }

    void StateEstimateBase::updateLinear(const vector_t &pos, const vector_t &linearVel) {
        rbd_state_.segment<3>(3) = pos;
        rbd_state_.segment<3>(info_.generalizedCoordinatesNum + 3) = linearVel;
    }

    void StateEstimateBase::publishMsgs(const nav_msgs::msg::Odometry &odom) const {
        rclcpp::Time time = odom.header.stamp;
        odom_pub_->publish(odom);

        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header = odom.header;
        pose.pose.pose = odom.pose.pose;
        pose_pub_->publish(pose);
    }
} // namespace legged
