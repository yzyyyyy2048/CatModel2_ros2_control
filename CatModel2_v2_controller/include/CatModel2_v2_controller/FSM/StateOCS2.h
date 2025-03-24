
#ifndef STATEOCS2_H
#define STATEOCS2_H

#include <SafetyChecker.h>
#include <ocs2_centroidal_model/CentroidalModelRbdConversions.h>
#include <ocs2_core/misc/Benchmark.h>
#include <CatModel2_v2_dummy/visualization/CatModel2Visualizer.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_msgs/msg/detail/mpc_observation__struct.hpp>
#include <CatModel2_v2_controller/control/TargetManager.h>
#include <CatModel2_v2_controller/estimator/StateEstimateBase.h>
#include <CatModel2_v2_controller/interface/LeggedInterface.h>
#include <CatModel2_v2_controller/wbc/WbcBase.h>
#include <rclcpp/duration.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include "controller_common/FSM/FSMState.h"
#include <CatModel2_v2_controller/plotdata/my_publisher.hpp>
#include <CatModel2_v2_controller/parameter/parameter_tuning_receiver.hpp>
#include "CatModel2_v2_controller/unittest/unittest_control.hpp"
#include "CatModel2_v2_controller/vmc/vm_control.hpp"

namespace ocs2 {
    class MPC_MRT_Interface;
    class MPC_BASE;
    class PinocchioEndEffectorKinematics;
}

struct CtrlComponent;

namespace ocs2::legged_robot {
    class StateOCS2 final : public FSMState {
    public:
        StateOCS2(CtrlInterfaces &ctrl_interfaces,
                  const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> &node,
                  const std::string &package_share_directory,
                  const std::vector<std::string> &joint_names,
                  const std::vector<std::string> &feet_names,
                  double default_kp,
                  double default_kd
        );

        void enter() override;

        void run(const rclcpp::Time &time,
                 const rclcpp::Duration &period) override;

        void exit() override;

        FSMStateName checkChange() override;

        void setupStateEstimate(const std::string &estimator_type);

    private:
        void updateStateEstimation(const rclcpp::Duration &period);

        void updateGaitPhase(feet_array_t<scalar_t>& swing_phase, feet_array_t<scalar_t>& swing_time);

        void setupParameter();

        void setupLeggedInterface();

        void setupMpc();

        void setupMrt();

        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
        rclcpp::Publisher<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_publisher_;
        std::shared_ptr<StateEstimateBase> estimator_;
        std::shared_ptr<TargetManager> target_manager_;
        std::shared_ptr<LeggedRobotVisualizer> visualizer_;

        std::shared_ptr<LeggedInterface> legged_interface_;
        std::shared_ptr<PinocchioEndEffectorKinematics> eeKinematicsPtr_;

        // Unittest control
        std::shared_ptr<UnittestControl> unittest_control_;

        // VM control
        std::shared_ptr<VMControl> vm_control_;

        // Whole Body Control
        std::shared_ptr<WbcBase> wbc_;
        std::shared_ptr<SafetyChecker> safety_checker_;

        // Nonlinear MPC
        std::shared_ptr<MPC_BASE> mpc_;
        std::shared_ptr<MPC_MRT_Interface> mpc_mrt_interface_;

        std::shared_ptr<CentroidalModelRbdConversions> rbd_conversions_;

        // unit test
        std::shared_ptr<MyPublisher> my_pub_ptr_;

        std::shared_ptr<ControlParameter> control_parameter_ptr_;

        std::thread control_parameter_thread_;

        std::shared_ptr<PinocchioInterface> pinocchio_interface_pub_ptr_;

        SystemObservation observation_;

        vector_t measured_rbd_state_;
        std::thread mpc_thread_;
        std::atomic_bool controller_running_{}, mpc_running_{};
        benchmark::RepeatedTimer mpc_timer_;
        benchmark::RepeatedTimer wbc_timer_;

        std::string task_file_;
        std::string urdf_file_;
        std::string reference_file_;
        std::string gait_file_;

        std::vector<std::string> joint_names_;
        std::vector<std::string> feet_names_;
        double default_kp_;
        double default_kd_;

        bool verbose_;
        bool mpc_need_updated_;
        vector_t optimized_state_, optimized_input_;
    };
}


#endif //STATEOCS2_H
