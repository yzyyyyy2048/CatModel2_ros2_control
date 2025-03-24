#pragma once


#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <CatModel2_v2_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include "robot_util.hpp"
#include <CatModel2_v2_controller/parameter/parameter_tuning_receiver.hpp>

namespace ocs2::legged_robot {
    class UnittestControl {
        using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
        using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

    public:
        ~UnittestControl() = default;

        UnittestControl(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                const PinocchioEndEffectorKinematics &eeKinematics, const std::shared_ptr<ControlParameter>& control_parameter_ptr);

        vector_t update(const vector_t &rbdStateMeasured,
                                scalar_t currentTime);

    protected:
        void updateMeasured(const vector_t &rbdStateMeasured);

        std::shared_ptr<ControlParameter> control_parameter_ptr_;
        PinocchioInterface pinocchio_interface_measured_;
        CentroidalModelInfo info_;

        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
        CentroidalModelPinocchioMapping mapping_;

        vector_t q_measured_, v_measured_, input_last_;
        matrix_t j_, dj_;
        contact_flag_t contact_flag_{};

        // Task Parameters:
        vector_t torque_limits_;
    };
} // namespace legged