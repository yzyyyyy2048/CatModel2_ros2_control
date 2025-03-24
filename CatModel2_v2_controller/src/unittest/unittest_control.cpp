//
// Created by qiayuan on 2022/7/1.
//

#include "CatModel2_v2_controller/unittest/unittest_control.hpp"

#include <utility>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/LoadData.h>
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace ocs2::legged_robot {
    UnittestControl::UnittestControl(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics &eeKinematics, const std::shared_ptr<ControlParameter>& control_parameter_ptr)
        : pinocchio_interface_measured_(pinocchioInterface),
          info_(std::move(info)),
          ee_kinematics_(eeKinematics.clone()),
          mapping_(info_),
          input_last_(vector_t::Zero(info_.inputDim)) {
        control_parameter_ptr_ = control_parameter_ptr;
        q_measured_ = vector_t(info_.generalizedCoordinatesNum);
        v_measured_ = vector_t(info_.generalizedCoordinatesNum);
    }

    vector_t UnittestControl::update(const vector_t &rbdStateMeasured,
                             scalar_t currentTime) {

        for (int leg = 0; leg < contact_flag_.size(); ++leg) {
            contact_flag_[leg] = true;
        }
        updateMeasured(rbdStateMeasured);

        PARAMETER_GET_INT(swing_test_leg,control_parameter_ptr_);
        PARAMETER_GET_INT(swing_test_type,control_parameter_ptr_);
        PARAMETER_GET_FLOAT(swing_test_K,control_parameter_ptr_);
        PARAMETER_GET_FLOAT(swing_test_Freq,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(swing_test_pos,control_parameter_ptr_);

        PARAMETER_GET_FLOAT(swing_test_joint_kp,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(swing_test_joint_pos,control_parameter_ptr_);

        PARAMETER_GET_FLOAT_VECTOR(kp_lin_foot_vmc,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(kd_lin_foot_vmc,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(swing_inertia_vmc,control_parameter_ptr_);

        vector3_t pos_target = vector3_t::Zero();

        scalar_t PI = 3.1415926;
        scalar_t sin_swing = swing_test_K * std::sin( 2.0 * PI * swing_test_Freq * currentTime );
        scalar_t cos_swing = swing_test_K * std::cos( 2.0 * PI * swing_test_Freq * currentTime );

        switch (swing_test_type)
        {
        case 0 /*lin x*/:
            pos_target = vector3_t::Zero();
            pos_target[0] = sin_swing;
            break;
        case 1 /*lin y*/:
            pos_target = vector3_t::Zero();
            pos_target[1] = sin_swing;
            break;
        case 2 /*lin z*/:
            pos_target = vector3_t::Zero();
            pos_target[2] = sin_swing;
            break;  
        case 3 /*lin x y*/:
            pos_target = vector3_t::Zero();
            pos_target[0] = sin_swing;
            pos_target[1] = cos_swing;
            break;
        case 4 /*lin y z*/:
            pos_target = vector3_t::Zero();
            pos_target[1] = sin_swing;
            pos_target[2] = cos_swing;
            break;
        case 5 /*lin x z*/:
            pos_target = vector3_t::Zero();
            pos_target[0] = sin_swing;
            pos_target[2] = cos_swing;
            break;

        default:
            break;
        }

        const auto &model = pinocchio_interface_measured_.getModel();
        const auto &data = pinocchio_interface_measured_.getData();

        std::vector<std::string> hip_names = {"LF_HAA", "RF_HAA", "LH_HAA", "RH_HAA"};
        std::vector<int> hip_ids;
        hip_ids.resize(hip_names.size());
        vector_t torque_swing = vector_t::Zero(info_.actuatedDofNum);

        torque_swing = swing_test_joint_kp * ( swing_test_joint_pos - q_measured_.segment(6,info_.actuatedDofNum) );

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_measured_);
        std::vector<vector3_t> foot_pos_world = ee_kinematics_->getPosition(vector_t());
        std::vector<vector3_t> foot_vel_world = ee_kinematics_->getVelocity(vector_t(), vector_t());

        matrix_t R_base = rotz(q_measured_(3)) * roty(q_measured_(4)) * rotx(q_measured_(5));
        // std::cout << "R_base = \n" << R_base << std::endl;
        for (int leg = 0; leg < hip_names.size(); ++leg) {
            auto frame_id = model.getFrameId(hip_names[leg]);
            hip_ids[leg] = model.idx_qs[model.getJointId(hip_names[leg])];

            // model.parents[model.getJointId(hip_names[leg])];
            // auto base_name = model.names[model.parents[model.getJointId(hip_names[leg])]];
            // model.jointPlacements[model.getJointId(hip_names[leg])].trans()
            // std::cout << "base_name_" << leg << " = \n" << data.oMf[model.getFrameId(base_name)].rotation() << std::endl;
            
            matrix_t j_leg = R_base.transpose() * j_.block<3, 3>(3*leg, hip_ids[leg]);
            // std::cout << hip_names[leg] << "_id = " << hip_ids[leg] << std::endl;
            vector_t hip_pos_world = data.oMf[frame_id].translation();
            pinocchio::Motion v = pinocchio::getFrameVelocity(model, data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
            vector_t hip_vel_world = v.linear();            

            vector_t pos_base = R_base.transpose() * (foot_pos_world[leg] - hip_pos_world);
            vector_t vel_base = R_base.transpose() * (foot_vel_world[leg] - hip_vel_world);

            vector_t pos_ref_base = swing_test_pos.segment<3>(3*leg) + pos_target;

            vector_t acc_lin_base = (kp_lin_foot_vmc.array() * (pos_ref_base - pos_base).array()) - (kd_lin_foot_vmc.array() * (vel_base).array());
            vector_t ddq = Eigen::JacobiSVD<matrix_t>(j_leg, Eigen::ComputeFullU | Eigen::ComputeFullV).solve( acc_lin_base );

            if (leg == swing_test_leg) {
            torque_swing.segment(hip_ids[leg]-hip_ids[0], 3) = swing_inertia_vmc.array() * ddq.array();    
            }
        }
        return torque_swing;
    }

    void UnittestControl::updateMeasured(const vector_t &rbdStateMeasured) {
        q_measured_.head<3>() = rbdStateMeasured.segment<3>(3);
        q_measured_.segment<3>(3) = rbdStateMeasured.head<3>();
        q_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(6, info_.actuatedDofNum);
        v_measured_.head<3>() = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
        v_measured_.segment<3>(3) = getEulerAnglesZyxDerivativesFromGlobalAngularVelocity<scalar_t>(
            q_measured_.segment<3>(3), rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum));
        v_measured_.tail(info_.actuatedDofNum) = rbdStateMeasured.segment(
            info_.generalizedCoordinatesNum + 6, info_.actuatedDofNum);

        const auto &model = pinocchio_interface_measured_.getModel();
        auto &data = pinocchio_interface_measured_.getData();

        // auto frame_id = model.getFrameId("trunk");
        
        // For floating base EoM task
        forwardKinematics(model, data, q_measured_, v_measured_);
        computeJointJacobians(model, data);
        updateFramePlacements(model, data);
        crba(model, data, q_measured_);
        data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
        nonLinearEffects(model, data, q_measured_, v_measured_);
        j_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            getFrameJacobian(model, data, info_.endEffectorFrameIndices[i], pinocchio::LOCAL_WORLD_ALIGNED,
                             jac);
            j_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }

        // For not contact motion task
        computeJointJacobiansTimeVariation(model, data, q_measured_, v_measured_);
        dj_ = matrix_t(3 * info_.numThreeDofContacts, info_.generalizedCoordinatesNum);
        for (size_t i = 0; i < info_.numThreeDofContacts; ++i) {
            Eigen::Matrix<scalar_t, 6, Eigen::Dynamic> jac;
            jac.setZero(6, info_.generalizedCoordinatesNum);
            getFrameJacobianTimeVariation(model, data, info_.endEffectorFrameIndices[i],
                                          pinocchio::LOCAL_WORLD_ALIGNED, jac);
            dj_.block(3 * i, 0, 3, info_.generalizedCoordinatesNum) = jac.template topRows<3>();
        }

        // matrix_t R_body = data.oMf[frame_id].rotation();
        // matrix_t R_body_test = rotz(q_measured_(3)) * roty(q_measured_(4)) * rotx(q_measured_(5));

        // std::cout << "R_body\n" << R_body << std::endl;
        // std::cout << "R_body_test\n" << R_body_test << std::endl;

        // rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum) base ang vel world

    }
} // namespace legged
