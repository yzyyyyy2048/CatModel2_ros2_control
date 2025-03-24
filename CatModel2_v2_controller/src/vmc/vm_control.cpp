//
// Created by qiayuan on 2022/7/1.
//

#include "CatModel2_v2_controller/vmc/vm_control.hpp"

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
    VMControl::VMControl(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                     const PinocchioEndEffectorKinematics &eeKinematics, const std::shared_ptr<ControlParameter>& control_parameter_ptr)
        : pinocchio_interface_measured_(pinocchioInterface),
          info_(std::move(info)),
          ee_kinematics_(eeKinematics.clone()),
          mapping_(info_),
          input_last_(vector_t::Zero(info_.inputDim)) {
        control_parameter_ptr_ = control_parameter_ptr;
        q_measured_ = vector_t(info_.generalizedCoordinatesNum);
        v_measured_ = vector_t(info_.generalizedCoordinatesNum);

        qpoases_interface_ = std::make_shared<QPoasesInterface>();
        five_order_line_ = std::make_shared<FiveOrderLine<scalar_t>>();
        is_first_control_ = true;
        foot_pos_start_hip_.resize(info_.numThreeDofContacts);
        foot_pos_final_hip_.resize(info_.numThreeDofContacts);
        for (int leg = 0; leg < info_.numThreeDofContacts; ++leg) {
            foot_pos_start_hip_[leg].setZero();
            foot_pos_final_hip_[leg].setZero();
        }
    }

    vector_t VMControl::update(const vector_t &rbdStateMeasured,
                             scalar_t currentTime, feet_array_t<scalar_t> swing_phase, feet_array_t<scalar_t> swing_time, vector_t cmd) {

        for (int leg = 0; leg < contact_flag_.size(); ++leg) {
            if (swing_phase[leg] > 0.999) {
                contact_flag_[leg] = true;
            }
            else {
                contact_flag_[leg] = false;
            }
        }

        PARAMETER_GET_FLOAT_VECTOR(foot_position_stand,control_parameter_ptr_);
        PARAMETER_GET_FLOAT(capture_alpha_swing_x,control_parameter_ptr_);
        PARAMETER_GET_FLOAT(capture_alpha_swing_y,control_parameter_ptr_);
        PARAMETER_GET_FLOAT(alpha_swing_x,control_parameter_ptr_);
        PARAMETER_GET_FLOAT(alpha_swing_y,control_parameter_ptr_);

        PARAMETER_GET_FLOAT(foot_height,control_parameter_ptr_);
        PARAMETER_GET_FLOAT(foot_stamp,control_parameter_ptr_);

        PARAMETER_GET_FLOAT(root_height,control_parameter_ptr_);

        PARAMETER_GET_FLOAT_VECTOR(kp_lin_foot_vmc,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(kd_lin_foot_vmc,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(swing_inertia_vmc,control_parameter_ptr_);

        PARAMETER_GET_FLOAT_VECTOR(weight_root_vmc,control_parameter_ptr_);
        PARAMETER_GET_FLOAT(weight_force_vmc,control_parameter_ptr_);

        PARAMETER_GET_FLOAT_VECTOR(kp_lin_root_vmc,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(kd_lin_root_vmc,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(kp_ang_root_vmc,control_parameter_ptr_);
        PARAMETER_GET_FLOAT_VECTOR(kd_ang_root_vmc,control_parameter_ptr_);

        updateMeasured(rbdStateMeasured);

        const auto &model = pinocchio_interface_measured_.getModel();
        const auto &data = pinocchio_interface_measured_.getData();

        std::vector<std::string> hip_names = {"FL_hip_joint", "FR_hip_joint", "RL_hip_joint", "RR_hip_joint"};
        std::vector<int> hip_ids;
        hip_ids.resize(hip_names.size());
        vector_t torque_swing = vector_t::Zero(info_.actuatedDofNum);

        ee_kinematics_->setPinocchioInterface(pinocchio_interface_measured_);
        std::vector<vector3_t> foot_pos_world = ee_kinematics_->getPosition(vector_t());
        std::vector<vector3_t> foot_vel_world = ee_kinematics_->getVelocity(vector_t(), vector_t());

        vector3_t base_pos_world = rbdStateMeasured.segment<3>(3);
        vector3_t base_lin_vel_world = rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum + 3);
        matrix3_t base_So3_world = rotz(q_measured_(3)) * roty(q_measured_(4)) * rotx(q_measured_(5));
        vector3_t base_ang_vel_body = base_So3_world.transpose() * rbdStateMeasured.segment<3>(info_.generalizedCoordinatesNum);

        vector3_t base_pos_target_world(base_pos_world[0], base_pos_world[1], root_height);
        vector3_t base_lin_vel_target_world = vector3_t::Zero();
        matrix3_t base_So3_target_world = rotz(q_measured_(3));
        vector3_t base_ang_vel_target_world = vector3_t::Zero();

        base_lin_vel_target_world = base_So3_target_world * cmd.segment<3>(0);
        base_lin_vel_target_world[2] = 0.0;

        base_ang_vel_target_world[2] = cmd(3);

        // matrix_t R_base = rotz(q_measured_(3)) * roty(q_measured_(4)) * rotx(q_measured_(5));
        // // std::cout << "R_base = \n" << R_base << std::endl;
        for (int leg = 0; leg < hip_names.size(); ++leg) {
            auto frame_id = model.getFrameId(hip_names[leg]);
            hip_ids[leg] = model.idx_qs[model.getJointId(hip_names[leg])];

            // update foot pos

            // model.parents[model.getJointId(hip_names[leg])];
            auto base_name = model.names[model.parents[model.getJointId(hip_names[leg])]];
            // model.jointPlacements[model.getJointId(hip_names[leg])].trans()
            // std::cout << "base_name_" << leg << " = \n" << data.oMf[model.getFrameId(base_name)].rotation() << std::endl;
            matrix3_t R_base = data.oMf[model.getFrameId(base_name)].rotation();
            vector3_t T_base = data.oMf[model.getFrameId(base_name)].translation();

            matrix_t j_leg = R_base.transpose() * j_.block<3, 3>(3*leg, hip_ids[leg]);
            // std::cout << hip_names[leg] << "_id = " << hip_ids[leg] << std::endl;
            vector3_t hip_pos_world = data.oMf[frame_id].translation();
            pinocchio::Motion v = pinocchio::getFrameVelocity(model, data, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
            vector3_t hip_vel_world = v.linear();            

            vector3_t pos_base = R_base.transpose() * (foot_pos_world[leg] - hip_pos_world);
            vector3_t vel_base = R_base.transpose() * (foot_vel_world[leg] - hip_vel_world);

            // plan
            if (is_first_control_) {
                foot_pos_start_hip_[leg] = pos_base;
                foot_pos_final_hip_[leg] = pos_base;
            }

            vector3_t pos_ref_base = pos_base;

            if (!contact_flag_[leg]) {
                matrix3_t R_yaw = rotz(q_measured_(3));
      
                // vector3_t foot_pos_swing = foot_position_stand.segment<3>(3 * leg);
                vector3_t base_lin_vel_yaw = R_yaw.transpose() * base_lin_vel_world;
                vector3_t base_lin_vel_cmd = R_yaw.transpose() * base_lin_vel_target_world;
                vector3_t foot_pos_swing_yaw = (vector3_t(capture_alpha_swing_x, capture_alpha_swing_y, 0.0).array() * (std::sqrt(root_height / 9.8)) * (base_lin_vel_yaw - base_lin_vel_cmd).array()).matrix();
                foot_pos_swing_yaw += (vector3_t(alpha_swing_x, alpha_swing_y, 0.0).array() * ( (swing_time[leg]) * base_lin_vel_yaw).array()).matrix();
                foot_pos_swing_yaw += (1.0 - swing_phase[leg]) * swing_time[leg] * base_lin_vel_cmd;

                vector3_t foot_pos_swing_world = R_yaw * foot_pos_swing_yaw + rotz((1.0 - swing_phase[leg]) * swing_time[leg] * base_ang_vel_target_world[2]) * R_base * foot_position_stand.segment<3>(3 * leg);

                foot_pos_swing_world[2] = -hip_pos_world[2] - foot_stamp;

                foot_pos_final_hip_[leg] = R_base.transpose() * foot_pos_swing_world;
                
                scalar_t T_swing = swing_time[leg];
                scalar_t t_swing = swing_time[leg] * swing_phase[leg];
                //  x
                five_order_line_->set(foot_pos_start_hip_[leg][0], 0.0, 0.0, foot_pos_final_hip_[leg][0], 0.0, 0.0, T_swing, t_swing);
                pos_ref_base[0] = five_order_line_->X();

                //  y
                five_order_line_->set(foot_pos_start_hip_[leg][1], 0.0, 0.0, foot_pos_final_hip_[leg][1], 0.0, 0.0, T_swing, t_swing);
                pos_ref_base[1] = five_order_line_->X();

                //  z
                if (t_swing < 0.5*T_swing) {
                    five_order_line_->set(foot_pos_start_hip_[leg][2], 0.0, 0.0, foot_pos_final_hip_[leg][2] + foot_height, 0.0, 0.0, 0.5*T_swing, t_swing);
                    pos_ref_base[2] = five_order_line_->X();
                }
                else {
                    five_order_line_->set(foot_pos_final_hip_[leg][2] + foot_height, 0.0, 0.0, foot_pos_final_hip_[leg][2], 0.0, 0.0, 0.5*T_swing, t_swing - 0.5*T_swing);
                    pos_ref_base[2] = five_order_line_->X();
                }

            } 
            else {
                foot_pos_start_hip_[leg] = pos_base;
                foot_pos_final_hip_[leg] = pos_base;
            }
            // !plan

            vector_t acc_lin_base = (kp_lin_foot_vmc.array() * (pos_ref_base - pos_base).array()) - (kd_lin_foot_vmc.array() * (vel_base).array());
            vector_t ddq = Eigen::JacobiSVD<matrix_t>(j_leg, Eigen::ComputeFullU | Eigen::ComputeFullV).solve( acc_lin_base );
            torque_swing.segment(3*leg, 3) = swing_inertia_vmc.array() * ddq.array();
        }

        is_first_control_ = false;

        vector3_t base_lin_acc_world = (kp_lin_root_vmc.array()*(base_pos_target_world - base_pos_world).array()) + 
                                       (kd_lin_root_vmc.array()*(base_lin_vel_target_world - base_lin_vel_world).array());

        base_lin_acc_world[2] += data.M(0, 0) * 9.81;

        matrix3_t base_So3_target_diff_body = base_So3_world.transpose() * base_So3_target_world;

        vector3_t base_ang_vel_target_body = base_So3_world.transpose() * base_ang_vel_target_world;

        vector3_t base_ang_acc_body = (kp_ang_root_vmc.array() * logm(base_So3_target_diff_body).array()) + 
                                      (kd_ang_root_vmc.array() * (base_ang_vel_target_body - base_ang_vel_body).array());

        vector_t base_acc = vector_t::Zero(6);
        base_acc << base_lin_acc_world, base_ang_acc_body;

        // set vmc task
        matrix_t Root_Force_Transition = matrix_t::Zero(6, 3 * contact_flag_.size());
        matrix_t A_vmc  = matrix_t::Zero(5 * contact_flag_.size(), 3 * contact_flag_.size());
        vector_t ub_vmc  = vector_t::Zero(5 * contact_flag_.size());
        vector_t lb_vmc  = vector_t::Zero(5 * contact_flag_.size());

        scalar_t mu = 0.5;
        scalar_t force_ub = 200.0;
        scalar_t force_lb = 0.0;
        matrix_t A_friction = matrix_t::Zero(5, 3);
        A_friction << 1.0, 0.0, -mu,
                        0.0, 1.0, -mu,
                        -1.0, 0.0, -mu,
                        0.0,-1.0, -mu,
                        0.0, 0.0, 1.0;

        for (int leg = 0; leg < contact_flag_.size(); ++leg) {
            Root_Force_Transition.block(0, 3 * leg, 3, 3).setIdentity();
            vector3_t foot_pos_base = base_So3_world.transpose() * (foot_pos_world[leg] - base_pos_world);
            Root_Force_Transition.block(3, 3 * leg, 3, 3) = skew(foot_pos_base) * (base_So3_world.transpose());
            // friction
            A_vmc.block(5 * leg, 3 * leg, 5, 3) = A_friction;
            ub_vmc(4 + 5 * leg) = static_cast<scalar_t>(contact_flag_[leg]) * force_ub;
            lb_vmc.segment(5 * leg, 5) = -1.0e5 * vector_t::Ones(5);
            lb_vmc(4 + 5 * leg) = static_cast<scalar_t>(contact_flag_[leg]) * force_lb;
        }

    
        matrix_t Q_vmc = ( weight_root_vmc.asDiagonal() * Root_Force_Transition ).transpose() * ( weight_root_vmc.asDiagonal() * Root_Force_Transition );
        Q_vmc.diagonal().array() += weight_force_vmc * weight_force_vmc;
        vector_t F_vmc = ( weight_root_vmc.asDiagonal() * Root_Force_Transition ).transpose() * ( weight_root_vmc.asDiagonal() * -base_acc );

        qpoases_interface_->Solve(Q_vmc, F_vmc, A_vmc, lb_vmc, ub_vmc);
        vector_t torque_vmc = -j_.block(0, 6,  3 * contact_flag_.size(), info_.generalizedCoordinatesNum - 6).transpose() * qpoases_interface_->GetSolved().segment(0, 3 * contact_flag_.size());

        for (int leg = 0; leg < contact_flag_.size(); ++leg) {
            if (!contact_flag_[leg]) {
                torque_vmc.segment(3*leg, 3) = torque_swing.segment(3*leg, 3);
            }
        }
        //TODO: waist pd control
        return torque_vmc;
    }

    void VMControl::updateMeasured(const vector_t &rbdStateMeasured) {
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

        auto frame_id = model.getFrameId("trunk");
        
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
