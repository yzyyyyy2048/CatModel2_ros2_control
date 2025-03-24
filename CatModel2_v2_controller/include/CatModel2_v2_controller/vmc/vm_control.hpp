#pragma once


#include <ocs2_centroidal_model/PinocchioCentroidalDynamics.h>
#include <ocs2_legged_robot/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <CatModel2_v2_controller/parameter/parameter_tuning_receiver.hpp>
#include <CatModel2_v2_controller/unittest/robot_util.hpp>

#include <CatModel2_v2_controller/vmc/qpoases_interface.hpp>
namespace ocs2::legged_robot {
    template <typename Type>  
    class FiveOrderLine {
    public: 
    FiveOrderLine() {
        x = 0.0;
        dx = 0.0;
        ddx = 0.0;
    }
    void set(Type x0,Type dx0,Type ddx0,Type xT,Type dxT,Type ddxT,Type T,Type t);
    Type X() {return x;}
    Type dX() {return dx;}
    Type ddX() {return ddx;}
    private:
    Type x;
    Type dx;
    Type ddx; 
    };

    template <typename Type> 
    void FiveOrderLine<Type>::set(Type x0,Type dx0,Type ddx0,Type xT,Type dxT,Type ddxT,Type T,Type t) {
    t = std::min<Type>(std::max<Type>(t, 0.0), T);
    Type t2 = t* t;
    Type t3 = t2*t;
    Type t4 = t3*t;
    Type t5 = t4*t;


    Type T2 = T* T;
    Type T3 = T2*T;
    Type T4 = T3*T;
    Type T5 = T4*T;


    Eigen::Matrix<Type, 6, 1> b;
    b << x0,dx0,ddx0,xT,dxT,ddxT;

    Eigen::Matrix<Type, 6, 6> A;
    A << 1.0, 0.0,   0.0,    0.0,     0.0,     0.0,
        0.0, 1.0,   0.0,    0.0,     0.0,     0.0,
        0.0, 0.0,   2.0,    0.0,     0.0,     0.0,
        1.0,   T,    T2,     T3,      T4,      T5,
        0.0, 1.0, 2.0*T, 3.0*T2,  4.0*T3,  5.0*T4,
        0.0, 0.0,   2.0,  6.0*T, 12.0*T2, 20.0*T3;
    
    Eigen::Matrix<Type, 6, 6> A_T = A.transpose();

    Eigen::Matrix<Type, 6, 6> H = (A_T*A) + 1.0e-15*Eigen::Matrix<Type, 6, 6>::Identity();

    Eigen::Matrix<Type, 6, 1> r = H.llt().solve(A_T*b);

    Eigen::Matrix<Type, 1, 6> t_x;
    Eigen::Matrix<Type, 1, 6> t_dx;
    Eigen::Matrix<Type, 1, 6> t_ddx;

    t_x << 1.0,t,t2,t3,t4,t5;
    t_dx << 0.0,1.0,2.0*t,3.0*t2,4.0*t3,5.0*t4;
    t_ddx << 0.0,0.0,2.0,6.0*t,12.0*t2,20.0*t3;

    x = t_x*r;
    dx = t_dx*r;
    ddx = t_ddx*r;
    }

    class VMControl {
        using Vector6 = Eigen::Matrix<scalar_t, 6, 1>;
        using Matrix6 = Eigen::Matrix<scalar_t, 6, 6>;

    public:
        ~VMControl() = default;

        VMControl(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                const PinocchioEndEffectorKinematics &eeKinematics, const std::shared_ptr<ControlParameter>& control_parameter_ptr);

        vector_t update(const vector_t &rbdStateMeasured,
                                scalar_t currentTime, feet_array_t<scalar_t> swing_phase, feet_array_t<scalar_t> swing_time, vector_t cmd);

    protected:
        void updateMeasured(const vector_t &rbdStateMeasured);

        std::shared_ptr<ControlParameter> control_parameter_ptr_;
        PinocchioInterface pinocchio_interface_measured_;
        CentroidalModelInfo info_;

        std::unique_ptr<PinocchioEndEffectorKinematics> ee_kinematics_;
        CentroidalModelPinocchioMapping mapping_;
        std::shared_ptr<QPoasesInterface> qpoases_interface_;
        std::shared_ptr<FiveOrderLine<scalar_t>> five_order_line_;

        std::vector<vector3_t> foot_pos_start_hip_;
        std::vector<vector3_t> foot_pos_final_hip_;

        vector_t q_measured_, v_measured_, input_last_;
        matrix_t j_, dj_;
        contact_flag_t contact_flag_{};

        bool is_first_control_;

        // Task Parameters:
        vector_t torque_limits_;
    };
} // namespace legged