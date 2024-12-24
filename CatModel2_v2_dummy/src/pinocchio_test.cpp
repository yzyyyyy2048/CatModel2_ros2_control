#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include <iostream>


int main()
{
    std::string path = "/home/yzy/CatModel2_ros2_control_ws/src/CatModel2_v2_description/urdf/CatModel2_v2.urdf";
    std::cout << path << std::endl;

    pinocchio::Model model_;
    pinocchio::urdf::buildModel(path, model_);
    pinocchio::Data data_(model_);
    std::cout << model_.name << std::endl;

    Eigen::VectorXd q = pinocchio::neutral(model_);
    std::cout << "q: " << q.transpose() << std::endl;
    pinocchio::forwardKinematics(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);
    for(const auto& frame : model_.frames) {
    std::cout << "Frame name: " << frame.name << std::endl;
}

    pinocchio::FrameIndex LF_index = model_.getFrameId("LF_FOOT");
    std::cout << "LF_FOOT: " << data_.oMf[LF_index].translation().transpose() << std::endl;

    
    // print all the joint names of model_
    for (pinocchio::JointIndex joint_id = 0;
         joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
        std::cout << std::setw(24) << std::left << model_.names[joint_id]
                  << ": " << std::fixed << std::setprecision(4)
                  << data_.oMi[joint_id].translation().transpose() << std::endl;
    return 0;
}