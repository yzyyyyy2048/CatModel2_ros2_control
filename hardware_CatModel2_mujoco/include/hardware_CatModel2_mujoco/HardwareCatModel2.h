

#ifndef HARDWARECATMODEL2_H
#define HARDWARECATMODEL2_H

#include "hardware_interface/system_interface.hpp"

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class HardwareCatModel2 final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    std::vector<double> imu_states_;
    std::vector<double> foot_force_;

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };


    void initLowCmd();

    void lowStateMessageHandle(const void *messages);

    unitree_go::msg::dds_::LowCmd_ low_cmd_{}; // default init
    unitree_go::msg::dds_::LowState_ low_state_{}; // default init

    /*publisher*/
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_publisher_;
    /*subscriber*/
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lows_tate_subscriber_;
};


#endif 
