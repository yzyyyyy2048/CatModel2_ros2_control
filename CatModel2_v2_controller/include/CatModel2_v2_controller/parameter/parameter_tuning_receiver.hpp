#ifndef PARAMETER_TUNING_RECEIVER_HPP  
#define PARAMETER_TUNING_RECEIVER_HPP 

#include "control_parameter.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ParameterTuningReceiver {  
public:
  ParameterTuningReceiver(const std::shared_ptr<ControlParameter>& control_parameter_ptr);  
  void run();
private:


  // ros2
  rclcpp::Node::SharedPtr node_;  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  std::shared_ptr<ControlParameter> control_parameter_ptr_;

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;  
};  

#endif // PARAMETER_TUNING_RECEIVER_HPP