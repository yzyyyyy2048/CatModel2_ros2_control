#ifndef MY_PUBLISHER_HPP  
#define MY_PUBLISHER_HPP  
  
#include <rclcpp/rclcpp.hpp>  
#include <std_msgs/msg/float32_multi_array.hpp>  
  
class MyPublisher  
{  
public:  
    MyPublisher(rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("float32_multi_array_publisher"));  
    void setArrayLength(int length);  
    void inputArray(float* array);  
    void publishArray();  
    rclcpp::Node::SharedPtr getNode();
private:  
    rclcpp::Node::SharedPtr node_;  
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;  
    std_msgs::msg::Float32MultiArray array_msg_; 
};  
  
#endif // MY_PUBLISHER_HPP