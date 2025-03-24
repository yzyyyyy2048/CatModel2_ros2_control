// #include "my_publisher.hpp"  
#include <CatModel2_v2_controller/plotdata/my_publisher.hpp>

MyPublisher::MyPublisher(rclcpp::Node::SharedPtr node)  
: node_(node), publisher_(node_->create_publisher<std_msgs::msg::Float32MultiArray>("my_topic", 10))  
{  
}  
  
void MyPublisher::setArrayLength(int length)  
{  
    array_msg_.data.resize(length);  
}  
  
void MyPublisher::inputArray(float* array)  
{  
    for (int i = 0; i < array_msg_.data.size(); i++) {  
        array_msg_.data[i] = array[i];  
    }   
}  
  
void MyPublisher::publishArray()  
{  
    publisher_->publish(array_msg_);  
}

rclcpp::Node::SharedPtr MyPublisher::getNode()
{
    return node_;
}