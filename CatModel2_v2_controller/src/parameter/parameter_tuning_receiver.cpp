#include <CatModel2_v2_controller/parameter/parameter_tuning_receiver.hpp>


ParameterTuningReceiver::ParameterTuningReceiver(const std::shared_ptr<ControlParameter>& control_parameter_ptr) {
  node_ = std::make_shared<rclcpp::Node>("parameter_tuning_receiver");
  subscription_ = node_->create_subscription<std_msgs::msg::String>(  
            "parameter_tuning", 10, std::bind(&ParameterTuningReceiver::topic_callback, this, std::placeholders::_1));
  control_parameter_ptr_ = control_parameter_ptr;
}

void ParameterTuningReceiver::run() {
  rclcpp::spin(node_);
  if (!rclcpp::ok()) {
    // Shutdown the ROS2 node
    rclcpp::shutdown();
  }
}

void ParameterTuningReceiver::topic_callback(const std_msgs::msg::String::SharedPtr msg) const {  
    std::istringstream iss(msg->data);  
    std::string token;  
    std::string name, type;  
      
    // Get the name  
    if (std::getline(iss, name, '@')) {  
        // Get the type  
        if (std::getline(iss, type, '@')) {  
            // Process the data based on the type  
            if (type == "float") {  
                float data;  
                iss >> data;  
                // Use the float data  
                // std::cout << name << "^" << type << "^" << data << std::endl;
                control_parameter_ptr_->set<float>(name, data);
            } else if (type == "int") {  
                int data;  
                iss >> data;  
                // Use the int data 
                // std::cout << name << "^" << type << "^" << data << std::endl; 
                control_parameter_ptr_->set<int>(name, data);
            } else if (type == "bool") {  
                std::string bool_str;  
                iss >> std::boolalpha >> bool_str;  // Read bool as string then convert  
                bool data = (bool_str == "true") || (bool_str == "True");  
                // Use the bool data  
                // std::cout << name << "^" << type << "^" << data << std::endl;
                control_parameter_ptr_->set<bool>(name, data);
            } else if (type == "list") {  
                // Assuming the list is in the format [f1, f2, f3, ...]  
                std::vector<float> data;  
                if (iss.get() == '[') {  // Check for opening bracket  
                    while (iss) {  
                        float value;  
                        if (iss >> value) {  
                            data.push_back(value);  
                            // Check if the next character is a comma or closing bracket  
                            char next = iss.peek();  
                            if (next == ',' || next == ']') {  
                                iss.ignore();  // Ignore the comma or break if it's the closing bracket  
                                if (next == ']') break;  // Stop if it's the closing bracket  
                            } else {  
                                // Error handling for malformed list  
                            }  
                        } else {  
                            // Error handling for malformed list or non-float value in the list  
                        }  
                    }  
                } else {  
                    // Error handling for missing opening bracket  
                }  
                // Use the vector<float> data  
                // for (int k = 0; k < data.size(); ++k) {
                //   if (k) {
                //     std::cout << " , ";
                //   }
                //   std::cout << data[k];
                // }
                // std::cout << std::endl;
                control_parameter_ptr_->set<std::vector<float>>(name, data);
            } else {  
                // Handle unknown type or error cases  
            }  

        } else {  
            // Handle error if type delimiter not found or other issues with the message format  
        }  
    } else {  
        // Handle error if name delimiter not found or other issues with the message format  
    }  
} 