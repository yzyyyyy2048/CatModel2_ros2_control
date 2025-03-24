#ifndef CONTROL_PARAMETER_HPP
#define CONTROL_PARAMETER_HPP

#include <map>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include <mutex>

#define PARAMETER_GET_FLOAT_VECTOR(name,parameter) \
  std::vector<float> name ## _std_vector_float = parameter->get<std::vector<float>>(#name);\
  vector_t name(name ## _std_vector_float.size());\
  for (int k = 0; k < name ## _std_vector_float.size(); ++k) { \
    name[k] = static_cast<scalar_t>(name ## _std_vector_float[k]); \
  }

#define PARAMETER_GET_FLOAT(name,parameter)  \
scalar_t name = static_cast<scalar_t>(parameter->get<float>(#name));

#define PARAMETER_GET_INT(name,parameter)  \
int name = static_cast<int>(parameter->get<int>(#name));

#define PARAMETER_GET_BOOL(name,parameter)  \
bool name = static_cast<bool>(parameter->get<bool>(#name));

class ControlParameter {
public:
  ControlParameter(const std::string& yaml_file) {
    // Load the YAML file.
    YAML::Node config = YAML::LoadFile(yaml_file);

    // Iterate over the YAML nodes and store the parameters.
    for (const auto& node : config) {
      const std::string& parameter_name = node.first.as<std::string>();
      if (node.second.IsScalar()) {
        // Scalar parameter (float, int, bool).
        if (node.second.IsScalar() && node.second.Type() == YAML::NodeType::Scalar) {
          if (node.second.Tag() == "!float") {
            float_parameters_[parameter_name] = node.second.as<float>();
          } else if (node.second.Tag() == "!int") {
            int_parameters_[parameter_name] = node.second.as<int>();
          } else if (node.second.Tag() == "!bool") {
            bool_parameters_[parameter_name] = node.second.as<bool>();
          }
        }
      } else if (node.second.IsSequence()) {
        // Array parameter.
        if (node.second.Type() == YAML::NodeType::Sequence) {

          array_parameters_[parameter_name] = node.second.as<std::vector<float>>();
          
        }
      }
    }
  }

  template<typename T>
  T get(const std::string& parameter_name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if constexpr (std::is_same_v<T, float>) {
      auto it = float_parameters_.find(parameter_name);
      if (it != float_parameters_.end()) {
        return it->second;
      }
    } else if constexpr (std::is_same_v<T, int>) {
      auto it = int_parameters_.find(parameter_name);
      if (it != int_parameters_.end()) {
        return it->second;
      }
    } else if constexpr (std::is_same_v<T, bool>) {
      auto it = bool_parameters_.find(parameter_name);
      if (it != bool_parameters_.end()) {
        return it->second;
      }
    } else if constexpr (std::is_same_v<T, std::vector<float>>) {
      auto it = array_parameters_.find(parameter_name);
      if (it != array_parameters_.end()) {
        return it->second;
      }
    }
    throw std::runtime_error("Invalid parameter name or type");
  }

  template<typename T>
  void set(const std::string& parameter_name, const T& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    if constexpr (std::is_same_v<T, float>) {
      float_parameters_[parameter_name] = value;
    } else if constexpr (std::is_same_v<T, int>) {
      int_parameters_[parameter_name] = value;
    } else if constexpr (std::is_same_v<T, bool>) {
      bool_parameters_[parameter_name] = value;
    } else if constexpr (std::is_same_v<T, std::vector<float>>) {
      array_parameters_[parameter_name] = value;
    } else {
      throw std::runtime_error("Invalid parameter type");
    }
  }

 private:
  std::map<std::string, float> float_parameters_;
  std::map<std::string, int> int_parameters_;
  std::map<std::string, bool> bool_parameters_;
  std::map<std::string, std::vector<float>> array_parameters_;
  mutable std::mutex mutex_;
};


#endif // CONTROL_PARAMETER_HPP