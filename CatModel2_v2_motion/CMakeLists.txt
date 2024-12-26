cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(CatModel2_v2_motion)

set(dependencies
  rclcpp
  ocs2_ros_interfaces
  ocs2_robotic_tools
  ocs2_switched_model_interface
  grid_map_filters_rsl
  Boost
  CatModel2_v2_interface
  CatModel2_v2_dummy
)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)
find_package(ocs2_robotic_tools REQUIRED)
find_package(ocs2_switched_model_interface REQUIRED)
find_package(grid_map_filters_rsl REQUIRED)
find_package(CatModel2_v2_interface REQUIRED)
find_package(CatModel2_v2_dummy REQUIRED)

## Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
  log_setup
  log
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Declare a C++ library
add_library(${PROJECT_NAME}
  src/LoadMotions.cpp
  src/CatModel2MotionDummy.cpp
  src/CatModel2MotionInterface.cpp
  )
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
  )
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})

# Declare a C++ Executable
add_executable(CatModel2_motion_command
  src/CatModel2MotionNode.cpp
  )
ament_target_dependencies(CatModel2_motion_command
  ${dependencies}
  )
target_link_libraries(CatModel2_motion_command   
  ${PROJECT_NAME}
)
target_compile_options(CatModel2_motion_command PRIVATE ${OCS2_CXX_FLAGS})

#############
## Install ##
#############

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(TARGETS 
CatModel2_motion_command
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()
find_package(ament_cmake_gtest)

ament_add_gtest(test_${PROJECT_NAME}
  test/testLoadMotions.cpp
)
ament_target_dependencies(test_${PROJECT_NAME}
  ${dependencies}
)
target_link_libraries(test_${PROJECT_NAME}
  ${PROJECT_NAME}
  -lstdc++fs
)

ament_export_dependencies(${dependencies})  
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()