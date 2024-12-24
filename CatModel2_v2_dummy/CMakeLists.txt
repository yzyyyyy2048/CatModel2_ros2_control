cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
project(CatModel2_v2_dummy)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(dependencies
        tf2_ros
        kdl_parser
        robot_state_publisher
        CatModel2_v2_interface
        ocs2_ros_interfaces
)

find_package(ament_cmake REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(CatModel2_v2_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(ocs2_ros_interfaces REQUIRED)

###########
## Build ##
###########

# Main library
add_library(${PROJECT_NAME}
        src/gait/GaitKeyboardPublisher.cpp
        src/gait/GaitReceiver.cpp
        src/visualization/CatModel2Visualizer.cpp
)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

ament_target_dependencies(${PROJECT_NAME} ${dependencies})
target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})



## SQP-MPC node for legged robot
add_executable(CatModel2_sqp_mpc src/CatModel2SqpMpcNode.cpp)
ament_target_dependencies(CatModel2_sqp_mpc ${dependencies})
target_link_libraries(CatModel2_sqp_mpc ${PROJECT_NAME})
target_compile_options(CatModel2_sqp_mpc PRIVATE ${OCS2_CXX_FLAGS})


# Dummy node
add_executable(CatModel2_dummy src/CatModel2DummyNode.cpp)
ament_target_dependencies(CatModel2_dummy ${dependencies})
target_link_libraries(CatModel2_dummy ${PROJECT_NAME})
target_compile_options(CatModel2_dummy PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for target trajectories
add_executable(CatModel2_target src/CatModel2PoseCommandNode.cpp)
ament_target_dependencies(CatModel2_target ${dependencies})
target_link_libraries(CatModel2_target ${PROJECT_NAME})
target_compile_options(CatModel2_target PRIVATE ${OCS2_CXX_FLAGS})

# Keyboard command node for gait
add_executable(CatModel2_gait_command src/CatModel2GaitCommandNode.cpp)
ament_target_dependencies(CatModel2_gait_command ${dependencies})
target_link_libraries(CatModel2_gait_command ${PROJECT_NAME})
target_compile_options(CatModel2_gait_command PRIVATE ${OCS2_CXX_FLAGS})


# pinocchio_test
add_executable(pino_test src/pinocchio_test.cpp)
ament_target_dependencies(pino_test ${dependencies})
target_link_libraries(pino_test ${PROJECT_NAME})
target_compile_options(pino_test PRIVATE ${OCS2_CXX_FLAGS})

find_package(ament_lint_auto REQUIRED)
ament_lint_auto_find_test_dependencies()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target CatModel2_v2_dummy")
    add_clang_tooling(
            TARGETS
            ${PROJECT_NAME}    
            CatModel2_sqp_mpc
            CatModel2_dummy
            CatModel2_target
            CatModel2_gait_command
            SOURCE_DIRS src include
            CT_HEADER_DIRS include
            CF_WERROR
    )
endif(cmake_clang_tools_FOUND)


#############
## Install ##
#############

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(
        TARGETS
        CatModel2_sqp_mpc
        CatModel2_dummy
        CatModel2_target
        CatModel2_gait_command
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${dependencies})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()