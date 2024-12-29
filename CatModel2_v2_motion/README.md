# CatModel2_Motion

## Run
run with dummy controller
```
ros2 launch CatModel2_v2_motion CatModel2_motion_dummy.launch.py
```

## Issues - TODO
motion模块将轨迹发布到 legged_robot_mpc_target节点
原 ROS1 下对于目标轨迹的跟踪管理：
在CatModel2Controller中，setupMpc函数的rosRefernceManagePtr订阅了legged_robot_mpc_target节点
```cpp
auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName, leggedInterface_->getReferenceManagerPtr());
rosReferenceManagerPtr->subscribe(nh);
```
（rosReferenceManager是ocs2的实现）
但是在 ROS2 的 legged_control 中，setupMpc函数使用的是自己实现的 TargetManager类管理目标轨迹。
```cpp
ctrl_comp_.target_manager_ = std::make_shared<TargetManager>(ctrl_comp_,
                                                             legged_interface_->getReferenceManagerPtr(),
                                                             task_file_, reference_file_);
```