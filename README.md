# CatModel2 ROS2 Control
与ros1版本的程序类似，在ros2下仍采用多工作空间的方式管理各个package

## 1.OCS2
创建OCS2的工作空间并克隆ros2版本的OCS2
```
https://github.com/legubiao/ocs2_ros2.git
```
按照仓库中的readme编译ocs2_legged_robot_ros与ocs2_anymal_mpc
## 2.mujoco
这里需要使用源码的方式安装mujoco，并且需要安装unitree_sdk2作为依赖项
```
https://github.com/yzyyyyy2048/mujoco.git
```
## 3.quadruped_ros2_control
ros2版本的legged_control，并且集中了其他形式的控制器，配置的过程中同时需要安装unitree_ros2
另，编译成功后运行ocs2控制器出现卡顿需要手动调整controller_manager的频率
```
https://github.com/yzyyyyy2048/quadruped_ros2_control
```

## 4.CatModel2 ROS2 Control
coming soon