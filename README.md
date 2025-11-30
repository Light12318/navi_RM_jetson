## author 牢🐴

# question
- 部分依赖仍然存在问题
- 雷达消息接口有问题,没有正常调入fast_lio的数据
- 模拟数据训练还没整明白

# 基于ro2 和 nav2 自动巡检
## 1.项目介绍
该项目基于肉丝2和nav2来实现多点自动巡检,
目前处于大残状态，只能初步完成几个点之间的巡检，以及当到达对应点后传回相应信息，目前增加了到达目的地获取图像的功能（额因为视觉部分忘的有点多，所以暂时挂个在这，后面找视觉的人问问）
## 2.功能包介绍
- rm_bnrobot_sim 机器人仿真
- rm_bnrobot_nav 机器人导航
- rm_bnrobot_function 导航相关功能包
- rm_interfaces  目标相关接口
- livox_ros_driver2 雷达驱动
- can_twist_node 指令发送节点
- rm_bringup 一键启动功能包（暂不可用）
## 3 安装依赖
- 系统版本： Ubunt22.04     
- ROS 版本：ROS 2 Humble
本项目建图使用 slam-toolbox，fast_lio(暂不可用)，导航使用 Navigation 2 ,仿真采用 Gazebo，运动控制采用 ros2-control ，
依赖：（没写到的只有去问豆包了）

1. 安装基本依赖（slam_toolbox,nav2,ros2_control）

```
sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox
sudo apt install -y ros-humble-gazebo-ros2-control
```

2. 安装仿真相关功能包

```
sudo apt install ros-$ROS_DISTRO-robot-state-publisher  ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-xacro
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
sudo apt install ros-humble-tf2-ros ros-humble-tf2-geometry-msgs
sudo apt install ros-humble-robot-localization
sudo apt install ros-humble-pointcloud-to-laserscan
```

## 2.2 接口
| Topic name | Type | Note |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | Mid360 自定义消息类型 |
| `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` | ROS2 点云消息类型 |
| `/livox/imu` | `sensor_msgs/msg/Imu` | Gazebo 插件仿真 IMU |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 麦克纳姆轮小车运动控制接口 |

## 2.3 运行
1. rviz里显示机器人(非必要步骤，仅调试urdf时可方便查看)
- 参数：model
```
ros2 launch rm_bnrobot_sim robot_sim.launch.py
```
2. 在gazebo中启动仿真 
- 参数：model:模型路径 world：地图路径
```
ros2 launch rm_bnrobot_sim gazebo_sim.launch.py
```
3. 启动nav导航  
- 参数：map, use_sim_time , param_file
```
ros2 launch rm_bnrobot_nav nav_load.launch.py
```
4. 初始化坐标
```
ros2 run rm_bnrobot_function init_pose.py
```
5. 获取坐标 
```
ros2 run rm_bnrobot_function get_pose.py 
```
6. 导航到指定点 
- 参数：goal.x goal.y goal.z:目标点的x,y,z坐标
```
ros2 run rm_bnrobot_function nav_pose.py 
```

