import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # ================= 1. 路径配置 =================
    # 请根据你的实际包名确认这些路径
    pkg_sim = get_package_share_directory('rm_bnrobot_sim')
    pkg_fastlio = get_package_share_directory('fast_lio')
    pkg_bringup = get_package_share_directory('sim_bringup') # 假设配置在此包
    
    # 文件路径
    gazebo_launch_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    world_path = os.path.join(pkg_sim, 'world', 'custom_room.world')
    model_path = os.path.join(pkg_sim, 'urdf', 'bngu_sentinel', 'bnbot.urdf.xacro')
    fast_lio_config = os.path.join(pkg_bringup, 'config', 'mid360_sim.yaml')
    rviz_config = os.path.join(pkg_bringup, 'rviz', 'rviz2_test.rviz')

    # ================= 2. 参数定义 =================
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # ================= 3. 基础仿真组件 =================
    
    # A. 解析 URDF (这里包含了你的倒置安装 TF)
    robot_description = Command(['xacro ', model_path])

    # B. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # C. 启动 Gazebo
    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'world': world_path, 'verbose': 'true'}.items()
    )

    # D. 在 Gazebo 中生成机器人
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bnbot', '-z', '0.2'],
        output='screen'
    )

    # ================= 4. 算法组件 =================

    # E. FAST_LIO (延迟启动以等待 Gazebo 就绪)
    node_fast_lio = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        parameters=[fast_lio_config, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # F. 静态 TF: 桥接 odom 和 FAST-LIO 的起始原点
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'odom', 'camera_init'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # G. RViz2
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ================= 5. 启动描述 =================
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # 立即启动环境
        node_robot_state_publisher,
        launch_gazebo,
        node_spawn_entity,
        node_static_tf,

        # 延迟启动算法和可视化，确保仿真时钟已同步
        TimerAction(period=5.0, actions=[node_fast_lio]),
        TimerAction(period=6.0, actions=[node_rviz]),
    ])