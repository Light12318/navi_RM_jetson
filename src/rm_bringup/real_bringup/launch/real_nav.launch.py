import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    nav_package_path = get_package_share_directory('rm_bnrobot_nav')
    real_nav_package_path = get_package_share_directory('real_bringup')
    bringup_path = get_package_share_directory('nav2_bringup')
    default_map_path = os.path.join(nav_package_path, 'maps', 'room.yaml')
    nav2_params_path = os.path.join(real_nav_package_path, 'config', 'real_nav2_params.yaml')
    rviz2_path = os.path.join(nav_package_path, 'rviz', 'rviz2.rviz')
    
    # Fast LIO 相关路径
    fast_lio_path = get_package_share_directory('fast_lio')
    fast_lio_config_path = os.path.join(real_nav_package_path, 'config')

    urdf_path = get_package_share_directory('rm_bnrobot_sim')
    model_path = urdf_path + '/urdf/bngu_sentinel/bnbot_real.urdf'
    
    # 为 Launch 声明参数
    mode_arg_path = launch.actions.DeclareLaunchArgument(
        name='model', default_value=str(model_path),
        description='URDF 的绝对路径'
    )

    declare_use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    declare_map_path = launch.actions.DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='地图文件路径'
    )
    declare_params_path = launch.actions.DeclareLaunchArgument(
        'param_file',
        default_value=nav2_params_path,  
        description='导航参数文件路径'
    )
    
    declare_use_fast_lio = launch.actions.DeclareLaunchArgument(
        'use_fast_lio',
        default_value='True',
        description='是否使用 Fast LIO 进行定位'
    )
    
    use_sim_arg = launch.substitutions.LaunchConfiguration('use_sim_time')
    map_arg = launch.substitutions.LaunchConfiguration('map')
    param_arg = launch.substitutions.LaunchConfiguration('param_file')
    use_fast_lio_arg = launch.substitutions.LaunchConfiguration('use_fast_lio')

    # 获取文件内容生成新的参数
    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # 静态 TF 发布器
    odom_camera_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'camera_init', 'odom']
    )
    
    odom_footprint_node = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_footprint']
    )

    # Fast LIO 节点 - 现在直接输出，不使用 EKF
    fast_lio_node = None
    if fast_lio_path:
        fast_lio_node = launch_ros.actions.Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',  
            output='screen',
            parameters=[
                PathJoinSubstitution([fast_lio_config_path, 'mid360_real.yaml']),  # 配置文件优先（如激光参数、IMU参数）
                {
                    # 1. 时间配置（实车设 False，仿真设 True）
                    'use_sim_time': use_sim_arg,
                    'use_system_time': False,
                    
                    # 2. 核心映射：camera_init → map，body → base_link（关键参数）
                    'publish_frame_id': 'map',          # 父帧：原 camera_init 直接映射为 map
                    'child_frame_id': 'base_link',       # 子帧：原 body 直接映射为 base_link
                    'publish_tf': True,                  # 启用 TF 发布（发布 map → base_link）
                    
                    # 3. 里程计发布（适配 map 帧，可选但建议开启）
                    'publish_odom': True,                # 启用里程计发布
                    'odom_topic': '/odom',               # 里程计话题（Nav2 可订阅）
                    'odom_frame_id': 'map',              # 里程计父帧 = map（与 publish_frame_id 一致）
                    'base_link_frame_id': 'base_link',   # 里程计子帧 = base_link（与 child_frame_id 一致）
                    
                    # 4. 外参配置（修正格式，避免解析错误）
                    # 说明：extrinsic_T/R 是 LiDAR 相对于 IMU 的变换（若纯 LiDAR 模式，则是 LiDAR 相对于 base_link 的变换）
                    'extrinsic_T': [0.0, 0.0, 0.0],      # 平移：x,y,z（单位：米，无偏移则设为 0）
                    'extrinsic_R': [0.0, 0.0, 0.0, 1.0], # 旋转：四元数（x,y,z,w），单位矩阵对应此值（无旋转）
                    # （若 LiDAR 安装有偏移，比如高 0.5 米 → extrinsic_T: [0.0, 0.0, 0.5]）
                    
                    # 5. 其他辅助参数
                    'publish_rate': 50.0,                # TF/里程计发布频率
                }
            ],
            remappings=[
                # 无需额外重映射，直接发布目标帧和话题
            ],
            condition=launch.conditions.IfCondition(use_fast_lio_arg)
        )

    # pointcloud_to_laserscan 节点 - 使用 base_footprint 作为目标坐标系
    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_footprint',      # 使用 base_footprint
            'transform_tolerance': 0.5,            # 增加容忍时间
            'min_height': 0.0,
            'max_height': 2.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 50.0,
            'use_inf': False,
            'inf_epsilon': 1.0,
            'use_sim_time': use_sim_arg,
        }],
        remappings=[
            ('cloud_in', '/cloud_registered'),
            ('scan', '/scan')
        ],
        condition=launch.conditions.IfCondition(use_fast_lio_arg)
    )

    # 导航相关节点组合
    navi_group = launch.actions.GroupAction([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_path, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': map_arg,
                'use_sim_time': use_sim_arg,
                'params_file': param_arg,
                'autostart': 'True',
                'use_composition': 'True',
            }.items(),
        )
    ])

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz2_path],
        parameters=[{'use_sim_time': use_sim_arg}],
        output='screen'
    )

    # 定义启动描述
    ld = launch.LaunchDescription([
        declare_map_path,
        declare_use_sim_time,
        declare_params_path,
        declare_use_fast_lio,
        mode_arg_path
    ])

    # 第一步：启动所有静态 TF
    ld.add_action(robot_state_publisher_node)  
    # ld.add_action(odom_camera_node)  
    # ld.add_action(odom_footprint_node)  
    
    # 第二步：启动 Fast LIO（直接输出）
    if fast_lio_node:
        ld.add_action(fast_lio_node)
    
    # 第三步：延迟启动激光转扫描
    if pointcloud_to_laserscan_node:
        ld.add_action(launch.actions.TimerAction(
            period=2.0,
            actions=[pointcloud_to_laserscan_node]
        ))

    # 第四步：延迟启动导航和 RViz
    ld.add_action(launch.actions.TimerAction(
        period=5.0,
        actions=[navi_group, rviz_node]
    ))

    return ld