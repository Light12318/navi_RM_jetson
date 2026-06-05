import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # ================= 1. 路径定义 =================
    robot_name = "bngu_sentinel"
    sim_package_path = get_package_share_directory('rm_bnrobot_sim')
    nav_package_path = get_package_share_directory('rm_bnrobot_nav')
    bringup_path = get_package_share_directory('nav2_bringup')
    # 注意：这里假设你的配置文件都在对应的包下
    launch_path = get_package_share_directory('sim_bringup') 
    
    # Gazebo 相关路径
    gazebo_launch_path = os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    default_world_path = os.path.join(sim_package_path, 'world', 'custom_room.world')
    
    # 导航与算法配置路径
    default_map_path = os.path.join(nav_package_path, 'maps', 'room.yaml')
    nav2_params_path = os.path.join(launch_path, 'config', 'sim_nav_params.yaml')
    rviz2_path = os.path.join(nav_package_path, 'rviz', 'rviz2.rviz')
    fast_lio_config_path = os.path.join(launch_path, 'config', 'mid360_sim.yaml')
    localization_config_path = os.path.join(launch_path, 'config', 'localization_sim.yaml')
    pcd_map_path = os.path.join(nav_package_path, 'maps', 'pcd', 'simmap_normal.pcd')
    
    # URDF 模型路径 (使用第一个脚本中的路径)
    model_path = os.path.join(sim_package_path, 'urdf', 'bngu_sentinel', 'bnbot.urdf.xacro')

    # ================= 2. 声明 Launch 参数 =================
    declare_use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='是否使用仿真时间'
    )
    declare_world = launch.actions.DeclareLaunchArgument(
        'world', default_value=default_world_path, description='Gazebo 世界文件路径'
    )
    declare_model = launch.actions.DeclareLaunchArgument(
        'model', default_value=model_path, description='URDF 模型路径'
    )
    declare_map = launch.actions.DeclareLaunchArgument(
        'map', default_value=default_map_path, description='2D栅格地图路径'
    )

    use_sim_arg = LaunchConfiguration('use_sim_time')
    world_arg = LaunchConfiguration('world')
    model_arg = LaunchConfiguration('model')

    # ================= 3. 基础环境 (Gazebo & Robot State) =================
    
    # 解析 xacro
    robot_description_content = Command(['xacro ', model_arg])
    robot_description = {'robot_description': launch_ros.parameter_descriptions.ParameterValue(robot_description_content, value_type=str)}

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_arg}]
    )

    # Gazebo 物理仿真服务器
    gazebo_node = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments=[('world', world_arg), ('verbose', 'true')]
    )

    # 在 Gazebo 中生成实体
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', robot_name, '-z', '0.1'],
        output='screen'
    )

    # ================= 4. 控制器 =================
    
    # 加载 Joint State Broadcaster
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'bnbot_joint_state_broadcaster'],
        output='screen'
    )


    # ================= 5. 算法与导航组件 =================

    # FAST_LIO2
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fast_lio',
        output='screen',
        parameters=[fast_lio_config_path,
                    {
                        'use_sim_time': use_sim_arg,
                        'use_system_time': False,
                        

                        'publish_frame_id': 'mid360_laser',        # 原来是 'map'，必须改成 'odom'
                        'child_frame_id': 'base_link',       
                        'publish_tf': True,                
                        

                        'publish_odom': True,               
                        'odom_topic': '/odom',               
                        'odom_frame_id': 'odom',           # 原来是 'map'，必须改成 'odom'
                        'base_link_frame_id': 'base_link',  
                        
                        'publish_rate': 50.0,                
                    }],
        remappings=[('/Odometry_loc', '/odom'), ('/cloud_registered_1', '/cloud_registered')]
    )


    # 点云转 LaserScan (用于 2D 避障)
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        parameters=[{
            'target_frame': 'odom',
            'transform_tolerance': 0.01,
            'min_height': 0.5,
            'max_height': 2.0,
            'use_sim_time': use_sim_arg,
        }],
        remappings=[('/cloud_in', '/cloud_registered'), ('/scan', '/scan')]
    )

    # Nav2 导航组
    navi_group = launch.actions.GroupAction([
        SetRemap(src='/map', dst='/map_2d'),
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_path, 'launch', 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_arg,
                'params_file': nav2_params_path,
                'autostart': 'True',
            }.items(),
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': LaunchConfiguration('map'), 'use_sim_time': use_sim_arg}]
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            parameters=[{'use_sim_time': use_sim_arg, 'autostart': True, 'node_names': ['map_server']}]
        )
    ])
    localization_node = launch_ros.actions.Node(
        package='open3d_loc',     
        executable='global_localization_node',       
        name='global_localization_node',
        output='screen',
        parameters=[
            localization_config_path,  
            {
                'path_map': '/home/mage/navigation/nav_RM_4/test991.pcd',
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.2,
                'voxelsize_fine': 0.1,   
                'threshold_fitness': 0.35,      # 调严一点，防止位姿乱飞
                'threshold_fitness_init': 0.4,
                'loc_frequence': 0.1,           # 建议 0.1s (10Hz)
                'save_scan': False,
                'hidden_removal': False,
                'maxpoints_source': 10000,
                'maxpoints_target': 50000,
                'filter_odom2map': True,        # 开启滤波，让定位更平滑
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.7,
                'dis_updatemap': 0.5,         # 适当加大更新间距，减少计算压力
                'use_sim_time': True            # 核心：必须为 True 
            }
        ],
        remappings=[
            ('/Odometry_loc','/odom'),
            ('/cloud_registered_1','/cloud_registered')
        ]
    )


    # RViz2
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz2_path], parameters=[{'use_sim_time': use_sim_arg}]
    )

    # ================= 6. 静态 TF =================
    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_arg}]
    )
    static_tf_motion = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'motion_link']
    )
    
    static_tf_camera = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0', '0', '0.0', '0', '0', '0', '1', 'odom', 'camera_init']
    )

    # ================= 7. 启动顺序控制 (Event Handlers) =================

    return launch.LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_model,
        declare_map,

        # 第一阶段：基础环境
        robot_state_publisher_node,
        gazebo_node,
        spawn_entity_node,
        static_tf_motion,
        static_tf_camera,

        # 第二阶段：生成成功后加载控制器
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[load_joint_state_controller],
            )
        ),

        # 第三阶段：算法延迟启动 (保证 Gazebo 和 TF 树已就绪)
        launch.actions.TimerAction(period=3.0, actions=[fast_lio_node]),
        launch.actions.TimerAction(period=4.0, actions=[static_tf_map_to_odom]),
        # launch.actions.TimerAction(period=3.0, actions=[localization_node]),
        launch.actions.TimerAction(period=5.0, actions=[pointcloud_to_laserscan_node]),
        launch.actions.TimerAction(period=8.0, actions=[navi_group, rviz_node]),
    ])