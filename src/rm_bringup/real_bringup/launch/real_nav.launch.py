import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap

def generate_launch_description():
    nav_package_path = get_package_share_directory('rm_bnrobot_nav')
    real_nav_package_path = get_package_share_directory('real_bringup')
    bringup_path = get_package_share_directory('nav2_bringup')
    default_map_path = os.path.join(nav_package_path, 'maps', 'classroom.yaml')
    nav2_params_path = os.path.join(real_nav_package_path, 'config', 'real_nav2_params.yaml')
    rviz2_path = os.path.join(nav_package_path, 'rviz', 'rviz2.rviz')
    rviz2_path2 = os.path.join(real_nav_package_path, 'rviz', 'fastlio2.rviz')
    localization_config_path = os.path.join(real_nav_package_path, 'config', 'localization.yaml')
    
    
    # Fast_lio 相关路径
    fast_lio_path = get_package_share_directory('fast_lio')
    fast_lio_config_path = os.path.join(real_nav_package_path, 'config')

    urdf_path = get_package_share_directory('rm_bnrobot_sim')
    model_path = urdf_path + '/urdf/bngu_sentinel/bnbot_real.xacro'


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
        description='是否使用 Fast LIO 进行定位',

    )
    
    use_sim_arg = launch.substitutions.LaunchConfiguration('use_sim_time')
    map_arg = launch.substitutions.LaunchConfiguration('map')
    param_arg = launch.substitutions.LaunchConfiguration('param_file')
    use_fast_lio_arg = launch.substitutions.LaunchConfiguration('use_fast_lio')


    robot_description = launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(
            ['xacro ', launch.substitutions.LaunchConfiguration('model')]),
        value_type=str
    )
    
    # 状态发布节点
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description},
                    {'use_sim_time': use_sim_arg}]
    )
    


    # Fast_lio 节点
    fast_lio_node = None
    if fast_lio_path:
        fast_lio_node = launch_ros.actions.Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='fast_lio',  
            output='screen',
            parameters=[
                PathJoinSubstitution([fast_lio_config_path, 'mid360_real.yaml']), 
                    {
                        'use_sim_time': use_sim_arg,
                        'use_system_time': True,
                        

                        'publish_frame_id': 'odom',        # 原来是 'map'，必须改成 'odom'
                        'child_frame_id': 'mid360',        # 原来是 'base_link'，必须改成 ''，因为我们不想让 Fast LIO 发布 TF
                        'publish_tf': True,                
                        

                        'publish_odom': True,               
                        'odom_topic': '/odom',               
                        'odom_frame_id': 'odom',           # 原来是 'map'，必须改成 'odom'
                        'base_link_frame_id': 'mid360',  
                        
                        'publish_rate': 50.0,                
                    }
            ],
            remappings=[
            ('/Odometry_loc','/odom'),
            ('/cloud_registered_1','/cloud_registered')
            ],
            condition=launch.conditions.IfCondition(use_fast_lio_arg),

        )
    
    # 为了满足 open3d_loc 底层 C++ 代码的需求，强行发布的静态 TF
# 1. motion_link
    static_tf_base_center = launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_center_broadcaster',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                '--frame-id', 'mid360', '--child-frame-id', 'motion_link'
            ]
        )

    # 2. 规范化 imu_link 
    static_tf_imulink2baselink = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imulink2baselink',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0.0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'mid360', '--child-frame-id', 'imu_link'
        ]
    )
    
    static_tf_camera_init2odom = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_init2odom',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'odom', '--child-frame-id', 'camera_init'
        ]
    )

    

    localization_node = launch_ros.actions.Node(
        package='open3d_loc',     
        executable='global_localization_node',       
        name='global_localization_node',
        output='screen',
        parameters=[
            localization_config_path,  
            {
                'path_map': '/home/mage/navigation/nav_rm_5/classroomnormal_0.pcd',
                'pcd_queue_maxsize': 10,
                'voxelsize_coarse': 0.2,
                'voxelsize_fine': 0.1,   # 实时定位阶段的扫描分辨率，这个要与pcd分辨率匹配
                'threshold_fitness': 0.25,
                'threshold_fitness_init': 0.25,
                'loc_frequence': 0.5,
                'save_scan': False,
                'hidden_removal': False,
                'maxpoints_source': 10000,
                'maxpoints_target': 50000,
                'filter_odom2map': False,
                'kalman_processVar2': 0.001,
                'kalman_estimatedMeasVar2': 0.02,
                'confidence_loc_th': 0.8,
                'dis_updatemap': 0.5,
                'use_sim_time': use_sim_arg
            }
        ],
        remappings=[
            ('/Odometry_loc','/odom'),
            ('/cloud_registered_1','/cloud_registered')
        ]

    )

    pointcloud_to_laserscan_node = launch_ros.actions.Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'mid360',      
            'transform_tolerance': 0.5,            
            'min_height': -0.4,
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
            ('/cloud_in', '/cloud_registered'),
            ('/scan', '/scan')
        ],
        condition=launch.conditions.IfCondition(use_fast_lio_arg)
    )

    # 导航相关节点组合
    navi_group = launch.actions.GroupAction([

        SetRemap(src='/map', dst='/map_2d'),

        # 1. 导航控制模块
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_path, 'launch', 'navigation_launch.py')), 
            launch_arguments={
                'use_sim_time': use_sim_arg,
                'params_file': param_arg,
                'autostart': 'True',
                'use_composition': 'False',
            }.items(),
            # 注意：这里删掉了报错的 remappings 参数
        ),
        
        # 2.  2D 地图服务器 
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_arg, 'use_sim_time': use_sim_arg}]
        ),
        
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_arg},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
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
    static_tf_map_to_odom = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_publisher',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_arg}]
    )


    ld = launch.LaunchDescription([
        declare_map_path,
        declare_use_sim_time,
        declare_params_path,
        declare_use_fast_lio,
        mode_arg_path,
        static_tf_imulink2baselink,
        static_tf_base_center,
        static_tf_camera_init2odom,

    ])

    # 静态 TF
    ld.add_action(robot_state_publisher_node)  

    # Fast LIO
    if fast_lio_node:
        ld.add_action(fast_lio_node)
        ld.add_action(launch.actions.TimerAction(
            period=0.0,
            actions=[localization_node]#[static_tf_map_to_odom]#
        ))
        
    
    # 延迟启动激光转扫描
    if pointcloud_to_laserscan_node:
        ld.add_action(launch.actions.TimerAction(
            period=4.0,
            actions=[pointcloud_to_laserscan_node]
        ))

    # 延迟启动导航和 RViz
    ld.add_action(launch.actions.TimerAction(
        period=6.0,
        actions=[navi_group, rviz_node]
    ))

    return ld