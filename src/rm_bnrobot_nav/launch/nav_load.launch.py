import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    nav_package_path=get_package_share_directory('rm_bnrobot_nav')
    bringup_path=get_package_share_directory('nav2_bringup')
    default_map_path=os.path.join(nav_package_path,'maps','room.yaml')
    nav2_params_path=os.path.join(nav_package_path,'config','nav2_params.yaml')
    rviz2_path=os.path.join(nav_package_path,'rviz','rviz2.rviz')

    declare_use_sim_time = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
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
    use_sim_arg=launch.substitutions.LaunchConfiguration('use_sim_time')
    map_arg=launch.substitutions.LaunchConfiguration('map')
    param_arg=launch.substitutions.LaunchConfiguration('param_file')



    navi_load=launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_path,'launch','bringup_launch.py')),
            launch_arguments={
                'map': map_arg,
                'use_sim_time': use_sim_arg,
                'params_file': param_arg}.items(),
    )
    rviz_node=launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_path],
            parameters=[{'use_sim_time': use_sim_arg}],
            output='screen')
    

    
    return launch.LaunchDescription([
        declare_map_path,
        declare_use_sim_time,
        declare_params_path,
        navi_load,
        rviz_node,
        # pointcloud_to_laserscan
    ])
