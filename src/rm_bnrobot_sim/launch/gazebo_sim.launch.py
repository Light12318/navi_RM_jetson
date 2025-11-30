import launch
import launch_ros
from launch_ros.actions import Node 
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    robot_name="bngu_sentinel"
    #默认路径（功能包，urdf,world,gazebo）
    package_path=get_package_share_directory('rm_bnrobot_sim')
    default_urdf_path=os.path.join(package_path,'urdf/bngu_sentinel/bnbot.urdf.xacro')
    default_world_path=os.path.join(package_path,'world/custom_room.world')
    gazebo_path=os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')
    config_path=os.path.join(package_path,'config','bnbot_ros2_controller.yaml')
    #2个启动参数，model和world
    model_path_arg=launch.actions.DeclareLaunchArgument(
        name='model',default_value=default_urdf_path,
        description='模型默认加载路径'
    )
    world_path_arg=launch.actions.DeclareLaunchArgument(
        name='world',default_value=default_world_path,
        description='默认世界加载路径'
    )
    #加载xacro,加载模型文件
    robot_model=launch_ros.parameter_descriptions.ParameterValue(
        launch.substitutions.Command(['xacro ',launch.substitutions.LaunchConfiguration("model")])
    )
    robot_state_node=Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description':robot_model}]
    )
    gazebo_node=launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_path),
        launch_arguments=[('world',launch.substitutions.LaunchConfiguration('world')),('verbose','true')]
    )
    spawn_node = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name,
                    '-x',"0",#7.537520
                    '-y',"0",#8.201410
                    '-z',"0" ,#0.60
                    '-Y',"0"])
    
    #bnbot_joint_state_broadcaster 控制器
    load_joint_state_controller = launch.actions.ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
            'bnbot_joint_state_broadcaster'],
        output='screen'
    )

    # # 加载激活 bnbot_effort_controller 控制器
    # load_bnbot_effort_controller = launch.actions.ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active','bnbot_effort_controller'], 
    #     output='screen')
    return launch.LaunchDescription([
        model_path_arg,
        world_path_arg,
        robot_state_node,
        gazebo_node,

        spawn_node,  
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=spawn_node,
                on_exit=[load_joint_state_controller],)
            ),
        # launch.actions.RegisterEventHandler(
        #     event_handler=launch.event_handlers.OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_bnbot_effort_controller]
        #     ))
        


    ])
