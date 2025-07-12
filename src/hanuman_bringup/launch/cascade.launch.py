from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

import os
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    description_package_name = "hanuman04"

    rviz_config = os.path.join(get_package_share_directory(
        description_package_name), "config", "humanoid" + ".rviz")
    
    robot_description = os.path.join(get_package_share_directory(
        description_package_name), "urdf", "hanuman" + ".urdf.xacro")
    
    robot_description_config = xacro.process_file(
        robot_description,
        mappings={'use_sim_time': 'true'}
    )
   
    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package_name),
            'config',
            'controllers.yaml',
        ]
    )
   
    default_world = os.path.join(
        get_package_share_directory(description_package_name),
        'worlds',
        'empty.world'
    )    
    
    world = LaunchConfiguration('world')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )
    
    joint_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_config.toxml(),
                'use_sim_time': True
            }
        ],
        output="screen"
    )
    
    rviz2 = Node(
        package="rviz2", executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )

    spawn_entity = Node(
        package='ros_gz_sim', 
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'my_bot', '-z', '0.5'],
        output='screen'
    )
   
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", robot_controllers],
        parameters=[{'use_sim_time': True}]
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--param-file", robot_controllers],
        parameters=[{'use_sim_time': True}]
    )
    
    start_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster,
                velocity_controller,
            ]
        )
    )
    
    control_node = Node(
        package="hanuman_control",
        executable="hanuman_control",
        name="cascade_controller_publisher",
        output="screen"
    )

    return LaunchDescription([
        joint_state_publisher,
        # rviz2, 
        world_arg,
        gazebo,
        spawn_entity,
        start_controllers,
        control_node,        
    ])