from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

import os
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    ld = LaunchDescription()

    description_package_name = "hanuman04"

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package_name),
            'config',
            'controllers.yaml',
        ]
    )

    rviz_config = os.path.join(get_package_share_directory(
        description_package_name), "config", "humanoid" + ".rviz")

    robot_description = os.path.join(get_package_share_directory(
        description_package_name), "urdf", "hanuman" + ".urdf.xacro")

    robot_description_config = xacro.process_file(
        robot_description,
    )

    joint_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description_config.toxml(),
            }
        ],
        output="screen"
    )

    # Controller configuration
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(description_package_name),
            'config',
            'controllers.yaml',
        ]
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--param-file", robot_controllers],
        parameters=[{'use_sim_time': True}]
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--param-file", robot_controllers],
        parameters=[{'use_sim_time': True}]
    )

    rviz2 = Node(
        package="rviz2", executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    # Dynamixel Hardware Interface
    dynamixel_motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('dynamixel_ros2_multi_motor_control'),
                'launch',
                'dynamixel_hardware_interface.launch.py'
            ])
        ])
    )

    # Control Node
    control_node = Node(
        package='hanuman_control',
        executable='hanuman_control',
        name="single_loop_publisher",
        output="screen",
    )

    trajectory_node = TimerAction(
        period=10.0,  # 10-second delay
        actions=[
            Node(
                package='hanuman_control',
                executable='trajectory_publisher.py',
                name="trajectory_publisher",
                output="screen",
            )
        ]
    )

    # Add actions to launch description
    ld.add_action(joint_state_publisher)
    ld.add_action(dynamixel_motor)
    ld.add_action(control_node)
    ld.add_action(trajectory_node)
    return ld
