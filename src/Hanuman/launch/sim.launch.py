import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    package_name = "hanuman04"

    rviz_config = os.path.join(get_package_share_directory(
        package_name), "config", "humanoid" + ".rviz")
    
    robot_description = os.path.join(get_package_share_directory(
        package_name), "urdf", "hanuman" + ".urdf.xacro")
    robot_description_config = xacro.process_file(robot_description)
    
    joint_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_config.toxml()}],
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
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description','-entity', 'robot'],
        output='screen'
    )
   
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    velocity_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controller", "--controller-manager", "/controller_manager"],
    )

    return LaunchDescription([
        joint_state_publisher,
        rviz2,
        gazebo,
        spawn_entity,
        joint_state_broadcaster,
        velocity_controller,
    ])
