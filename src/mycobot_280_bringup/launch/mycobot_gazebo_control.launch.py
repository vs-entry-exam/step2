"""Bring up Gazebo with ros2_control for myCobot 280 M5."""

from pathlib import Path

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory('mycobot_280_bringup'))
    gazebo_share = Path(get_package_share_directory('gazebo_ros'))

    xacro_file = bringup_share / 'urdf' / 'gazebo' / 'mycobot_280_m5_gazebo.urdf.xacro'
    robot_description = {
        'robot_description': xacro.process_file(str(xacro_file)).toxml()
    }
    use_sim_time = {'use_sim_time': True}

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(gazebo_share / 'launch' / 'gazebo.launch.py')
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description, use_sim_time],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mycobot_280', '-topic', 'robot_description'],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_group_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster, arm_group_controller],
        )
    )

    return LaunchDescription(
        [
            gazebo,
            robot_state_publisher,
            spawn_entity,
            start_controllers,
        ]
    )
