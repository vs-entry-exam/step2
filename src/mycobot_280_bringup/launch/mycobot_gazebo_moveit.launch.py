"""Launch Gazebo + MoveIt2 for myCobot 280 M5."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    description_pkg = FindPackageShare('mycobot_description')
    moveit_pkg = FindPackageShare('mycobot_280_moveit2')

    xacro_file = PathJoinSubstitution([
        description_pkg,
        'urdf',
        'mycobot_280_m5',
        'mycobot_280_m5_gazebo.urdf.xacro',
    ])

    robot_description_content = Command(['xacro ', xacro_file])
    robot_description = {
        'robot_description': ParameterValue(robot_description_content, value_type=str),
        'use_sim_time': True,
    }

    controllers_file = PathJoinSubstitution([
        moveit_pkg,
        'config',
        'ros2_controllers.yaml',
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py',
            ])
        )
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_file, {'robot_description': robot_description['robot_description']}],
        output='screen',
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mycobot_280', '-topic', 'robot_description'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_group_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_group_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner, arm_group_controller_spawner],
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                moveit_pkg,
                'launch',
                'move_group.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    moveit_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                moveit_pkg,
                'launch',
                'moveit_rviz.launch.py',
            ])
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
    )

    return LaunchDescription(
        [
            gazebo_launch,
            robot_state_publisher,
            ros2_control_node,
            spawn_entity,
            start_controllers,
            move_group_launch,
            moveit_rviz_launch,
        ]
    )
