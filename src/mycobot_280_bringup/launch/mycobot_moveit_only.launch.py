"""Launch move_group and RViz for myCobot 280 M5 using bringup-local configs."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetLaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)

# add:
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    bringup_share = Path(get_package_share_directory('mycobot_280_bringup'))

    moveit_builder = (
        MoveItConfigsBuilder('mycobot_280_m5', package_name='mycobot_280_bringup')
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics('config/moveit/kinematics.yaml')
        .joint_limits('config/moveit/joint_limits.yaml')
        .planning_pipelines(
            default_planning_pipeline='ompl', pipelines=['ompl'], load_all=False
        )
        .trajectory_execution('config/moveit/moveit_controllers.yaml')
        .pilz_cartesian_limits('config/moveit/pilz_cartesian_limits.yaml')
        .parameter('use_sim_time', True)
        .yaml(
            'config/moveit/trajectory_execution.yaml',
            parameter_namespace='trajectory_execution',
        )
        .yaml(
            'config/moveit/planning_scene_monitor.yaml',
            parameter_namespace='planning_scene_monitor',
        )
        .yaml('config/moveit/ompl_planning.yaml', parameter_namespace='ompl')
    )

    moveit_config = moveit_builder.to_moveit_configs()

    # Gazebo 없이 로봇 모델/TF를 띄우기 위해 robot_state_publisher + joint_state_publisher 추가
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],  # URDF 로딩
        output="screen",
    )
    joint_state_pub = Node(
        package="joint_state_publisher",               # 필요 시 joint_state_publisher_gui로 교체 가능
        executable="joint_state_publisher",
        output="screen",
    )

    move_group_launch = generate_move_group_launch(moveit_config)
    moveit_rviz_launch = generate_moveit_rviz_launch(moveit_config)

    return LaunchDescription(
        [
            SetLaunchConfiguration(
                'rviz_config',
                str(bringup_share / 'config' / 'moveit' / 'moveit.rviz'),
            ),
            robot_state_pub,
            joint_state_pub,
            *move_group_launch.entities,
            *moveit_rviz_launch.entities,
        ]
    )
