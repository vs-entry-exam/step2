import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_slam = LaunchConfiguration('launch_slam')
    launch_nav2 = LaunchConfiguration('launch_nav2')
    map_yaml = LaunchConfiguration('map')
    nav2_params = LaunchConfiguration('nav2_params')
    use_ros2_control = LaunchConfiguration('use_ros2_control')

    gazebo_pkg = get_package_share_directory('agv_pro_gazebo')
    nav2_pkg = get_package_share_directory('agv_pro_navigation2')
    world = LaunchConfiguration('world')
    default_world = os.path.join(gazebo_pkg, 'worlds', 'indoor_room.world')
    xacro_file_default = os.path.join(gazebo_pkg, 'urdf', 'agv_pro.xacro')
    xacro_file_ros2_control = os.path.join(
        get_package_share_directory('myagv_pro_bringup'), 'urdf', 'agv_pro_with_ros2_control.xacro')
    rviz_config = os.path.join(nav2_pkg, 'rviz', 'agvpro_navigation2.rviz')
    default_map = os.path.join(nav2_pkg, 'map', 'map.yaml')
    default_nav2_params = os.path.join(nav2_pkg, 'param', 'agvpro.yaml')

    selected_xacro = PythonExpression([
        "'", use_ros2_control, "' == 'true' and '", xacro_file_ros2_control, "' or '", xacro_file_default, "'"
    ])
    robot_description_content = ParameterValue(
        # Command(['xacro ', selected_xacro]),
        Command(['xacro ', xacro_file_ros2_control]),
        # Command(['xacro ', xacro_file_default]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}
    use_sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world,
            'gui': gui
        }.items()
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'myagv_pro',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time_param}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time_param}]
    )

    slam_gmapping = Node(
        package='slam_gmapping',
        executable='slam_gmapping',
        name='slam_gmapping',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time_param,
             'base_frame': 'base_footprint',
             'odom_frame': 'odom',
             'map_frame': 'map'}
        ],
        condition=IfCondition(launch_slam)
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_yaml,
            'params_file': nav2_params,
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items(),
        condition=IfCondition(launch_nav2)
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time_param}],
        output='screen',
        condition=IfCondition(launch_rviz)
    )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_joint_state_broadcaster',
        arguments=['fishbot_joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ros2_control)
    )

    load_mecanum_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        name='spawner_mecanum_drive_controller',
        arguments=['fishbot_mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ros2_control)
    )

    joint_state_spawner_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster]
        ),
        condition=IfCondition(use_ros2_control)
    )

    mecanum_controller_spawner_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_mecanum_drive_controller]
        ),
        condition=IfCondition(use_ros2_control)
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time provided by /clock.'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Full path to the Gazebo world file to load.'
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Launch the Gazebo client GUI.'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz with the navigation display configuration.'
        ),
        DeclareLaunchArgument(
            'launch_slam',
            default_value='true',
            description='Launch slam_gmapping to publish map→odom.'
        ),
        DeclareLaunchArgument(
            'launch_nav2',
            default_value='true',
            description='Launch Nav2 (bringup_launch.py).' ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map,
            description='Full path to the map YAML file for Nav2.'
        ),
        DeclareLaunchArgument(
            'nav2_params',
            default_value=default_nav2_params,
            description='Full path to the Nav2 parameter file.'
        ),
        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='false',
            description='Load ros2_control controllers through controller_manager spawners.'
        ),
        gazebo_launch,
        spawn_entity,
        robot_state_publisher,
        joint_state_publisher,
        slam_gmapping,
        nav2_launch,
        rviz,
        joint_state_spawner_handler,
        mecanum_controller_spawner_handler,
    ])
