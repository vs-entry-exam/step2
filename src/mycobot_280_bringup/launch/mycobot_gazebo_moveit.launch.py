# """Launch Gazebo + MoveIt2 for myCobot 280 M5."""

# from pathlib import Path

# import xacro
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from moveit_configs_utils import MoveItConfigsBuilder
# from moveit_configs_utils.launches import (
#     generate_move_group_launch,
#     generate_moveit_rviz_launch,
# )


# def generate_launch_description() -> LaunchDescription:
#     bringup_share = Path(get_package_share_directory('mycobot_280_bringup'))
#     moveit_share = Path(get_package_share_directory('mycobot_280_moveit2'))
#     gazebo_share = Path(get_package_share_directory('gazebo_ros'))

#     xacro_file = bringup_share / 'urdf' / 'gazebo' / 'mycobot_280_m5_gazebo.urdf.xacro'
#     srdf_file = bringup_share / 'config' / 'mycobot_280_m5.srdf'

#     robot_description = {
#         'robot_description': xacro.process_file(str(xacro_file)).toxml()
#     }
#     use_sim_time = {'use_sim_time': True}

#     gazebo_launch = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             str(gazebo_share / 'launch' / 'gazebo.launch.py')
#         )
#     )

#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         parameters=[robot_description, use_sim_time],
#         output='screen',
#     )

#     spawn_entity = Node(
#         package='gazebo_ros',
#         executable='spawn_entity.py',
#         arguments=['-entity', 'mycobot_280', '-topic', 'robot_description'],
#         output='screen',
#     )

#     joint_state_broadcaster_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
#         output='screen',
#     )

#     arm_group_controller_spawner = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['arm_group_controller', '--controller-manager', '/controller_manager'],
#         output='screen',
#     )

#     start_controllers = RegisterEventHandler(
#         OnProcessExit(
#             target_action=spawn_entity,
#             on_exit=[joint_state_broadcaster_spawner, arm_group_controller_spawner],
#         )
#     )

#     moveit_config_builder = MoveItConfigsBuilder(
#         'firefighter', package_name='mycobot_280_moveit2'
#     )
#     moveit_config_builder.parameter('use_sim_time', True)
#     moveit_config = moveit_config_builder.to_moveit_configs()
#     moveit_config.robot_description = robot_description
#     moveit_config.robot_description_semantic = {
#         'robot_description_semantic': srdf_file.read_text()
#     }

#     move_group_launch = generate_move_group_launch(moveit_config)
#     moveit_rviz_launch = generate_moveit_rviz_launch(moveit_config)

#     return LaunchDescription(
#         [
#             gazebo_launch,
#             robot_state_publisher,
#             spawn_entity,
#             start_controllers,
#             *move_group_launch.entities,
#             *moveit_rviz_launch.entities,
#         ]
#     )

# from launch import LaunchDescription
# from launch.actions import TimerAction
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
# from pathlib import Path

# def generate_launch_description():
#     bringup_share = Path(get_package_share_directory('mycobot_280_bringup'))

#     # MoveIt config
#     moveit = (
#         MoveItConfigsBuilder('mycobot_280_m5', package_name='mycobot_280_bringup')
#         .robot_description()
#         .robot_description_semantic()
#         .robot_description_kinematics('config/moveit/kinematics.yaml')
#         .joint_limits('config/moveit/joint_limits.yaml')
#         .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
#         .trajectory_execution('config/moveit/moveit_controllers.yaml')   
#         .pilz_cartesian_limits('config/moveit/pilz_cartesian_limits.yaml')
#         .yaml('config/moveit/ompl_planning.yaml', parameter_namespace='ompl')
#         .yaml('config/moveit/trajectory_execution.yaml', parameter_namespace='trajectory_execution')
#         .yaml('config/moveit/planning_scene_monitor.yaml', parameter_namespace='planning_scene_monitor')
#         .to_moveit_configs()
#     )

#     move_group = Node(
#         package='moveit_ros_move_group',
#         executable='move_group',
#         output='screen',
#         parameters=[
#             moveit.to_dict(),
#             str(bringup_share / 'config' / 'moveit' / 'moveit_controllers.yaml'),
#             {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'},
#             {'use_sim_time': True}
#         ],
#     )
#     rviz = Node(
#         package='rviz2',
#         executable='rviz2',
#         output='screen',
#         arguments=['-d', str(bringup_share / 'config' / 'moveit' / 'moveit.rviz')],
#         parameters=[moveit.robot_description, moveit.robot_description_semantic, moveit.planning_pipelines],
#     )

#     # Gazebo + controllers가 먼저 준비되도록 약간 지연 실행
#     delayed_moveit = TimerAction(period=3.0, actions=[move_group, rviz])
#     return LaunchDescription([delayed_moveit])

# from launch import LaunchDescription
# from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# from moveit_configs_utils import MoveItConfigsBuilder
# from pathlib import Path
# import xacro

# def generate_launch_description():
#     bringup = Path(get_package_share_directory('mycobot_280_bringup'))
#     gazebo = Path(get_package_share_directory('gazebo_ros'))

#     # 1) Gazebo + RSP + Spawn
#     urdf_xacro = bringup / 'urdf' / 'gazebo' / 'mycobot_280_m5_gazebo.urdf.xacro'
#     robot_description_xml = xacro.process_file(str(urdf_xacro)).toxml()

#     gz = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gazebo / 'launch' / 'gazebo.launch.py')))
#     rsp = Node(package='robot_state_publisher', executable='robot_state_publisher',
#                parameters=[{'robot_description': robot_description_xml, 'use_sim_time': True}])
#     spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
#                  arguments=['-entity', 'mycobot_280', '-topic', 'robot_description'])

#     # 2) ros2_control 컨트롤러 스포너(타임아웃 포함)
#     jsb = Node(package='controller_manager', executable='spawner',
#                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
#                           '--controller-manager-timeout', '20'])
#     arm = Node(package='controller_manager', executable='spawner',
#                arguments=['arm_group_controller', '--controller-manager', '/controller_manager',
#                           '--controller-manager-timeout', '20'])

#     start_ctrl = RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb, arm]))

#     # 3) MoveIt 설정 + “노드 스코프 포함” 컨트롤러 파라미터 파일 주입
#     moveit = (
#         MoveItConfigsBuilder('mycobot_280_m5', package_name='mycobot_280_bringup')
#         .robot_description()  # 이 줄로도 동일 URDF 사용(또는 생략하고 RSP의 것을 사용)
#         .robot_description_semantic()
#         .robot_description_kinematics('config/moveit/kinematics.yaml')
#         .joint_limits('config/moveit/joint_limits.yaml')
#         .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
#         .yaml('config/moveit/ompl_planning.yaml', parameter_namespace='ompl')
#         .yaml('config/moveit/trajectory_execution.yaml', parameter_namespace='trajectory_execution')
#         .yaml('config/moveit/planning_scene_monitor.yaml', parameter_namespace='planning_scene_monitor')
#         .to_moveit_configs()
#     )

#     move_group = Node(
#         package='moveit_ros_move_group', executable='move_group', output='screen',
#         parameters=[moveit.to_dict(),
#                     str(bringup / 'config' / 'moveit' / 'move_group_controllers.params.yaml'),
#                     {'use_sim_time': True}]
#     )
#     rviz = Node(
#         package='rviz2', executable='rviz2', output='screen',
#         arguments=['-d', str(bringup / 'config' / 'moveit' / 'moveit.rviz')],
#         parameters=[moveit.robot_description, moveit.robot_description_semantic, moveit.planning_pipelines],
#     )

#     # 컨트롤러가 올라올 시간을 주고(또는 spawner 끝난 후) MoveIt 실행
#     delayed_moveit = TimerAction(period=3.0, actions=[move_group, rviz])

#     return LaunchDescription([gz, rsp, spawn, start_ctrl, delayed_moveit])

"""Bring up Gazebo + ros2_control + myCobot spawn + controllers + MoveIt2 + RViz (all-in-one)."""

from pathlib import Path
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    RegisterEventHandler, 
    TimerAction, 
    SetEnvironmentVariable, 
    SetLaunchConfiguration
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
)

def generate_launch_description() -> LaunchDescription:
    bringup = Path(get_package_share_directory('mycobot_280_bringup'))
    gazebo  = Path(get_package_share_directory('gazebo_ros'))

    # --- 1) URDF(xacro) → robot_description ---
    urdf_xacro = bringup / 'urdf' / 'gazebo' / 'mycobot_280_m5_gazebo.urdf.xacro'
    robot_description_xml = xacro.process_file(str(urdf_xacro)).toxml()

    # --- 2) Gazebo ---
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gazebo / 'launch' / 'gazebo.launch.py'))
    )

    # 전역 sim time (RViz/MoveIt/Gazebo 시간동기)
    use_sim_time_param = {'use_sim_time': True}

    # --- 3) robot_state_publisher (Gazebo와 공유할 robot_description) ---
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_xml}, use_sim_time_param],
        output='screen',
    )
    # jsp = Node(
    #     package="joint_state_publisher",               
    #     executable="joint_state_publisher",
    #     output="screen",
    # )

    # --- 4) Gazebo에 엔티티 스폰 ---
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'mycobot_280', '-topic', 'robot_description'],
        output='screen',
    )

    # --- 5) ros2_control 컨트롤러 스포너(타임아웃 포함) ---
    jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '20',
            '--unload-on-kill'
        ],
        output='screen',
    )
    arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_group_controller',                 # ← ros2_controllers.yaml와 동일해야 함
            '--controller-manager', '/controller_manager',
            '--controller-manager-timeout', '20',
            '--unload-on-kill'
        ],
        output='screen',
    )

    # 스폰 완료 후 컨트롤러 로드/활성
    start_ctrl = RegisterEventHandler(
        OnProcessExit(target_action=spawn, on_exit=[jsb, arm])
        # OnProcessExit(target_action=spawn, on_exit=[arm])
    )

    # --- 6) MoveIt 설정 빌드 (기본 설정만 병합) ---
    #  !! 컨트롤러 매핑은 별도의 "노드 스코프 포함 params.yaml"로 move_group에 직접 주입 !!
    from moveit_configs_utils import MoveItConfigsBuilder
    moveit = (
        MoveItConfigsBuilder('mycobot_280_m5', package_name='mycobot_280_bringup')
        .robot_description()  # rsp와 동일 URDF 사용
        .robot_description_semantic()
        .robot_description_kinematics('config/moveit/kinematics.yaml')
        .joint_limits('config/moveit/joint_limits.yaml')
        .trajectory_execution('config/moveit/moveit_controllers.yaml')
        # .yaml('config/moveit/moveit_controllers.yaml')
        .pilz_cartesian_limits('config/moveit/pilz_cartesian_limits.yaml')
        .planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])
        .yaml('config/moveit/ompl_planning.yaml', parameter_namespace='ompl')
        .yaml('config/moveit/trajectory_execution.yaml', parameter_namespace='trajectory_execution')
        .yaml('config/moveit/planning_scene_monitor.yaml', parameter_namespace='planning_scene_monitor')
        .to_moveit_configs()
    )

    # MoveIt 컨트롤러 매핑(노드 스코프 포함) 파라미터 파일 경로
    # 파일 내용 예시는 아래 참고 섹션 참조
    # move_group_params = str(bringup / 'config' / 'moveit' / 'move_group_controllers.params.yaml')

    # move_group = Node(
    #     package='moveit_ros_move_group',
    #     executable='move_group',
    #     output='screen',
    #     parameters=[
    #         moveit.to_dict(),
    #         move_group_params,         # ★ 노드 스코프 포함 params.yaml (필수)
    #         use_sim_time_param,
    #     ],
    # )

    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     arguments=['-d', str(bringup / 'config' / 'moveit' / 'moveit.rviz')],
    #     parameters=[
    #         moveit.robot_description,
    #         moveit.robot_description_semantic,
    #         moveit.planning_pipelines,
    #         use_sim_time_param,
    #     ],
    # )

    move_group_launch = generate_move_group_launch(moveit)
    moveit_rviz_launch = generate_moveit_rviz_launch(moveit)

    # 컨트롤러가 active 될 시간을 주고 MoveIt/RViz 실행
    # delayed_moveit = TimerAction(period=3.0, actions=[move_group, rviz])

    return LaunchDescription([
        # Gazebo → RSP → Spawn → Controllers → (delay) MoveIt/RViz
        SetParameter(name='use_sim_time', value=True),
        SetEnvironmentVariable(name='RCUTILS_COLORIZED_OUTPUT', value='1'),
        SetLaunchConfiguration(
            'rviz_config',
            str(bringup / 'config' / 'moveit' / 'moveit.rviz'),
        ),
        gz, rsp, spawn, start_ctrl, 
        # delayed_moveit,
        *move_group_launch.entities, 
        *moveit_rviz_launch.entities,
    ])
