# Step2 – VLA Train Data

### Project Goals

* **myAGV Pro**: Gazebo + SLAM + Nav2를 end-to-end로 구동하고, 실센서/시뮬 센서 모두에서 **내비게이션 데이터**(맵, 주행 궤적, `/scan`·`/odom`·`/tf`·`/cmd_vel`)를 수집할 수 있게 한다.
* **myCobot 280 M5**: Gazebo + MoveIt(플래닝/실행)을 안정화하고, **조작 데이터**(RGB/Joint states/Trajectory) 기록 파이프라인을 준비한다.
* **VLA 학습 대비**: 조작/주행의 \*\*입력(관측)\*\*과 **출력(액션)** 토픽을 명확히 정의하고, 재현 가능한 **런치/수집 절차**를 문서화한다.



### Artifacts

* AGV: Gazebo 맵, SLAM, Nav2 Goal, 경로/TF 뷰
  * <img width="2560" height="1440" alt="myagv_gzb_slm_nv" src="https://github.com/user-attachments/assets/380e00ef-1a48-45d2-b4d2-c46d20e1cae8" />
  * ![rosgraph_agv](https://github.com/user-attachments/assets/89ae825d-331a-4780-baee-8b964a4e8c11)
  * [myAGV Gazebo+SLAM+Nav2 Demo](https://youtu.be/KWQHvcB6-xM)

* Cobot: MoveIt 플래닝 장면, Gazebo 실행 반영
  * <img width="2560" height="1440" alt="mycobot_gzb_mp" src="https://github.com/user-attachments/assets/2da665b7-1c65-4536-a97c-65ba390b54f6" />
  * ![rosgraph_cobot](https://github.com/user-attachments/assets/6a83c474-962d-4bd2-9ed2-fe04d1b87385)
  * [myCobot Gazebo+MoveIt Demo](https://youtu.be/ChGDlB8bcLQ)





### Prerequisites

```
Ubuntu 22.04
ROS2 Humble
Gazebo, MoveIt2, Nav2, SLAM Toolbox
```

### Dependencies

* [agv\_pro\_ros2](https://github.com/elephantrobotics/agv_pro_ros2.git)
* [mycobot\_ros2](https://github.com/elephantrobotics/mycobot_ros2.git)



### Shell Configuration (Optional but Recommended)

개발 편의를 위해 아래를 `~/.bashrc`에 추가하세요.

```bash
#==================================================================#
# ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/step2/install/setup.bash

#=================================#
# Build aliases
alias cb='colcon build --symlink-install'

# Build only a specific package (usage: cbpkg <package_name>)
cbpkg() {
  colcon build --symlink-install --packages-select "$1"
}

#=================================#
# ROS2 aliases
alias rz='ros2 run rviz2 rviz2'
alias gz='ros2 launch gazebo_ros gazebo.launch.py'
alias rd='ros2 doctor'
alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias rnl='ros2 node list'
alias rqtall='(ros2 run rqt_gui rqt_gui & ros2 run rqt_graph rqt_graph &)'
```



### Build

```bash
# 1) 워크스페이스 루트에서 빌드
cd ~/step2
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 2) 환경 적용
source install/setup.bash
```



### How to Run

#### A. AGV – Gazebo + SLAM + Nav2 (시뮬 end-to-end)

```bash
source /opt/ros/humble/setup.bash
source ~/step2/install/setup.bash

# 예시: 통합 런치 (리포의 bringup 런치 기준 이름에 맞춰 수정)
ros2 launch myagv_pro_bringup sim_slam_nav2.launch.py

# 검증
ros2 topic list | grep -E "/scan|/map|/cmd_vel|/tf"
# RViz에서 초기 위치 설정 후 Nav2 Goal 클릭 → /cmd_vel 생성 확인
```



#### B. Cobot – MoveIt Only (플래닝/실행 빠른 실험)

```bash
source /opt/ros/humble/setup.bash
source ~/step2/install/setup.bash

ros2 launch mycobot_280_bringup mycobot_moveit_only.launch.py

# RViz MotionPlanning: Start=Current, Plan → Execute
```




#### C. Cobot – Gazebo + MoveIt 통합 (시뮬에서 실제 실행 반영)

```bash
source /opt/ros/humble/setup.bash
source ~/step2/install/setup.bash

ros2 launch mycobot_280_bringup mycobot_gazebo_moveit_all.launch.py

# 상태 점검
ros2 control list_controllers
# → joint_state_broadcaster, arm_group_controller = active

ros2 action list | grep follow_joint_trajectory
# → /arm_group_controller/follow_joint_trajectory

ros2 param get /move_group moveit_simple_controller_manager.controller_names
# → ['arm_group_controller']
```




### Troubleshooting

* **메모리 부족(OOM)로 빌드 중단**: 병렬도 축소/스왑 확장/패키지 선택 빌드

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1
# 또는
export MAKEFLAGS=-j1
colcon build --symlink-install --packages-select <패키지명>
```



### Project Limitations 

* omni 를 nav2에서 활용할 수 없다.
* Plan & Excute 완료 후에도 재플랜 되는 문제가 있다.
* Web 로봇 시뮬레이션 미구현
* VLA 학습용 데이터 파이프라인 미구현



