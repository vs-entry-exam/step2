# Step 2: AGV/Cobot 시뮬레이션 및 데이터 수집 환경 구축

## 1. 프로젝트 개요

ROS2와 Gazebo를 사용하여 AGV(Automated Guided Vehicle)와 Cobot(myCobot 280)의 시뮬레이션 환경을 구축합니다. 최종적으로 VLA(Vision-Language-Action) 모델 학습에 필요한 주행 및 조작 데이터를 수집하는 것을 목표로 합니다.

## 2. 목표

- **AGV (자율 주행)**:
    - **2D SLAM**: `SLAM Toolbox`를 사용하여 Gazebo 가상 환경의 2D 지도를 생성합니다.
    - **2D Navigation**: 생성된 지도를 기반으로 `Nav2` 스택을 활용하여 목적지까지 자율 주행을 구현합니다.
    - **데이터 수집**: 주행 데이터(`/scan`, `/odom`, `/tf`, `/cmd_vel`)를 수집할 수 있는 파이프라인을 구축합니다.

- **Cobot (로봇 팔 조작)**:
    - **플래닝 및 실행**: `MoveIt2`를 사용하여 로봇 팔의 동작을 계획하고 Gazebo 시뮬레이션에 정확히 반영합니다.
    - **데이터 수집**: 조작 데이터(RGB 이미지, Joint states, Trajectory) 기록을 위한 기반을 마련합니다.

## 3. 결과물 (Artifacts)

### 시스템 구성도

![시스템 구성도](https://github.com/user-attachments/assets/3a2727a2-a072-4130-8f86-4f49fe742da8)

### 실행 화면

**AGV: Gazebo + SLAM + Nav2**
![AGV 시뮬레이션 화면](https://github.com/user-attachments/assets/380e00ef-1a48-45d2-b4d2-c46d20e1cae8)

**Cobot: Gazebo + MoveIt**
![Cobot 시뮬레이션 화면](https://github.com/user-attachments/assets/2da665b7-1c65-4536-a97c-65ba390b54f6)

### 실행 영상

- **[AGV 시연 영상](https://youtu.be/KWQHvcB6-xM)**
- **[Cobot 시연 영상](https://youtu.be/ChGDlB8bcLQ)**

## 4. 핵심 패키지

- `myagv_pro_bringup`: AGV의 SLAM, Navigation 등 주요 기능을 통합 실행하는 런치 파일을 포함합니다.
- `mycobot_280_bringup`: myCobot의 MoveIt, Gazebo 연동을 위한 런치 파일을 포함합니다.
- `agv_pro_gazebo`: AGV의 Gazebo 시뮬레이션 월드 및 로봇 모델을 로드합니다.
- `navigation2`, `slam_toolbox`, `moveit2`: 자율 주행 및 로봇 팔 제어를 위한 핵심 ROS2 프레임워크입니다.

## 5. 실행 방법

### AGV: Gazebo + SLAM + Nav2 통합 실행

```bash
# 워크스페이스 환경 설정
source ~/step2/install/setup.bash
# 통합 런치 파일 실행
ros2 launch myagv_pro_bringup sim_slam_nav2.launch.py
```

### Cobot: Gazebo + MoveIt 통합 실행

```bash
# 워크스페이스 환경 설정
source ~/step2/install/setup.bash
# 통합 런치 파일 실행
ros2 launch mycobot_280_bringup mycobot_gazebo_moveit.launch.py
```

## 6. 자체 평가 및 개선 방향

### 미비한 점

- **데이터 파이프라인 미완성**: VLA 학습에 필요한 데이터를 `ros2 bag` 등으로 기록하고 가공하는 자동화된 파이프라인이 구현되지 않았습니다.
- **시뮬레이션 한계**: Omni-drive AGV의 특성이 Nav2에 완전히 반영되지 않았고, MoveIt의 플래닝/실행 안정성이 부족한 경우가 있습니다.
- **통합 시나리오 부재**: AGV와 Cobot이 연동되는 복합적인 작업 시나리오가 없습니다.

### 개선 방향

- **데이터 수집 파이프라인 구축**: `ros2 bag`을 이용해 정의된 토픽들을 자동으로 기록하고, 학습에 적합한 포맷으로 변환하는 스크립트를 개발합니다.
- **시뮬레이션 고도화**: Nav2의 Controller 플러그인을 Omni-drive에 최적화된 것으로 교체/튜닝하고, MoveIt의 재플래닝 문제를 해결하여 안정성을 높입니다.
- **통합 시나리오 개발**: AGV가 특정 위치로 이동한 후 Cobot이 물체를 집는 등, 두 로봇이 협력하는 통합 시뮬레이션 환경을 구축하여 보다 복잡한 데이터셋을 확보합니다.