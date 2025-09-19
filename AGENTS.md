# AGENT STATUS

**Last Updated:** 2025-09-20

## Goal

* **myAGV Pro**: Gazebo + SLAM + Nav2 end-to-end 구동, 실센서 데이터 수집 파이프라인 준비.
* **myCobot 280 M5**: Gazebo + MoveIt(플래닝/실행) 안정 구동, 조작 데이터셋 기록 파이프라인 확장.

## Completed

* `agv_pro_ros2`를 `step2/src/`로 이관, `sim_slam_nav2.launch.py`로 **Gazebo + SLAM + Nav2** E2E 확인.
* `indoor_room.world` 기본 월드 설정, 시작 포즈 중앙 배치, LiDAR 마운트 +0.05 m 상향, `/scan` 리맵 검증.
* myCobot: `base_footprint` 도입 및 바닥 접촉 안정화, `joint4–6` 원점/리밋 정합, ros2\_control 트랜스미션 추가.
* 리소스 정리: `urdf/gazebo` / `urdf/moveit` 분리, MoveIt YAML/RViz를 `config/moveit/`로 통일.
* bringup 로컬 패키지 `mycobot_280_bringup` 생성:

  * **통합 런치** `mycobot_gazebo_moveit.launch.py`로 Gazebo → 스폰 → 컨트롤러 활성 → MoveIt + RViz 순차 기동.
  * MoveIt 컨트롤러 매핑을 `move_group_controllers.params.yaml`(노드 스코프 포함)로 주입.
  * 모든 노드 `use_sim_time:=true`, `/joint_states`는 **joint\_state\_broadcaster 단일 퍼블리셔**로 일원화.
  * **Plan & Execute가 Gazebo 팔에 적용**됨(SUCCEEDED 확인).

## Current Status & Issues

* 통합 구동 성공(Plan/Execute OK).
* 경미 이슈:

  * RViz `Object Recognition` 경고(미사용 디스플레이) → 비활성화로 제거 가능.
  * Plan & Excute 완료 후에도 Planning 반복 현상

## Next Steps

1. **문서화**
   * `README.md`에 **Gazebo 단독 / MoveIt 단독 / 통합 런치** 절차와 검증 명령 표준화.
   * 실행화면 스크린샷 및 녹화 등

2. **데이터 파이프라인**
   * 조작/주행 로그 → RLDS 변환 스크립트 초안 및 토픽·주기·타임스탬프 정책 명세.

## How to Run (Quick)

```bash
# 환경
source /opt/ros/humble/setup.bash
source ~/step2/install/setup.bash

# 통합 런치
ros2 launch myagv_pro_bringup sim_slam_nav2.launch.py               # myAGV Pro
ros2 launch mycobot_280_bringup mycobot_gazebo_moveit.launch.py     # myCobot 280
```

## Repository Guidelines

* **구조**: 새 ROS 2 패키지는 `src/<pkg>/`, 런치 `launch/`, 파라미터 `config/`, 샘플 데이터 `data/|assets/`. 대용량 로그는 `.gitignore`.
* **빌드/실행**: `colcon build --symlink-install`, 단일 패키지 `--packages-select`. 스모크 테스트는 `rviz2`, `gazebo.launch.py`.
* **스타일**: ROS 2 규칙(4-space, Python snake\_case, launch kebab-case). `ament_lint_auto`, `black/flake8` 사용.
* **테스트**: `colcon test` 수행, 핵심 노드 커버리지 보고.
* **커밋/PR**: Conventional Commits(예: `doc:`, `feat:`, `fix:` 등), subject ≤ 72자, 변경 동기/테스트 명령/스크린샷(또는 bag) 첨부. 오버레이 수정 시 검증 방법 기재.
