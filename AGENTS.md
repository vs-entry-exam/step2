# AGENT STATUS

**Last Updated:** 2025-09-19

## Goal
- myAGV Pro: run Gazebo + SLAM + Nav2 end-to-end while preparing the real-sensor data collection pipeline.
- myCobot 280 M5: run Gazebo + RViz (MoveIt motion planning) reliably and extend the pipeline to record robot/manipulation datasets.

## Completed
- Verified the dummy LiDAR pipeline cleanly through bringup/TF and restored `GAZEBO_*` defaults for consistent environment setup (`setup.bash`).
- Added the Gazebo ray sensor on `laser_link`, remapped `~/out` → `/scan`, and confirmed the integrated Gazebo + SLAM + Nav2 launch (`myagv_pro_bringup/sim_slam_nav2.launch.py`) works end-to-end.
- Authored `indoor_room.world`, set it as the default world, spawned the AGV at room center, and raised the lidar mount by +0.05 m to keep the chassis out of the laser field.
- Brought up Gazebo + Nav2 + RViz, validated topic flow, initialized pose, and confirmed Nav2 goals produce `/cmd_vel` commands.
- Migrated the `agv_pro_ros2` stack into `step2/src/` and verified `sim_slam_nav2.launch.py` runs successfully via the step2 workspace.
- Introduced `base_footprint` in `mycobot_280_m5_gazebo.urdf.xacro`, anchoring the pedestal so the robot no longer collides with the ground on spawn and redirecting Gazebo friction/static params to that footprint.
- Aligned `joint4`–`joint6` origins and limits with the reference hardware URDF so the myCobot arm now spawns with correctly connected geometry.
- Rebuilt the `base_footprint` pad (0.01 m) and adjusted the fixed-joint offset so myCobot now spawns flush with the floor even with physics enabled.
- Cloned fresh copies of `mycobot_ros2` and `agv_pro_ros2` into `step2/src/` and added an initial `mycobot_280_bringup` package with a Gazebo + MoveIt launch skeleton.

## Current Status & Issues
- AGV simulation is stable; swapping in the production laser model remains outstanding and will require topic/type bridging as needed.
- myCobot arm links line up correctly and sit flush with the floor, but the new Gazebo + MoveIt launch (step2) still loads mismatched URDFs (`mycobot_280_m5_gazebo` vs `firefighter`) so TF frames diverge and the arm collapses in Gazebo while MoveIt holds the init pose.
- ros2_control fails to start in the step2 launch (no `/controller_manager` service), so MoveIt trajectory execution cannot actuate joints yet.
- AGV simulation remains stable; production laser model swap still outstanding.

## Next Steps
1. Mirror the AGV bringup pattern: duplicate the Gazebo xacro into `step2/mycobot_280_bringup`, align the robot name/root frame with MoveIt, remove the root inertial, and make both Gazebo/MoveIt pull the same `robot_description`.
2. Inject `gazebo_ros2_control` + controller config (joint_state_broadcaster + arm_group_controller) so `/controller_manager` comes up and MoveIt trajectories can drive the Gazebo model.
3. Once motion planning succeeds, document the unified launch and revisit dataset logging requirements.
4. Resume the production laser swap-in for myAGV Pro, documenting any bridges or topic remaps needed.

## Repository Guidelines

### Project Structure & Module Organization
This repository records training data tooling that complements `agv_ws` and `cobot_ws`. Keep top-level files lean: docs such as `README.md` and `AGENTS.md` sit at the root. Place new ROS 2 packages under `src/<package_name>/` with the usual `package.xml` and `CMakeLists.txt` (C++) or `setup.py` (Python). Store shared launch files in `launch/`, parameter sets in `config/`, and sample datasets in `data/` or `assets/` while ignoring large raw logs through `.gitignore`. Document any new folder in `README.md` so other agents immediately know where to look.

### Build, Test, and Development Commands
Always initialize your environment before building:
```
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/agv_ws/install/setup.bash
source ~/cobot_ws/install/setup.bash
source ~/step2/install/setup.bash
```
Use `colcon build --symlink-install` for a full workspace build, and `colcon build --packages-select my_package` when iterating on a single package. Launch sanity checks with `ros2 run rviz2 rviz2` or `ros2 launch gazebo_ros gazebo.launch.py` after sourcing the workspace to verify integrations. Prefer adding helper aliases to `~/.bashrc` rather than storing machine-specific scripts in the repo.

### Coding Style & Naming Conventions
Follow ROS 2 style guides: 4-space indentation, UpperCamelCase for C++ class names, snake_case for Python nodes and files, and kebab-case for launch filenames. Enable `ament_lint_auto` and run `ament_uncrustify`/`clang-format` for C++, `black` and `flake8` for Python modules. Keep package names lowercase with underscores (e.g., `vla_dataset_tools`) and align message/service file names with their intent (`CollectVLAData.srv`).

### Testing Guidelines
Add `ament_add_gtest` or `ament_cmake_pytest` cases alongside implementation files, mirroring the package name (e.g., `src/collector.cpp` → `test/test_collector.cpp`). Run `colcon test` before opening a pull request and ensure critical nodes have coverage via `--pytest-with-coverage` or `lcov` exports shared in the PR description.

### Commit & Pull Request Guidelines
Follow the existing Conventional Commit style (`doc:`, `chore:`, `feat:`) with a concise imperative subject under 72 characters. Each pull request should describe motivation, link any tracking issue, list testing commands executed, and include screenshots or bag file references when UI or simulation results change. Request reviews from maintainers responsible for the affected workspace and keep PRs scoped so they build and test cleanly on their own.

#### Step2 Commit Rules
- Prefix each commit with the appropriate Conventional Commit type (`feat:`, `fix:`, `docs:`, `refactor:`, `chore:` 등) and keep the subject ≤ 72자.
- 하나의 논리 작업 단위만 담고, 관련 테스트/검증 명령을 커밋 메시지 본문에 추가.
- 빌드/런치 확인이 필요한 변경은 `ros2 launch` 혹은 `colcon build` 결과를 명시하고 실패 시 해결 후 커밋.
- 외부 워크스페이스(overlays)와 중복되는 패키지를 수정했을 경우, 커밋 본문에 오버레이 동작/검증 방법을 남긴다.
