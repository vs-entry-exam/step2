# Step2 - VLA train data

## Prerequisites
```
Ubuntu 22.04
ROS2 Humble
Gazebo, MoveIt2, Nav2, SLAM Toolbox
```

## Dependencies
- [agv_pro_ros2](https://github.com/elephantrobotics/agv_pro_ros2.git)  
- [mycobot_ros2](https://github.com/elephantrobotics/mycobot_ros2.git)

## Shell Configuration (Optional but Recommended)

For faster development workflow, you can add the following aliases and functions
into your `~/.bashrc`

```bash
#==================================================================#
# ROS2
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/agv_ws/install/setup.bash
source ~/cobot_ws/install/setup.bash

#=================================#
# Build aliases
alias cb='colcon build --symlink-install'

# Build only a specific package (usage: cbpkg <package_name>)
cbpkg() {
  colcon build --symlink-install --packages-select "$1"
}

#=================================#
# Workspace switching (usage: ws agv / ws cobot / ws ros2)
ws() {
    case "$1" in
        agv)
            source ~/agv_ws/install/setup.bash
            echo "[ws] Switched to agv_ws"
            ;;
        cobot)
            source ~/cobot_ws/install/setup.bash
            echo "[ws] Switched to cobot_ws"
            ;;
        ros2)
            source ~/ros2_ws/install/setup.bash
            echo "[ws] Switched to ros2_ws"
            ;;
        *)
            echo "Usage: ws {agv|cobot|ros2}"
            ;;
    esac
}

#=================================#
# ROS2 aliases
# Quick launch RViz2 and Gazebo
alias rz='ros2 run rviz2 rviz2'
alias gz='ros2 launch gazebo_ros gazebo.launch.py'

# ROS2 health check
alias rd='ros2 doctor'

# Frequently used topic/service/action commands
alias rtl='ros2 topic list'
alias rte='ros2 topic echo'
alias rnl='ros2 node list'

# Run rqt and rqt_graph simultaneously
alias rqtall='(ros2 run rqt_gui rqt_gui & ros2 run rqt_graph rqt_graph &)'
```

