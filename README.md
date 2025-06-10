# Franka Path Planner

ROS 2 package for motion planning and execution on Franka Emika robots using MoveIt 2.



## Installation

### Create workspace and clone repositories

```bash
mkdir -p ~/franka_ws/src
cd ~/franka_ws/src

git clone https://github.com/frankaemika/franka_ros2.git
git clone https://github.com/frankaemika/franka_description.git
git clone https://github.com/kaRpuri/franka_path_planner.git
```

### Install dependencies

```bash
cd ~/franka_ws
rosdep install --from-paths src --ignore-src -y
```

## Build the workspace

```bash
cd ~/franka_ws
colcon build --symlink-install
source install/setup.bash
```

## Run in Simulation Mode

### Terminal 1: Launch MoveIt with fake hardware

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=dont-care \
  use_fake_hardware:=true \
  launch_rviz:=true
```

### Terminal 2: Run planner

```bash
ros2 launch franka_path_planner deploy.launch.py
```

## Run with Real Robot

### Terminal 1: Launch MoveIt with real robot

```bash
ros2 launch franka_fr3_moveit_config moveit.launch.py \
  robot_ip:=<your_robot_ip> \
  use_fake_hardware:=false \
  launch_rviz:=true
```

### Terminal 2: Run planner

```bash
ros2 launch franka_path_planner deploy.launch.py
```

## Custom Trajectories

Edit `src/test_path.cpp`:

```cpp
geometry_msgs::msg::Pose target_pose;
target_pose.position.x = 0.5;
target_pose.position.y = 0.0;
target_pose.position.z = 0.4;
target_pose.orientation.w = 1.0;
```

## Notes

- Always source the workspace before running commands:

```bash
source ~/franka_ws/install/setup.bash
```

- Use parameter remapping if needed:

```bash
ros2 run franka_path_planner test_path \
  --ros-args \
  -p robot_description:=/robot_description \
  -p robot_description_semantic:=/robot_description_semantic
```

## License

Apache License 2.0
