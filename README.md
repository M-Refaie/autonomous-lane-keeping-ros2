# Autonomous Lane-Keeping Robot (ROS 2 + Gazebo)

A differential-drive robot that drives autonomously, holds its lane using IMU feedback, detects obstacles ahead with a LiDAR, and performs a smooth **lane-switch maneuver** to avoid them — all in **ROS 2 (Jazzy)** and simulated in **Gazebo**.

## Demo
▶ **[Watch the robot detect an obstacle and switch lanes (Gazebo)](https://drive.google.com/file/d/1N6GTgnALf1WMGJvGRewoFD-KnvfYr0Te/view)**

## What it does
- **Cruises straight** while a **P-controller on IMU yaw** keeps the heading locked to 0 rad (quaternion → yaw).
- **Watches the road ahead** by windowing the front sector of the `/scan` LiDAR (center ± 10 beams, filtering invalid/again-out-of-range returns).
- **When an obstacle comes within 1.5 m**, runs a **3-stage finite-state lane-switch**:
  1. **Turn** to a 45° heading (~0.8 s),
  2. **Cross** into the other lane at higher speed (~1.8 s),
  3. **Complete** — re-lock heading and start a 4 s cooldown before the next switch.

## Architecture
| | |
|---|---|
| **Node** | `lane_switcher` (`rclpy`) |
| **Subscribes** | `/scan` (`sensor_msgs/LaserScan`), `/imu` (`sensor_msgs/Imu`) |
| **Publishes** | `/cmd_vel` (`geometry_msgs/Twist`) |
| **Control** | P-controller (yaw stabilization) + timed FSM (lane switch) |
| **Robot** | differential drive via `ros2_control` (`diff_drive_controller.yaml`) |
| **Sim** | Gazebo (`ros_gz_sim`), launched from `robot_sim.launch.py` |

## Tech stack
ROS 2 (Jazzy) · rclpy · Gazebo / ros_gz · ros2_control (diff_drive_controller) · sensor fusion (LiDAR + IMU) · Python

## Repo structure
```
src/diff_drive_pkg/
├── diff_drive_pkg/lane_switcher.py   # the control node (perception + FSM + P-control)
├── launch/robot_sim.launch.py        # spawns the robot + controllers in Gazebo
├── config/diff_drive_controller.yaml # ros2_control differential-drive config
├── package.xml  setup.py  setup.cfg
```

## Build & run
```bash
cd autonomous-lane-keeping-ros2
colcon build --packages-select diff_drive_pkg
source install/setup.bash
ros2 launch diff_drive_pkg robot_sim.launch.py   # bring up the robot in Gazebo
ros2 run diff_drive_pkg lane_switcher            # start autonomous lane-keeping
```

## Key tuning parameters (in `lane_switcher.py`)
`linear_speed=0.5` · `obstacle_distance=1.5 m` · `P_GAIN=0.4` · `ANGLE_SETPOINT=45°` · `COOLDOWN_DELAY=4 s`

---
## Credits
Team project at Ain Shams University (Mechatronics, Design of Autonomous Systems / MCT443): Mohamed Refaie, Haneen Amr Ahmed, Touqa Moustafa Sayed, and Tuqa Nasr El-Din.
