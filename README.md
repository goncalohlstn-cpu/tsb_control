# TSB Control — Autonomous Systems Recruitment

## Project Overview

This project was developed for the second phase of the Técnico Solar Boat Autonomous Systems recruitment.

The objective is to implement a ROS 2 control system for a TurtleBot3 Burger robot, using PID controllers for surge velocity and heading. An additional Line-of-Sight (LOS) guidance node generates references for continuous waypoint-based path following.

The system includes:

- Sensor processing using odometry and IMU data, including planar position.
- PID control of surge velocity and heading.
- LOS guidance along a closed loiter trajectory with configurable initial waypoints and live waypoint updates.
- Anti-windup, command saturation, sensor timeout and input validation.
- ROS 2 launch files and YAML configuration for the PID controller and initial LOS waypoints.

## Architecture

The control system consists of **three ROS 2 nodes**:

### 1. Sensor Processing Node

The `sensor_processing_node` subscribes to `/odom` and `/imu` from the TurtleBot3 simulation. It extracts and publishes:

- Surge velocity from odometry (`/tsb/surge`).
- Planar position, `[x, y]`, from odometry (`/tsb/position`).
- Heading angle (yaw) from the IMU orientation quaternion (`/tsb/heading`).
- Yaw rate from IMU angular velocity (`/tsb/yaw_rate`).

### 2. LOS Node

The `los_node` subscribes to `/tsb/position` and `/tsb/waypoints` and computes a desired heading based on the robot's cross-track error relative to the current path segment. It publishes the desired surge velocity and heading in `/tsb/reference` as `[v_d, psi_d]`.

The initial path (loaded from `config/los_params.yaml`) is a **closed eight-waypoint polygon (an octagon)** approximating a circular loiter. When a valid new waypoint vector is received on `/tsb/waypoints`, it replaces the active path and guidance restarts at its first segment. Once the final segment has been followed, guidance returns to the first segment and continues indefinitely. Invalid waypoint vectors are rejected while the previous path remains active.

The LOS calculation uses:

```text
chi_p = atan2(y_end - y_start, x_end - x_start)
e     = -(x - x_start) * sin(chi_p) + (y - y_start) * cos(chi_p)
psi_d = chi_p - atan2(e, lookahead_distance)
```

Here, `chi_p` is the path-segment direction and `e` is the signed cross-track error. The desired heading is wrapped to `[-pi, pi]` before publication.

The node runs at **10 Hz**. It advances to the next segment when the robot is close enough to the current endpoint or has passed the end of the segment within the configured cross-track bound. It requires recent position data; when the position is unavailable or more than one second old, it publishes a zero surge reference while retaining the last heading reference.

### 3. PID Node

The `pid_node` receives the processed sensor data and the reference `[v_d, psi_d]` from the LOS node. It implements:

- A surge PID controller, with desired-speed feedforward.
- A heading controller with proportional and integral action, plus derivative damping based on the measured yaw rate.
- Heading-error wrapping, integral clamping and output saturation.

The node publishes `geometry_msgs/msg/Twist` commands to `/cmd_vel` at **10 Hz**.

### Data Flow

```text
Gazebo / TurtleBot3
   | /odom, /imu
   v
sensor_processing_node
   |                        \
   | /tsb/position           \ /tsb/surge, /tsb/heading, /tsb/yaw_rate
   v                          v
los_node -----------------> pid_node
         /tsb/reference          |
                                 | /cmd_vel
                                 v
                         Gazebo / TurtleBot3
```

### ROS 2 Topics

| Topic | Message Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | Robot odometry: position and velocity |
| `/imu` | `sensor_msgs/msg/Imu` | Robot orientation and angular velocity |
| `/tsb/position` | `std_msgs/msg/Float64MultiArray` | Robot position `[x, y]` (m) |
| `/tsb/waypoints` | `std_msgs/msg/Float64MultiArray` | Replacement path `[x0, y0, x1, y1, ...]` (m) |
| `/tsb/surge` | `std_msgs/msg/Float64` | Measured surge velocity (m/s) |
| `/tsb/heading` | `std_msgs/msg/Float64` | Measured heading (rad) |
| `/tsb/yaw_rate` | `std_msgs/msg/Float64` | Measured yaw rate (rad/s) |
| `/tsb/reference` | `std_msgs/msg/Float64MultiArray` | Desired `[surge velocity (m/s), heading (rad)]` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Linear (`linear.x`) and angular (`angular.z`) commands |

## Requirements

The project was developed and tested using:

- Ubuntu 22.04 LTS (WSL 2)
- ROS 2 Humble Hawksbill
- C++17
- Gazebo Classic
- TurtleBot3 Burger
- Colcon build system

The ROS 2 dependencies include `rclcpp`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `launch`, `launch_ros` and `ament_index_python`.

TurtleBot3 simulation packages must also be installed, including `turtlebot3_gazebo`.

## Installation and Build

The `tsb_control` package must be located inside the `src` directory of a ROS 2 workspace (the examples below use `~/tsb_ws`).

```bash
source /opt/ros/humble/setup.bash
cd ~/tsb_ws
colcon build --packages-select tsb_control --symlink-install
source install/setup.bash
```

Verify that the executables are installed:

```bash
ros2 pkg executables tsb_control
```

The expected executables are `sensor_processing_node`, `pid_node` and `los_node`.

## Running the System

**The control launch file does not start Gazebo or spawn the TurtleBot3.** Run the simulator separately.

### 1. Start Gazebo and TurtleBot3

In terminal 1:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Wait until the TurtleBot3 appears in Gazebo.

### 2. Start the control and guidance nodes

In terminal 2:

```bash
source ~/tsb_ws/install/setup.bash
ros2 launch tsb_control control.launch.py
```

`control.launch.py` launches:

- `sensor_processing_node`
- `pid_node`, with `config/pid_params.yaml`
- `los_node`, with initial waypoints loaded from `config/los_params.yaml`

**No manual reference command is needed during the loiter test:** the LOS node continuously publishes `/tsb/reference`.

### 3. Monitor the system

In terminal 3:

```bash
source ~/tsb_ws/install/setup.bash
ros2 node list
ros2 topic list
ros2 topic echo /tsb/position
```

Other useful diagnostics, run separately:

```bash
ros2 topic echo /tsb/reference
ros2 topic echo /cmd_vel
```

The `/tsb/reference` message contains two values:

- `data[0]`: desired surge velocity (m/s)
- `data[1]`: desired heading (rad)

The `/cmd_vel` output uses `linear.x` for linear velocity and `angular.z` for angular velocity.

### 4. Stop the simulation

Press `Ctrl+C` in the terminals running the control nodes and Gazebo.

## Parameters and Configuration

### PID Parameters (YAML)

The PID controller parameters are defined in `config/pid_params.yaml` and loaded by `control.launch.py` when the PID starts.

The following are the **defaults declared in `pid_node.cpp`**; values supplied by the YAML file take precedence.

| Parameter | Source-code default | Description |
|---|---:|---|
| `kp_surge` | 1.0 | Surge proportional gain |
| `ki_surge` | 0.2 | Surge integral gain |
| `kd_surge` | 0.05 | Surge derivative gain |
| `kp_heading` | 1.0 | Heading proportional gain |
| `ki_heading` | 0.1 | Heading integral gain |
| `kd_heading` | 0.2 | Heading derivative gain |
| `surge_integral_limit` | 1.0 | Surge integral accumulator limit |
| `heading_integral_limit` | 1.0 | Heading integral accumulator limit |
| `max_surge_cmd` | 0.15 m/s | Maximum absolute commanded linear speed |
| `max_yaw_rate_cmd` | 1.0 rad/s | Maximum absolute commanded angular velocity |

To modify PID parameters, edit `config/pid_params.yaml`, save the file and restart the control launch. For example, parameters can be inspected using:

```bash
ros2 param get /pid_node kp_surge
ros2 param list /pid_node
```

### LOS Settings and Waypoints

The lookahead, desired speed, and waypoint tolerance are **fixed in `src/los_node.cpp`**. The initial waypoint vector is a ROS 2 parameter, loaded from `config/los_params.yaml` by `control.launch.py` when the node starts:

| Setting | Value | Description |
|---|---:|---|
| `lookahead_` | 0.70 m | LOS lookahead distance |
| `desired_speed_` | 0.10 m/s | Desired surge speed |
| `waypoint_tolerance_` | 0.20 m | Waypoint acceptance radius |
| Update period | 100 ms | Guidance update period (10 Hz) |
| Position timeout | 1 s | Maximum allowed age of position data |

The eight waypoints, defined in the odometry coordinate frame, are:

| Waypoint | x (m) | y (m) |
|---|---:|---:|
| P1 | 0.000 | 0.000 |
| P2 | 0.439 | -1.061 |
| P3 | 1.500 | -1.500 |
| P4 | 2.561 | -1.061 |
| P5 | 3.000 | 0.000 |
| P6 | 2.561 | 1.061 |
| P7 | 1.500 | 1.500 |
| P8 | 0.439 | 1.061 |

They approximate a circle of radius **1.5 m**, centred at **(1.5, 0.0) m**. The path proceeds from P1 to P2 and so on, then from P8 to P1, continuously. The sequence is counter-clockwise as viewed from above in the standard x-right, y-up coordinate frame.

To change the **initial route**, edit `config/los_params.yaml` (flattened `[x0, y0, x1, y1, ...]` coordinates), then restart the control launch. With `--symlink-install`, this usually does not require recompiling; rebuild if the installed configuration is not updated. The source code provides the original eight points as a fallback when starting the LOS without a YAML override.

To replace the route **while the nodes are running**, publish at least three `[x, y]` pairs with finite coordinates and no consecutive repeated points. For example, send a square:

```bash
ros2 topic pub --once /tsb/waypoints std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 2.0, 0.0, 2.0, 2.0, 0.0, 2.0]}"
```

This replaces the active route immediately and resets guidance to segment P1 → P2. The final waypoint connects to the first for continuous loiter. Coordinates use the same odometry reference frame; changing routes while moving may produce a sharp turn. To recover the original YAML path, restart the launch (or publish the original eight points). The `waypoints` ROS parameter is **read only at startup** by this implementation; for live changes use `/tsb/waypoints`, not `ros2 param set`.

## Testing and Validation

Tests were performed with TurtleBot3 Burger in Gazebo Classic using the Empty World environment, to avoid obstacles during controller and guidance evaluation.

### 1. Node Communication and Sensor Processing

The nodes were started together using `control.launch.py`. Sensor processing publishes surge, heading, yaw rate and the added planar position topic. Communication can be checked using:

```bash
ros2 node list
ros2 topic echo /tsb/surge
ros2 topic echo /tsb/heading
ros2 topic echo /tsb/yaw_rate
ros2 topic echo /tsb/position
```

The position publisher was also tested independently by sending a known `/odom` position and checking that `/tsb/position` reproduced its `[x, y]` values.

### 2. PID Control

The PID controller computes commands at 10 Hz. The implementation includes surge reference feedforward, wrapped heading error, derivative damping based on measured yaw rate, integral clamping and output saturation.

For a **manual PID reference test**, the LOS node must not be running, since it otherwise also publishes to `/tsb/reference`. With the LOS stopped and the sensor and PID nodes running, an example command is:

```bash
ros2 topic pub --once /tsb/reference std_msgs/msg/Float64MultiArray "{data: [0.10, 0.0]}"
```

### 3. LOS Guidance and Continuous Loiter

The previous hard-coded loiter version was compiled and initially tested by publishing a simulated position to `/tsb/position` and observing its output in `/tsb/reference`.

In the integrated Gazebo test, the TurtleBot3 was observed following an **approximately octagonal path and repeating the circuit continuously**. This demonstrates end-to-end communication from sensor processing through LOS guidance and PID control to `/cmd_vel`.

The observed repeated motion is a functional test of the **earlier hard-coded loiter version**. After adding YAML-based waypoints and the new waypoint topic, repeat the Gazebo loiter test, verify the initial parameter using `ros2 param get /los_node waypoints`, and publish an alternate trajectory on `/tsb/waypoints` to validate runtime switching. These additional checks must be performed before claiming the new version was tested. Quantitative path-following accuracy (for example, RMS cross-track error) has not been measured.

### 4. Safety Mechanisms

**Input validation:** The PID rejects non-finite sensor values or reference values, and expects exactly two reference values. The LOS node rejects invalid position messages and malformed waypoint vectors (fewer than three pairs, odd-length vectors, non-finite coordinates, or consecutive duplicate points).

**Sensor timeout:** If the PID does not receive updated surge, heading or yaw-rate data for over one second, it publishes a zero `Twist` and suspends normal control.

**LOS position timeout:** If the LOS loses recent position data, it publishes zero desired surge speed and retains the last desired heading. This does not necessarily imply a completely zero `/cmd_vel`, since the PID may still command an angular correction.

**Command saturation:** The PID limits the absolute commanded linear speed and angular velocity according to its configured limits.

**Anti-windup:** The surge and heading integral accumulators are clamped to configured bounds.

### 5. PID Parameter Configuration

The ROS 2 parameter interface was checked using:

```bash
ros2 param list /pid_node
ros2 param get /pid_node kp_surge
```

YAML parameter loading was previously tested by changing a gain, restarting the control nodes, and restoring the original configuration.

## Known Limitations

- **Surge oscillation:** Oscillatory behaviour was observed during earlier surge controller tests. The PID gains have not been fine-tuned further.
- **Reference timeout:** The PID does not check the age of `/tsb/reference`. If the LOS node stops while sensor data continues, the last valid reference can remain active. This is distinct from the PID's sensor timeout.
- **Polygonal loiter:** The default guidance follows eight straight segments, not a mathematically exact circle. New paths follow the supplied straight segments; tracking errors can round or distort corners.
- **LOS configuration:** The lookahead, speed and waypoint tolerance remain fixed in C++. Initial waypoints are configured via YAML; a replacement trajectory can be received at runtime via `/tsb/waypoints`.
- **Coordinate frames:** The LOS assumes the position and heading used for guidance are consistent with the odometry reference frame. Frame alignment has not been independently verified.
- **Quantitative accuracy:** The repeated loiter was observed visually in Gazebo; no quantitative path-error metrics or controller tuning study have been included.
