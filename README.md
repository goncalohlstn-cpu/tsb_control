
# TSB Control — Autonomous Systems Recruitment

## Project Overview

This project was developed for the second phase of the
Técnico Solar Boat Autonomous Systems recruitment.

The objective is to implement a ROS 2 control system
for a TurtleBot3 Burger robot using PID controllers
for surge velocity and heading.

The system includes:
- Sensor processing using odometry and IMU data.
- PID control of surge velocity and heading.
- Anti-windup and command saturation.
- Sensor timeout and input validation.
- ROS 2 launch files and YAML parameter configuration.

## Architecture

The control system consists of two ROS 2 nodes:

### Sensor Processing Node

The `sensor_processing_node` subscribes to odometry
and IMU data from the TurtleBot3 simulation.

It extracts and publishes:
- Surge velocity from odometry.
- Heading angle from the IMU quaternion.
- Yaw rate from the IMU angular velocity.

### PID Node

The `pid_node` receives the processed sensor data
and the desired surge velocity and heading.

It implements two PID controllers:
- Surge velocity control.
- Heading control.

The node publishes linear and angular velocity
commands to `/cmd_vel`.

### ROS 2 Topics

| Topic | Message Type | Description |
|---|---|---|
| /odom | nav_msgs/msg/Odometry | Robot odometry |
| /imu | sensor_msgs/msg/Imu | Robot IMU data |
| /tsb/surge | std_msgs/msg/Float64 | Measured surge velocity (m/s) |
| /tsb/heading | std_msgs/msg/Float64 | Measured heading (rad) |
| /tsb/yaw_rate | std_msgs/msg/Float64 | Measured yaw rate (rad/s) |
| /tsb/reference | std_msgs/msg/Float64MultiArray | Desired surge and heading |
| /cmd_vel | geometry_msgs/msg/Twist | Linear and angular velocity commands |

## Requirements

The project was developed and tested using:

- Ubuntu 22.04 LTS (WSL 2)
- ROS 2 Humble Hawksbill
- C++17
- Gazebo Classic
- TurtleBot3 Burger
- Colcon build system

The following ROS 2 packages are required:

- rclcpp
- std_msgs
- geometry_msgs
- nav_msgs
- sensor_msgs
- launch
- launch_ros
- ament_index_python

TurtleBot3 simulation packages must also be installed.

## Installation and Build

### 1. Source ROS 2

```bash
source /opt/ros/humble/setup.bash
```

### 2. Navigate to the workspace

```bash
cd ~/tsb_ws
```

The `tsb_control` package must be located inside
the workspace's `src` directory.

### 3. Build the package

```bash
colcon build --packages-select tsb_control --symlink-install
```

### 4. Source the workspace

```bash
source install/setup.bash
```

### 5. Verify the installation

```bash
ros2 pkg list | grep tsb_control
```

The package should appear in the output.

## Running the System

The system can be tested using the TurtleBot3 Burger
in Gazebo.

### 1. Start the Gazebo simulation

Open a terminal and execute:

```bash
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Wait until the TurtleBot3 is spawned in Gazebo.

### 2. Launch the control nodes

Open a second terminal and execute:

```bash
source ~/tsb_ws/install/setup.bash
ros2 launch tsb_control control.launch.py
```

This launch file starts:
- sensor_processing_node
- pid_node

It also loads the PID parameters from
`config/pid_params.yaml`.

### 3. Publish a reference

Open a third terminal and execute:

```bash
source ~/tsb_ws/install/setup.bash
ros2 topic pub --once /tsb/reference std_msgs/msg/Float64MultiArray "{data: [0.10, 0.0]}"
```

The reference vector contains:

- data[0]: Desired surge velocity (m/s).
- data[1]: Desired heading (rad).

In this example, the desired surge velocity is
0.10 m/s and the desired heading is 0 rad.

### 4. Monitor the control commands

To inspect the commands published by the PID node:

```bash
ros2 topic echo /cmd_vel
```

The topic contains:
- linear.x: Linear velocity command (m/s).
- angular.z: Angular velocity command (rad/s).

### 5. Stop the simulation

Press Ctrl+C in the terminals running the
control nodes and the Gazebo simulation.

## Parameters and Configuration

The PID controller parameters are defined in:

`config/pid_params.yaml`

The configuration file is automatically loaded by
`control.launch.py` when the PID node starts.

### PID Gains

| Parameter | Default Value | Description |
|---|---|---|
| kp_surge | 1.0 | Surge proportional gain |
| ki_surge | 0.2 | Surge integral gain |
| kd_surge | 0.05 | Surge derivative gain |
| kp_heading | 1.0 | Heading proportional gain |
| ki_heading | 0.1 | Heading integral gain |
| kd_heading | 0.2 | Heading derivative gain |

### Anti-Windup and Saturation

| Parameter | Default Value | Description |
|---|---|---|
| surge_integral_limit | 1.0 | Surge integral accumulator limit |
| heading_integral_limit | 1.0 | Heading integral accumulator limit |
| max_surge_cmd | 0.15 | Maximum absolute linear velocity (m/s) |
| max_yaw_rate_cmd | 1.0 | Maximum absolute angular velocity (rad/s) |

### Modifying Parameters

To modify a parameter:

1. Open `config/pid_params.yaml`.
2. Change the desired value.
3. Save the file.
4. Restart the control launch file.

The parameters are loaded when the node starts,
so modifying the YAML does not require recompiling
the C++ source code.

For example, to inspect a parameter:

```bash
ros2 param get /pid_node kp_surge
```

To list all available parameters:

```bash
ros2 param list /pid_node
```

## Testing and Validation

The control system was implemented and tested using
the TurtleBot3 Burger in Gazebo Classic.

The Empty World environment was selected to evaluate
the controllers without interference from obstacles.

### 1. Node Communication

Both ROS 2 nodes were successfully launched using
`control.launch.py`.

The following commands can be used to verify
the system:

```bash
ros2 node list
ros2 topic list
```

Processed sensor data can be monitored using:

```bash
ros2 topic echo /tsb/surge
ros2 topic echo /tsb/heading
ros2 topic echo /tsb/yaw_rate
```

### 2. PID Control

The controllers were tested by publishing desired
surge velocity and heading references.

Example:

```bash
ros2 topic pub --once /tsb/reference std_msgs/msg/Float64MultiArray "{data: [0.10, 0.0]}"
```

The PID node computes the control commands at
a frequency of 10 Hz and publishes them to `/cmd_vel`.

The implementation includes:
- Surge PID control with reference feedforward.
- Heading PID control with angular error wrapping.
- Derivative action using measured yaw rate for heading.
- Integral anti-windup limits.
- Linear and angular command saturation.

### 3. Safety Mechanisms

The PID node includes several safety mechanisms:

**Input Validation**

Non-finite sensor values and reference values
are rejected. The reference vector must contain
exactly two values.

**Sensor Timeout**

If sensor data is not updated for more than
1 second, the controller publishes a zero
velocity command and suspends normal control.

**Command Saturation**

Linear velocity commands are limited to
+/- 0.15 m/s.

Angular velocity commands are limited to
+/- 1.0 rad/s.

**Anti-Windup**

The accumulated integral error is limited
to prevent excessive integral growth.

### 4. Parameter Configuration

The ROS 2 parameters were tested using:

```bash
ros2 param list /pid_node
ros2 param get /pid_node kp_surge
```

YAML parameter loading was verified by
temporarily modifying a gain and restarting
the control nodes.

The original parameter values were restored
after the test.

### 5. Known Limitations

**Surge Oscillation**

Oscillatory behaviour was observed in the
surge velocity controller during simulation.

The PID gains have not been further tuned,
as fine-tuning is not required for this project.

**Reference Timeout**

The current implementation does not include
a reference timeout. The last valid reference
remains active until replaced or the node stops.

