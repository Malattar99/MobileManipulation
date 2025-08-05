# Motion Generators

ROS2 C++ nodes for generating robot trajectories with inverse kinematics support.

## Overview

This package contains two motion generation nodes that demonstrate different trajectory planning approaches for robots with mobile bases and manipulator arms:

- **motion_generator_node**: Creates point-to-point Cartesian trajectories with trapezoidal velocity profiles
- **tracking_generator_node**: Maintains a fixed TCP (Tool Center Point) position in world coordinates while the mobile base moves

Both nodes use KDL (Kinematics and Dynamics Library) for kinematic calculations and support real-time trajectory generation.


## Usage

### motion_generator_node

Generates smooth trajectories between two predefined poses:

```bash
ros2 launch robot_model task_2a.launch.py
```

The node alternates between two configured poses when in continuous mode, or can be triggered via service call for single movements.
The 2 poses as well as the manipulator's velocity and acceleration can be configured in `linear_motion_params.yaml`

### Tracking Generator

Demonstrates coordinated control by keeping the end-effector at a fixed world position while moving the base in a straight line along the x-axis:

```bash
ros2 launch robot_model task_2b.launch.py
```
The target position as well as the manipulator's and the bas's velocity and acceleration can be configured in `tracking_motion_params.yaml`


## How It Works

### motion_generator_node

1. **Initialization**: Loads robot URDF and creates kinematic chain from base_link to tool0
2. **Trajectory Planning**: 
   - Calculates linear path between start and end poses
   - Generates trapezoidal velocity profile based on distance and velocity limits
   - Samples trajectory at 50Hz for smooth motion
3. **IK Solving**: Converts Cartesian positions to joint angles using Levenberg-Marquardt algorithm
4. **Publishing**: Sends complete trajectory to `/motion_library/arm_trajectory`

### Tracking Generator

1. **Dual Kinematic Chains**: 
   - Full chain (world→tool0) for global calculations
   - Arm chain (base_link→tool0) for relative movements
2. **Two-Phase Operation**:
   - **Phase 1**: Initial approach - arm moves to target while base is stationary
   - **Phase 2**: Coordinated tracking - base moves while arm compensates to maintain world position
3. **Real-time Compensation**: 
   - Calculates required arm configuration at 50Hz
   - Transforms world target to base frame as base moves
   - Publishes synchronized arm and base commands

## Topics

### Published Topics
- `/motion_library/arm_trajectory` (trajectory_msgs/JointTrajectory) - Arm motion commands
- `/motion_library/base_trajectory` (trajectory_msgs/JointTrajectory) - Base motion commands
- `/visualization_marker` (visualization_msgs/Marker) - Target position visualization

### Subscribed Topics
- `/joint_states` (sensor_msgs/JointState) - Current robot joint positions


## Configuration

Both nodes support configuration through ROS2 parameters. Key parameters include:
- Target poses (position and orientation)
- Velocity and acceleration limits
- Motion distances and operating modes
