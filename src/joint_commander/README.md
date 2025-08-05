# joint_commander

Python ROS2 nodes for robot trajectory control and coordination.

## Overview

This package provides two complementary nodes for robot trajectory management:

- **task_1**: Generates sinusoidal motion patterns and moves the base in a circular motion
- **publish_joint_angles**: Receives and executes trajectories from external motion planners

## Usage

### Continuous Joint Publisher

Runs pre-programmed motion patterns indefinitely:

```bash
ros2 launch robot_model task_1.launch.py
```

The robot will perform:
- Sinusoidal wave motions on all arm joints
- Circular motion pattern for the mobile base

### External Trajectory Controller

Listens for external trajectory commands and executes them

## How It Works

### task_1

Generates sinusoidal motion without external input:

1. **Motion Patterns**:
   - Arm: Each joint follows a sine wave with configurable amplitude and frequency
   - Base: Moves in a circular pattern with configurable radius
2. **Numerical Stability**: Uses bounded angle calculations to prevent overflow during extended operation
3. **Publishing Rate**: Sends commands at 50Hz for smooth motion

### publish_joint_angles

Acts as an intermediary between motion planners and robot controllers:

1. **Trajectory Reception**: Subscribes to arm and base trajectory topics
2. **Time Management**: 
   - Stores trajectories with absolute timing
   - Tracks playback progress independently for arm and base
3. **Interpolation**: Performs linear interpolation between trajectory points
4. **Mode Switching**: Automatically returns to idle when trajectories complete

## Topics

### task_1
**Published Topics:**
- `/joint_trajectory_controller/joint_trajectory` - Arm joint commands
- `/base_trajectory_controller/joint_trajectory` - Base position commands

### publish_joint_angles
**Published Topics:**
- `/joint_trajectory_controller/joint_trajectory` - Arm joint commands  
- `/base_trajectory_controller/joint_trajectory` - Base position commands

**Subscribed Topics:**
- `/motion_library/arm_trajectory` - External arm trajectory input
- `/motion_library/base_trajectory` - External base trajectory input

## Message Format

Both nodes use `trajectory_msgs/JointTrajectory` messages:
- Arm trajectories: 6 joints (shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3)
- Base trajectories: 3 DOF (translate_x, translate_y, rotate_z)

## Configuration

### task_1
Motion parameters are currently hardcoded but can be modified:
- Arm amplitude, frequency, and phase offsets
- Base radius and angular velocity
- Publishing rate

### External Trajectory Controller  
- Configurable trajectory completion buffer time
- Independent progress tracking for arm and base
