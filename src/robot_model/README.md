# robot_model

This repository contains description files and meshes for the ur5 manipulator. It has been changed so that the robot is on a floating base that can move in the `x`,`y` plane and rotate around the `z` axis.

## Structure of the repository

### urdf
  - `urdf/ur5_macro.xacro` - macro file with the mobile UR5-manipulator description. This file is usually included into external projects to visualize and configure UR manipulators properly. An example how to use this macro is in `urdf/ur5.urdf.xacro` file.
  - `urdf/ur5.ros2_control.xacro` - definition of manipulator's joints and interfaces for `ros2_control` framework.
  - `urdf/ur5.urdf.xacro` - is the main urdf file containing the ros2_control and simulation parameters.

### config 
  - `inital_positions.yaml` - contains the parameters for the robots initial position
  - `ur5_controllers.yaml` - contains the control paramters for ros2_control

## Testing description of a manipulator

To visualize the robot install this repository to you workspace and execute the following:

``` bash
ros2 launch robot_model view_ur5.launch.py
```

To view the robot model in gazebo execute the following

``` bash
ros2 launch robot_model gazebo_ur5.launch.py
```

