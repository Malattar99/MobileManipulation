from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Launch Gazebo with UR5
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robot_model"),
                "launch",
                "gazebo_ur5.launch.py"
            ])
        ])
    )
    
    # Launch joint publisher after 5 seconds
    joint_publisher_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("joint_commander"),
                        "launch",
                        "task_2.launch.py"
                    ])
                ])
            )
        ]
    )
    # Launch joint publisher after 5 seconds
    motion_commander_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare("motion_commander"),
                        "launch",
                        "linear_motion.launch.py"
                    ])
                ])
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        joint_publisher_launch,
        motion_commander_launch
    ])