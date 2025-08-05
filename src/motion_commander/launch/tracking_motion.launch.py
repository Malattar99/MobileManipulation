# tracking_generator.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    urdf_path = PathJoinSubstitution([
        FindPackageShare("robot_model"),
        "urdf",
        "ur5.urdf.xacro"
    ])
    
    # Path to the parameters YAML file
    params_file = PathJoinSubstitution([
        FindPackageShare("motion_commander"),  # Replace with your package name
        "config",
        "tracking_motion_params.yaml"
    ])
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    FindExecutable(name='xacro'),
                    ' ',
                    urdf_path
                ])
            }]
        ),
        Node(
            package='motion_commander',
            executable='tracking_generator_node',
            name='motion_generator_node',
            output='screen',
            parameters=[
                {
                    'robot_description': Command([
                        FindExecutable(name='xacro'),
                        ' ',
                        urdf_path
                    ])
                },
                params_file  # Load parameters from YAML file
            ]
        )
    ])