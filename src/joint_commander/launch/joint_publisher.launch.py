from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_commander',
            executable='joint_publisher', 
            name='joint_publisher',
            output='screen',
            parameters=[],
        ),
    ])
