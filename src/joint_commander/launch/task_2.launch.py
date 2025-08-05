from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joint_commander',
            executable='publish_joint_angles', 
            name='publish_joint_angles',
            output='screen',
            parameters=[],
        ),
    ])
