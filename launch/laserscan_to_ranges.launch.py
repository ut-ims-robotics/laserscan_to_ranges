from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laserscan_to_ranges',
            executable='laserscan_to_ranges',
            name='laserscan_to_ranges'
        )
    ])