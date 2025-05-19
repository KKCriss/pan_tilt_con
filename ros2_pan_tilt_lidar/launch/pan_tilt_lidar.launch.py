from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pan_tilt_lidar',
            executable='node_main',
            name='pan_tilt_lidar_node',
            output='screen',
            parameters=[{'param_name': 'param_value'}],  # Replace with actual parameters if needed
            remappings=[('/old/topic', '/new/topic')]  # Replace with actual topic remappings if needed
        )
    ])