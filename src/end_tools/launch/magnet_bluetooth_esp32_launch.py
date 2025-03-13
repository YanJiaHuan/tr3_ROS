from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='end_tools',
            executable='magnet_bluetooth_node',  # match the actual console script name
            name='magnet_bluetooth_node',
            output='screen',
            parameters=[
                {'rfcomm_port': '/dev/rfcomm0'},
                {'baud_rate': 115200}
            ]
        )
    ])
