from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emergency_stop_pkg',
            executable='emergency_stop_node',
            name='emergency_stop_node',
            output='screen',
            emulate_tty=True,
        ),
    ])