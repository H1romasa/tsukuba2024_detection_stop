from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare("realsense2_camera"), '/launch', '/rs_launch.py']),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'color_width': '640',
            'color_height': '480',
            'color_fps': '30'
        }.items()
    )

    emergency_stop_node = Node(
        package='emergency_stop_pkg',
        executable='emergency_stop_node',
        name='emergency_stop_node',
        output='screen',
    )

    return LaunchDescription([
        realsense_launch,
        emergency_stop_node
    ])