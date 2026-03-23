import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params_file = os.path.join(
        get_package_share_directory('offboard_control_ros2'),
        'config',
        'offboard_params.yaml',
    )

    return LaunchDescription([
        Node(
            package='offboard_control_ros2',
            executable='offboard_control_node',
            name='offboard_control_node',
            output='screen',
            parameters=[params_file],
        )
    ])
