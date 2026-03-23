from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        Node(
            package='offboard_control_ros2',
            executable='offboard_control_node',
            name='offboard_control_node',
            output='screen',
            parameters=['config/offboard_params.yaml'],
        )
    ])
