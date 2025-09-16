import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('motor'), 'config', 'param.yaml')

    motor_node = Node(
        package='motor',
        executable='motor_node',
        namespace='',
        name='motor_node',
        output='screen',
        emulate_tty=True,
        parameters=[config],
        remappings=[
            ('/cmd_vel/head', '/cmd_vel_head'),
        ]
    )

    return LaunchDescription([motor_node])
