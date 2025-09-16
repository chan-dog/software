import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('imu'), 'config', 'param.yaml')

    imu_node = Node(
        package='imu',
        executable='imu_node',
        namespace='',
        name='imu_node',
        output='screen',
        emulate_tty=True,
        parameters=[config],
        remappings=[
            ('/cmd_vel', '/cmd_vel/imu'),
        ]
    )

    return LaunchDescription([imu_node])