import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('chassis'), 'config', 'param.yaml')

    chassis_node = Node(
        package='chassis',
        executable='chassis_node',
        namespace='',
        name='chassis_node',
        output='screen',
        emulate_tty=True,
        parameters=[config],
        remappings=[
            ('/cmd_vel', '/cmd_vel/chassis'),
        ]
    )

    return LaunchDescription([chassis_node])