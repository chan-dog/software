import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    urdf_path = LaunchConfiguration('urdf_path', default=os.path.join(get_package_share_directory('bringup'), 'urdf', 'robot_base.urdf'))

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_path],
        output='screen'
    )
    
    rplidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_c1_launch.py')),
    )

    chassis_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('chassis'), 'launch', 'chassis_launch.py')),
    )

    # imu_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('wit_ros2_imu'), 'launch', 'rviz_and_imu.launch.py')), 
    # )

    ld = LaunchDescription()

    ld.add_action(robot_state_publisher_node)
    ld.add_action(rplidar_node)
    ld.add_action(chassis_node)
    # ld.add_action(imu_node)

    return ld