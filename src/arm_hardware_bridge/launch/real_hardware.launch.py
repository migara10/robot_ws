import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    moveit_pkg = get_package_share_directory('arm_moveit_config')

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'rsp.launch.py')
        )
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'move_group.launch.py')
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, 'launch', 'moveit_rviz.launch.py')
        )
    )

    hardware_bridge_node = Node(
        package='arm_hardware_bridge',
        executable='hardware_bridge_node',
        output='screen',
    )

    return LaunchDescription([
        rsp_launch,
        move_group_launch,
        rviz_launch,
        hardware_bridge_node,
    ])