from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    bringup_pkg_share = get_package_share_directory('bringup_pkg')

    full_system_launch = os.path.join(
        bringup_pkg_share,
        'launch',
        'full_system.launch.py'
    )

    full_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(full_system_launch)
    )

    offboard_node = Node(
        package='offboard_core_pkg',
        executable='offboard_test1_node',
        name='offboard_test1_node',
        output='screen'
    )

    delayed_offboard = TimerAction(
        period=5.0,
        actions=[offboard_node]
    )

    return LaunchDescription([
        full_system,
        delayed_offboard
    ])