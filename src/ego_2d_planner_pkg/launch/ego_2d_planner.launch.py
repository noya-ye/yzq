from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('ego_2d_planner_pkg')

    default_config = os.path.join(
        pkg_share,
        'config',
        'ego_2d_planner.yaml'
    )

    default_rviz_config = os.path.join(
        pkg_share,
        'rviz',
        'ego_2d_planner.rviz'
    )

    config = LaunchConfiguration('config')
    use_rviz = LaunchConfiguration('use_rviz')
    use_traj_server = LaunchConfiguration('use_traj_server')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to ego_2d_planner parameter file'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to start RViz2'
        ),

        DeclareLaunchArgument(
            'use_traj_server',
            default_value='true',
            description='Whether to start 2D traj server that converts selected_path to /position_cmd'
        ),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Path to RViz2 config file'
        ),

        Node(
            package='ego_2d_planner_pkg',
            executable='ego_replan_fsm_2d_node',
            name='ego_replan_fsm_2d_node',
            output='screen',
            parameters=[config]
        ),

        Node(
            condition=IfCondition(use_traj_server),
            package='ego_2d_planner_pkg',
            executable='traj_server_2d_node',
            name='traj_server_2d_node',
            output='screen',
            parameters=[config]
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])