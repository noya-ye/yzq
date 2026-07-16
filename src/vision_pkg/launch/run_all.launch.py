from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='vision_pkg',
            executable='shape_color_down_node',
            name='shape_color_down_node',
            output='screen'
        ),

    ])