from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='vision_pkg',
            executable='video_record_node',
            name='video_record_node',
            output='screen'
        ),

        
    ])