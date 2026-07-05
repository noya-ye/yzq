from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='vision_pkg',
            executable='DownCamera_Node',
            name='DownCamera_Node',
            output='screen'
        ),


        Node(
            package='vision_pkg',
            executable='ShapeColorDetect_Node',
            name='ShapeColorDetect_Node',
            output='screen'
        ),
        
    ])