from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='vision_pkg',
            executable='d435_node',
            name='d435_node',
            output='screen'
        ),

        Node(
            package='vision_pkg',
            executable='ShapeColorDetect_Node',
            name='ShapeColorDetect_Node',
            output='screen'
        ),

       
        Node(
            package='vision_pkg',
            executable='Positionpub_Node',
            name='Positionpub_Node',
            output='screen'
        ),
        # 🔥 bridge
        Node(
            package='vision_pkg',
            executable='vision_bridge',
            name='vision_bridge',
            output='screen'
        ),

        Node(
            package='vision_pkg',
            executable='DownCamera_Node',
            name='DownCamera_Node',
            output='screen'
        ),


        Node(
            package='vision_pkg',
            executable='ColorDetect_Node',
            name='ColorDetect_Node',
            output='screen'
        ),
    ])