from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fastlio_to_px4',
            executable='fastlio_to_px4_node',
            name='fastlio_to_px4_node',
            output='screen',
            parameters=[
                {
                    'odom_topic': '/fastlio2/lio_odom',
                    'px4_topic': '/fmu/in/vehicle_visual_odometry',
                    'use_odom_twist': True,
                    'zero_position_on_start': False,

                    'position_variance_x': 0.05,
                    'position_variance_y': 0.05,
                    'position_variance_z': 0.08,

                    'orientation_variance_roll': 0.02,
                    'orientation_variance_pitch': 0.02,
                    'orientation_variance_yaw': 0.05,

                    'velocity_variance_x': 0.05,
                    'velocity_variance_y': 0.05,
                    'velocity_variance_z': 0.08,
                }
            ]
        )
    ])