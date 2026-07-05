from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_debug_recorder',
            executable='debug_recorder_node',
            name='debug_recorder_node',
            output='screen',
            parameters=[{
                'sample_rate_hz': 10.0,
                'max_rows': 200000,
                'log_root_dir': '/home/yzq/debug_logs',
                'vehicle_status_topic': '/fmu/out/vehicle_status_v1',
                'local_position_topic': '/fmu/out/vehicle_local_position',
                'vehicle_attitude_topic': '/fmu/out/vehicle_attitude',
                'land_detected_topic': '/fmu/out/vehicle_land_detected',
                'offboard_mode_topic': '/fmu/in/offboard_control_mode',
                'trajectory_setpoint_topic': '/fmu/in/trajectory_setpoint',
            }]
        )
    ])
