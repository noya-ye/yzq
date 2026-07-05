from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ground_station_bridge_pkg",
            executable="ground_station_bridge_node",
            name="ground_station_bridge_node",
            output="screen",
            parameters=[
                {
                    "enable_serial": False,
                    "serial_device": "/dev/ttyUSB0",
                    "baudrate": 115200,
                    "print_tx": True,

                    "local_pos_topic": "/fmu/out/vehicle_local_position",
                    "attitude_topic": "/fmu/out/vehicle_attitude",
                    "vehicle_status_topic": "/fmu/out/vehicle_status_v1",

                    "task_status_topic": "/ground_station/task_status",
                    "raw_cmd_topic": "/ground_station/cmd",
                    "parsed_cmd_topic": "/ground_station/cmd_parsed",
                    "telemetry_topic": "/ground_station/telemetry",
                }
            ],
        )
    ])
