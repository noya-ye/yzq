from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="ground_station_bridge_pkg",
            executable="udp_ground_station_bridge",
            name="udp_ground_station_bridge",
            output="screen",
            parameters=[{
                "bind_ip": "0.0.0.0",
                "bind_port": 9000,

                # 修改成树莓派地面站IP
                "remote_ip": "192.168.31.187",
                "remote_port": 9001,

                "print_rx": True,
                "print_tx": True,
            }],
        )
    ])