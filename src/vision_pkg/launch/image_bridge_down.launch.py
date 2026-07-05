from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration("world").perform(context)
    model = LaunchConfiguration("model").perform(context)
    link = LaunchConfiguration("link").perform(context)
    sensor = LaunchConfiguration("sensor").perform(context)

    gz_image_topic = f"/world/{world}/model/{model}/link/{link}/sensor/{sensor}/image"
    gz_info_topic = f"/world/{world}/model/{model}/link/{link}/sensor/{sensor}/camera_info"

    ros_image_topic = "/camera/image_raw"
    ros_info_topic = "/camera/camera_info"

    image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="x500_mono_cam_down_image_bridge",
        output="screen",
        arguments=[
            gz_image_topic,
        ],
        remappings=[
            (gz_image_topic, ros_image_topic),
        ],
    )

    camera_info_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="x500_mono_cam_down_camera_info_bridge",
        output="screen",
        arguments=[
            f"{gz_info_topic}@gz.msgs.CameraInfo@sensor_msgs/msg/CameraInfo",
        ],
        remappings=[
            (gz_info_topic, ros_info_topic),
        ],
    )

    return [
        image_bridge,
        camera_info_bridge,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value="target_map_auto",
            description="Gazebo world name, must match /world/<name>/..."
        ),
        DeclareLaunchArgument(
            "model",
            default_value="x500_mono_cam_down_0",
            description="Gazebo model name"
        ),
        DeclareLaunchArgument(
            "link",
            default_value="camera_link",
            description="Gazebo camera link name"
        ),
        DeclareLaunchArgument(
            "sensor",
            default_value="imager",
            description="Gazebo camera sensor name"
        ),
        OpaqueFunction(function=launch_setup),
    ])
