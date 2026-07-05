import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


# 仿真桥接节点：将仿真环境中的图像数据从 /camera/image_raw 话题重新发布到 /camera/image_view 话题，以便在 RViz 中可视化。

class ImageRepubNode(Node):
    def __init__(self):
        super().__init__('image_repub_node')

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.pub_ = self.create_publisher(Image, '/camera/image_view', pub_qos)
        self.sub_ = self.create_subscription(Image, '/camera/image_raw', self.cb, sub_qos)

        self.get_logger().info('Republishing /camera/image_raw -> /camera/image_view')

    def cb(self, msg):
        self.pub_.publish(msg)

def main():
    rclpy.init()
    node = ImageRepubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
