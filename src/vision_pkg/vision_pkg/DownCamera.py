import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class CamPub(Node):
    def __init__(self):
        super().__init__('DownCamera_Node')

        self.cap = cv2.VideoCapture(0)   # 👉 设备1
        self.bridge = CvBridge()

        self.pub = self.create_publisher(Image, '/camera/down', 10)

        self.timer = self.create_timer(0.03, self.loop)  # ~30Hz

    def loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = CamPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()