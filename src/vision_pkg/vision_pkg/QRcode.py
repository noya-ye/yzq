import cv2
from pyzbar.pyzbar import decode

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from custom_vision_msgs.msg import ServoTarget
from std_msgs.msg import Bool

class QRcode(Node):
    def __init__(self):
        super().__init__('qrcode_node')

        self.bridge = CvBridge()
        self.enabled = False

        self.sub = self.create_subscription(
            Image,
            '/camera/down',
            self.image_callback,
            10
        )
        self.pub = self.create_publisher(
            ServoTarget,
            '/qrcode_point',
            10
        )
        self.enable_sub = self.create_subscription(
            Bool,
            '/vision/down/enable',
            self.enable_callback,
            10
        )
        self.debug_pub = self.create_publisher(
            Image,
            '/vision/QRcode/debug_image',
            10
        )

    def enable_callback(self, msg):
        self.enabled = bool(msg.data)

    def draw_center_cross(self, img):
        h, w = img.shape[:2]
        cx = w // 2
        cy = h // 2
        cv2.line(img, (cx - 20, cy), (cx + 20, cy), (0, 255, 0), 2)
        cv2.line(img, (cx, cy - 20), (cx, cy + 20), (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 4, (0, 255, 0), -1)

    def draw_text_bg(self, img, text, org, color=(255, 255, 255)):
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 2
        (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
        x, y = org
        cv2.rectangle(img,
                      (x - 3, y - th - 5),
                      (x + tw + 3, y + baseline + 3),
                      (0, 0, 0),
                      -1)
        cv2.putText(img, text, org, font, scale, color, thickness, cv2.LINE_AA)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )
        debug = frame.copy()
        h, w = frame.shape[:2]
        frame_cx = w // 2
        frame_cy = h // 2
        self.draw_center_cross(debug)

        if not self.enabled:
            self.draw_text_bg(debug, "ENABLE: false", (10, 30), (0, 0, 255))
            debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
            return

        found_target = False
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        qr_results = decode(gray)

        for qr_result in qr_results:
            found_target = True
            x1 = int(qr_result.rect.left)
            y1 = int(qr_result.rect.top)
            x2 = int(qr_result.rect.left + qr_result.rect.width)
            y2 = int(qr_result.rect.top + qr_result.rect.height)
            bbox_cx = (x1 + x2) // 2
            bbox_cy = (y1 + y2) // 2
            dx = float(bbox_cx - frame_cx)
            dy = float(bbox_cy - frame_cy)

            cv2.rectangle(debug, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(debug, (bbox_cx, bbox_cy), 4, (0, 255, 255), -1)
            self.draw_text_bg(debug, f"dx={dx:.1f} dy={dy:.1f}", (x1, y1 - 10))

            typex = -1
            text = qr_result.data.decode('utf-8').strip()
            if text.isdigit():
                num = int(text)
                if 1 <= num <= 24:
                    typex = num

            label = f"QR={typex if typex != -1 else 'NA'}"
            self.draw_text_bg(debug, label, (x1, y2 + 20))

            score = float(x2 - x1) * float(y2 - y1)
            self.get_logger().info(
                f"QR={typex}, dx={dx:.1f}, dy={dy:.1f}, score={score:.2f}"
            )

            t = ServoTarget()
            t.x = float(bbox_cx)
            t.y = float(bbox_cy)
            t.dx = dx
            t.dy = dy
            t.type = typex
            t.score = score
            self.pub.publish(t)

        if not found_target:
            self.draw_text_bg(debug, "no QR target", (10, 30), (0, 0, 255))

        debug_msg = self.bridge.cv2_to_imgmsg(debug, 'bgr8')
        debug_msg.header = msg.header
        self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = QRcode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()