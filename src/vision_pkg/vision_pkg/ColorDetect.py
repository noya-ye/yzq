import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge

import numpy as np
import cv2


# ============================================================
# ROI：地图区域提取
# ============================================================
def get_map_roi(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = np.array([0, 0, 160])
    upper = np.array([180, 40, 255])

    mask = cv2.inRange(hsv, lower, upper)

    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    roi_mask = np.zeros_like(mask)

    if len(contours) > 0:
        cnt = max(contours, key=cv2.contourArea)
        hull = cv2.convexHull(cnt)
        cv2.drawContours(roi_mask, [hull], -1, 255, -1)

    return roi_mask


# ============================================================
# 主节点
# ============================================================
class ColorDetectNode(Node):

    def __init__(self):
        super().__init__('color_detect_node')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/d435/color',
            self.image_callback,
            10
        )

        self.pub = self.create_publisher(
            PointStamped,
            '/target_point',
            10
        )

        # self.debug_pub = self.create_publisher(
        #     Image,
        #     '/vision/front/debug_image',
        #     10
        # )

        # ========================================================
        # 颜色范围（你第二段代码版本）
        # ========================================================
        self.color_ranges = {
            "red": [
                ([0, 100, 80], [10, 255, 255]),
                ([160, 100, 80], [180, 255, 255])
            ],
            "yellow": [
                ([20, 100, 80], [35, 255, 255])
            ],
            "green": [
                ([40, 50, 80], [85, 255, 255])
            ],
            "blue": [
                ([100, 100, 80], [130, 255, 255])
            ]
        }

        self.get_logger().info("Multi-target ColorDetectNode started")

    # ============================================================
    # callback
    # ============================================================
    def image_callback(self, msg):

        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = cv2.convertScaleAbs(img, alpha=1.5, beta=20)

        debug = img.copy()

        h, w = img.shape[:2]

        # ================= ROI =================
        roi_mask = get_map_roi(img)
        img_roi = cv2.bitwise_and(img, img, mask=roi_mask)

        hsv = cv2.cvtColor(img_roi, cv2.COLOR_BGR2HSV)

        target_count = 0

        # ========================================================
        # 多目标核心
        # ========================================================
        for color_name, ranges in self.color_ranges.items():

            mask_total = None

            for lower, upper in ranges:
                lower = np.array(lower, dtype=np.uint8)
                upper = np.array(upper, dtype=np.uint8)

                mask = cv2.inRange(hsv, lower, upper)
                mask_total = mask if mask_total is None else cv2.bitwise_or(mask_total, mask)

            # 去噪（你第二段保留思路）
            kernel = np.ones((5, 5), np.uint8)
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_OPEN, kernel)
            mask_total = cv2.morphologyEx(mask_total, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask_total,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            for cnt in contours:

                cnt = cv2.convexHull(cnt)
                area = cv2.contourArea(cnt)

                # ======================
                # 面积过滤（核心参数）
                # ======================
                if area < 50:
                    continue

                M = cv2.moments(cnt)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])

                # ======================
                # 发布点
                # ======================
                msg_out = PointStamped()
                msg_out.header = msg.header
                msg_out.point.x = float(cx)
                msg_out.point.y = float(cy)
                msg_out.point.z = 0.0

                self.pub.publish(msg_out)

                # ======================
                # debug
                # ======================
                # color_map = {
                #     "red": (0, 0, 255),
                #     "green": (0, 255, 0),
                #     "yellow": (0, 255, 255),
                #     "blue": (255, 0, 0),
                # }

                # draw_color = color_map.get(color_name, (255, 255, 255))

                # cv2.drawContours(debug, [cnt], -1, draw_color, 2)
                # cv2.circle(debug, (cx, cy), 5, draw_color, -1)

                # cv2.line(
                #     debug,
                #     (int(w * 0.5), int(h * 0.5)),
                #     (cx, cy),
                #     draw_color,
                #     2
                # )

                # cv2.putText(
                #     debug,
                #     f"{color_name} {int(area)}",
                #     (cx, cy),
                #     cv2.FONT_HERSHEY_SIMPLEX,
                #     0.5,
                #     draw_color,
                #     2
                # )

                target_count += 1

        # ======================
        # debug publish
        # ======================
        # cv2.putText(
        #     debug,
        #     f"targets: {target_count}",
        #     (10, 30),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     0.8,
        #     (0, 255, 0),
        #     2
        # )

        # dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        # dbg_msg.header = msg.header
        # self.debug_pub.publish(dbg_msg)


# ============================================================
def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()