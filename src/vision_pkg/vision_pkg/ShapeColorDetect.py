import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from custom_vision_msgs.msg import ServoTarget, ServoTargetArray


# =========================================================
# ROI 提取
# =========================================================
def get_map_roi(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lower = np.array([0, 0, 140])
    upper = np.array([180, 70, 255])

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


# =========================================================
# ROS Node
# =========================================================
class ShapeColorNode(Node):

    def __init__(self):

        super().__init__('ShapeColorDetect_Node')
    # =====================================================
# 稳定识别滤波
# =====================================================
      
        self.bridge = CvBridge()
        self.enabled = False
        self.image_sub = self.create_subscription(
            Image,
            '/camera/down',
            self.image_callback,
            10
        )

        self.enable_sub = self.create_subscription(
            Bool,
            '/vision/down/enable',
            self.enable_callback,
            10
        )

        self.pub = self.create_publisher(
            ServoTargetArray,
            '/vision/down/servo_targets',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/vision/down/debug_image',
            10
        )

        self.get_logger().info("ShapeColorDetect_Node started")

    # =====================================================
    def enable_callback(self, msg):
        self.enabled = bool(msg.data)

    # =====================================================
    def encode_type(self, color, shape):

        table = {
            ('red', 'circle'): 1,
            ('red', '4'): 2,
            ('red', '3'): 3,
            ('red', '5'): 4,
            ('red', '6'): 5,

            ('green', 'circle'): 6,
            ('green', '4'): 7,
            ('green', '3'): 8,
            ('green', '5'): 9,
            ('green', '6'): 10,

            ('yellow', 'circle'): 11,
            ('yellow', '4'): 12,
            ('yellow', '3'): 13,
            ('yellow', '5'): 14,
            ('yellow', '6'): 15,

            ('blue', 'circle'): 16,
            ('blue', '4'): 17,
            ('blue', '3'): 18,
            ('blue', '5'): 19,
            ('blue', '6'): 20,

            ('irregular', 'irregular'): 21,
        }

        return table.get((color, shape), 0)

    # =====================================================
    def draw_center_cross(self, img):

        h, w = img.shape[:2]
        cx = int(w * 0.5)
        cy = int(h * 0.5)

        cv2.line(img, (cx - 20, cy), (cx + 20, cy), (255, 255, 255), 2)
        cv2.line(img, (cx, cy - 20), (cx, cy + 20), (255, 255, 255), 2)
        cv2.circle(img, (cx, cy), 5, (255, 255, 255), -1)

    # =====================================================
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

    # =====================================================
    def image_callback(self, msg):

        try:
            img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f"cv_bridge error: {e}")
            return
        rule_centers = []
        h, w, _ = img.shape
        debug = img.copy()
        img = cv2.GaussianBlur(img, (5,5), 0)
        self.draw_center_cross(debug)

        roi_mask = get_map_roi(img)
        img = cv2.bitwise_and(img, img, mask=roi_mask)

        target_array = ServoTargetArray()

        # =================================================
        # ENABLE CHECK
        # =================================================
        if not self.enabled:
            self.draw_text_bg(debug, "ENABLE: false", (10, 30), (0, 0, 255))

            debug_show = cv2.resize(debug, (640, 480))
            debug_msg = self.bridge.cv2_to_imgmsg(debug_show, 'bgr8')
            debug_msg.header = msg.header
            self.debug_pub.publish(debug_msg)
            return

        # =================================================
        # HSV
        # =================================================
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        color_ranges = {
            "red": [
                ([0, 60, 40], [20, 255, 255]),
                ([150, 60, 40], [180, 255, 255])
            ],
            "yellow": [
                ([22, 70, 170], [36, 255, 255])
            ],
            "green": [
                ([40, 50, 80], [85, 255, 255])
            ],
            "blue": [
                ([85, 60, 40], [145, 255, 255])
            ]
        }

        # =================================================
        # 规则图形
        # =================================================
        for color_name, ranges in color_ranges.items():

            mask = None

            for lower, upper in ranges:
                temp = cv2.inRange(hsv, np.array(lower), np.array(upper))
                mask = temp if mask is None else cv2.bitwise_or(mask, temp)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:

                hull = cv2.convexHull(cnt)
                area = cv2.contourArea(hull)

                if area < 3700:
                    continue

                perimeter = cv2.arcLength(hull, True)
                if perimeter == 0:
                    continue

                circularity = 4 * np.pi * area / (perimeter * perimeter)

                epsilon = 0.02 * perimeter
                approx = cv2.approxPolyDP(hull, epsilon, True)
                vertices = len(approx)

                if circularity > 0.976:
                    shape = "circle"
                elif 3 <= vertices <= 6:
                    shape = str(vertices)
                else:
                    continue

                M = cv2.moments(hull)
                if M["m00"] == 0:
                    continue

                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                rule_centers.append((cx, cy))
                dx = (cx - w * 0.5) / float(w)
                dy = (cy - h * 0.5) / float(h)

                target_type = int(self.encode_type(color_name, shape))

                # =================================================
                # 连续帧稳定滤波
                # type 相同 + 中心距离接近
                # =================================================
                

                t = ServoTarget()
                t.x = 0.0
                t.y = 0.0
                t.dx = float(dx)
                t.dy = float(dy)
                t.type = target_type
                t.score = float(area)

                # =================================================
# 横向中心死区（1/6）
# =================================================
               
                target_array.targets.append(t)

                draw_color = {
                    "red": (0, 0, 255),
                    "green": (0, 255, 0),
                    "yellow": (0, 255, 255),
                    "blue": (255, 0, 0)
                }.get(color_name, (255, 255, 255))

                cv2.drawContours(debug, [hull], -1, draw_color, 2)
                cv2.circle(debug, (cx, cy), 6, draw_color, -1)

                cv2.line(debug,
                         (int(w * 0.5), int(h * 0.5)),
                         (cx, cy),
                         draw_color,
                         2)

                x, y, bw, bh = cv2.boundingRect(cnt)

                cv2.rectangle(debug,
                              (x, y),
                              (x + bw, y + bh),
                              draw_color,
                              2)

                self.draw_text_bg(debug,
                                  f"{color_name} {shape}",
                                  (x, max(20, y - 25)),
                                  draw_color)

                self.draw_text_bg(debug,
                                  f"dx={dx:+.3f} dy={dy:+.3f}",
                                  (x, max(40, y - 5)),
                                  draw_color)
                                # 在画完轮廓、中心点、bounding box 后，增加一行：
                self.draw_text_bg(debug, f"area={int(area)}", (x, y + bh + 15), draw_color)

        # =================================================
        # 不规则图形
        # =================================================
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (7,7), 0)

        edges = cv2.Canny(gray, 30, 100)

        kernel = np.ones((5, 5), np.uint8)
        edges = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel)

    

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:

            area = cv2.contourArea(cnt)

            if area < 3800:
                continue
            
        # =================================================
        # bounding box
        # =================================================
            x, y, bw, bh = cv2.boundingRect(cnt)

            if bh == 0:
                continue

            ratio = bw / float(bh)

            # 👉 放宽比例
            if ratio > 2.5 or ratio < 0.25:
                continue

            # =================================================
            # 尺寸限制（避免太大噪声）
            # =================================================
            if bw > 360 or bh > 360:
                continue
            if bw < 100 or bh <100:
                continue
            # =================================================
            # convex hull
            # =================================================
            hull = cv2.convexHull(cnt)
            hull_area = cv2.contourArea(hull)

            if hull_area <= 0:
                continue

            # =================================================
            # perimeter
            # =================================================
            perimeter = cv2.arcLength(cnt, True)
            if perimeter <= 0:
                continue

            # =================================================
            # circularity（放宽）
            # =================================================
            circularity = 4 * np.pi * area / (perimeter * perimeter)

            if circularity > 0.985:
                continue

            # =================================================
            # solidity（关键放宽）
            # =================================================
            solidity = area / hull_area

            if solidity > 0.99:
                continue

            # =================================================
            # polygon approx
            # =================================================
            epsilon = 0.02 * perimeter
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            vertices = len(approx)

            # 👉 只排除“非常规则”的情况，不要一刀切
            if 3<=vertices<=6 and area<=3500:
                continue

            M = cv2.moments(cnt)
            if M["m00"] == 0:
                continue

            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # =========================
            # 是否靠近规则图形
            # =========================
            is_rule = False

            for (rx, ry) in rule_centers:
                if abs(cx - rx) < 80 and abs(cy - ry) < 80:
                    is_rule = True
                    break

            # 👉 如果是规则图形附近，直接跳过 irregular
            if is_rule:
                continue
            dx = (cx - w * 0.5) / float(w)
            dy = (cy - h * 0.5) / float(h)

            target_type = 21

            # =================================================
            # 连续帧稳定滤波
            # =================================================
            

            t = ServoTarget()
            t.x = 0.0
            t.y = 0.0
            t.dx = float(dx)
            t.dy = float(dy)
            t.type = target_type
            t.score = float(area)

            # =================================================
# 横向中心死区（1/6）
# =================================================
           
            target_array.targets.append(t)

            cv2.drawContours(debug, [cnt], -1, (255, 255, 255), 2)
            cv2.circle(debug, (cx, cy), 6, (255, 255, 255), -1)

            cv2.line(debug,
                     (int(w * 0.5), int(h * 0.5)),
                     (cx, cy),
                     (255, 255, 255),
                     2)

            x, y, bw, bh = cv2.boundingRect(cnt)

            cv2.rectangle(debug,
                          (x, y),
                          (x + bw, y + bh),
                          (255, 255, 255),
                          2)

        # =================================================
                # =================================================
        # 发布目标
        # =================================================
        self.pub.publish(target_array)

        self.draw_text_bg(
            debug,
            f"ENABLE: true | targets: {len(target_array.targets)}",
            (10, 30),
            (0, 255, 0)
        )

        # =================================================
        # 降低 debug 图像分辨率
        # =================================================
        debug_show = cv2.resize(debug, (640, 480))

        debug_msg = self.bridge.cv2_to_imgmsg(debug_show, 'bgr8')
        debug_msg.header = msg.header

        self.debug_pub.publish(debug_msg)


# =========================================================
def main(args=None):
    rclpy.init(args=args)
    node = ShapeColorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()