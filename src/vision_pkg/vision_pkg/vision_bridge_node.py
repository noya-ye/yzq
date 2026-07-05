import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from custom_vision_msgs.msg import PositionTarget, PositionTargetArray

import math
# 订阅：
#   /vision/front/goal_position   geometry_msgs/msg/PointStamped

# 发布：
#   /vision/front/targets         custom_vision_msgs/msg/PositionTargetArray

# 作用：
#   接收前视视觉算出的单个目标世界坐标，
#   过滤掉距离原点超过 3.5m 的异常目标，
#   对 0.30m 内的重复目标做融合，
#   最终输出目标点数组给 TSP / offboard 任务使用。

class VisionBridgeNode(Node):

    def __init__(self):
        super().__init__('vision_bridge_node')

        self.input_topic = self.declare_parameter(
            'input_topic', '/vision/front/goal_position'
        ).value

        self.output_topic = self.declare_parameter(
            'output_topic', '/vision/front/targets'
        ).value

        # ===== 目标去重距离阈值 =====
        self.thresh = float(self.declare_parameter(
            'merge_thresh', 0.30
        ).value)

        # ===== 目标最大允许距离 =====
        # 只限制 XY 平面距离：sqrt(x^2 + y^2) <= max_dist
        self.max_dist = float(self.declare_parameter(
            'max_dist', 3.5
        ).value)

        self.max_dist2 = self.max_dist * self.max_dist

        self.sub = self.create_subscription(
            PointStamped,
            self.input_topic,
            self.callback,
            10
        )

        self.pub = self.create_publisher(
            PositionTargetArray,
            self.output_topic,
            10
        )

        self.targets = []

        self.get_logger().info(
            f"VisionBridgeNode started | "
            f"in={self.input_topic} | "
            f"out={self.output_topic} | "
            f"merge_thresh={self.thresh:.2f}m | "
            f"max_dist={self.max_dist:.2f}m"
        )

    def callback(self, msg):
        x = float(msg.point.x)
        y = float(msg.point.y)
        z = float(msg.point.z)

        # ============================================================
        # 1. 先过滤离原点太远的目标
        # 这里只判断 XY 平面距离，不把 z 算进去
        # ============================================================
        dist2_origin = x * x + y * y

        if dist2_origin > self.max_dist2:
            dist_origin = math.sqrt(dist2_origin)

            self.get_logger().warn(
                f"[VisionBridge] Reject far target: "
                f"({x:.2f}, {y:.2f}, {z:.2f}), "
                f"dist_xy={dist_origin:.2f}m > {self.max_dist:.2f}m"
            )
            return

        # ============================================================
        # 2. 和已有目标做距离匹配
        #    如果距离小于 thresh，认为是同一个目标，做平滑更新
        # ============================================================
        for i, t in enumerate(self.targets):
            dx = x - t[0]
            dy = y - t[1]
            d = math.sqrt(dx * dx + dy * dy)

            if d < self.thresh:
                self.targets[i][0] = 0.7 * t[0] + 0.3 * x
                self.targets[i][1] = 0.7 * t[1] + 0.3 * y
                self.targets[i][2] = 0.7 * t[2] + 0.3 * z

                self.publish_targets()
                return

        # ============================================================
        # 3. 如果不是已有目标，就加入新目标
        # ============================================================
        self.targets.append([x, y, z])

        self.get_logger().info(
            f"[VisionBridge] New front target: "
            f"({x:.2f}, {y:.2f}, {z:.2f}) "
            f"dist_xy={math.sqrt(dist2_origin):.2f}m "
            f"total={len(self.targets)}"
        )

        self.publish_targets()

    def publish_targets(self):
        msg = PositionTargetArray()

        for t in self.targets:
            # 双保险：发布前再过滤一次
            x = float(t[0])
            y = float(t[1])
            z = float(t[2])

            dist2_origin = x * x + y * y
            if dist2_origin > self.max_dist2:
                continue

            p = PositionTarget()
            p.x = x
            p.y = y
            p.z = z

            msg.targets.append(p)

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()