import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from custom_vision_msgs.msg import ServoTarget, ServoTargetArray


class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()
        self.enabled = False

        self.weights = "YOUR_MODEL_PATH"

        self.conf_threshold = 0.7
        self.iou_threshold = 0.45
        self.max_det = 5

        self.model = None
        # 定义类别
        self.class_names = {
            0: "elephant",
            1: "tiger",
            2: "wolf",
            3: "monkey",
            4: "peacock"
        }

        if self.weights != "YOUR_MODEL_PATH":
            try:
                self.get_logger().info(f"Loading YOLO model: {self.weights}")
                self.model = YOLO(self.weights)
                self.get_logger().info(f"Using {len(self.class_names)} classes: {list(self.class_names.values())}")
            except Exception as e:
                self.get_logger().error(f"Failed to load model: {e}")

        self.create_subscription(Image, "/camera/down", self.image_callback, 10)
        self.create_subscription(Bool, "/vision/down/enable", self.enable_callback, 10)

        self.pub = self.create_publisher(
            ServoTargetArray, 
            "/yolo_points", 
            10
        )
        self.debug_pub = self.create_publisher(
            Image, 
            "/vision/yolo/debug_image",
            10
        )

    def enable_callback(self, msg):
        self.enabled = msg.data

    def draw_center_cross(self, img):
        h, w = img.shape[:2]
        cx, cy = w // 2, h // 2
        cv2.line(img, (cx-20, cy), (cx+20, cy), (0,255,0), 2)
        cv2.line(img, (cx, cy-20), (cx, cy+20), (0,255,0), 2)
        cv2.circle(img, (cx, cy), 4, (0,255,0), -1)

    def draw_text_bg(self, img, text, org, color=(255,255,255)):
        font=cv2.FONT_HERSHEY_SIMPLEX
        scale=0.55
        thick=2
        (tw,th),base=cv2.getTextSize(text,font,scale,thick)
        x,y=org
        cv2.rectangle(img,(x-3,y-th-5),(x+tw+3,y+base+3),(0,0,0),-1)
        cv2.putText(img,text,org,font,scale,color,thick,cv2.LINE_AA)

    def image_callback(self, msg):
        frame=self.bridge.imgmsg_to_cv2(msg,"bgr8")
        debug=frame.copy()
        h,w=frame.shape[:2]
        cx,cy=w//2,h//2
        self.draw_center_cross(debug)

        if not self.enabled:
            self.draw_text_bg(debug,"ENABLE: false",(10,30),(0,0,255))
            out=self.bridge.cv2_to_imgmsg(debug,"bgr8")
            out.header=msg.header
            self.debug_pub.publish(out)
            return

        if self.model is None:
            self.draw_text_bg(debug,"NO MODEL",(10,30),(0,0,255))
            out=self.bridge.cv2_to_imgmsg(debug,"bgr8")
            out.header=msg.header
            self.debug_pub.publish(out)
            return

        rgb=cv2.cvtColor(frame,cv2.COLOR_BGR2RGB)

        results=self.model(
            rgb,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            max_det=self.max_det,
            verbose=False
        )

        tsa=ServoTargetArray()
        found=False

        for result in results:
            for box in result.boxes:
                x1,y1,x2,y2=map(int,box.xyxy[0].tolist())
                conf=float(box.conf[0])
                cls=int(box.cls[0])

                bx=(x1+x2)//2
                by=(y1+y2)//2

                dx=float(bx-cx)
                dy=float(by-cy)

                label=self.class_names.get(cls,str(cls))

                cv2.rectangle(debug,(x1,y1),(x2,y2),(0,255,0),2)
                self.draw_text_bg(debug,f"{label} {conf:.2f}",(x1,max(20,y1-10)))
                cv2.circle(debug,(bx,by),4,(0,255,255),-1)

                t=ServoTarget()
                t.x=float(bx)
                t.y=float(by)
                t.dx=dx
                t.dy=dy
                t.type=cls
                t.score=conf
                tsa.targets.append(t)
                found=True

        if not found:
            self.draw_text_bg(debug,"no YOLO target",(10,30),(0,0,255))

        self.pub.publish(tsa)
        out=self.bridge.cv2_to_imgmsg(debug,"bgr8")
        out.header=msg.header
        self.debug_pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node=YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=="__main__":
    main()