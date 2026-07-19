import rclpy
from rclpy.node import Node
import cv2
import torch
from ultralytics import YOLO
from custom_vision_msgs.msg import ServoTarget, ServoTargetArray

class YOLONode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        self.weights="/home/jetson/ros2_ws_px4/src/vision_pkg/animals.pt"
        self.device="cuda:0"
        self.conf_threshold=0.7
        self.iou_threshold=0.3
        self.max_det=5
        self.model=None
        self.class_names={
            0:"elephant",
            1:"wolf",
            2:"peacock",
            3:"tiger",
            4:"monkey"
        }
        if torch.cuda.is_available():
            self.get_logger().info(
                f"CUDA available: {torch.cuda.get_device_name(0)}"
            )
        else:
            self.get_logger().warning(
                "CUDA unavailable, using CPU"
            )
            self.device="cpu"
        try:
            self.get_logger().info(
                f"Loading YOLO model: {self.weights}"
            )
            self.model=YOLO(self.weights)
            self.model.to(self.device)
            self.get_logger().info(
                "YOLO model loaded"
            )
        except Exception as e:
            self.get_logger().error(
                f"Model loading failed: {e}"
            )
        self.cap=cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error(
                "Camera open failed"
            )
        else:
            self.get_logger().info(
                "Camera started"
            )
        self.pub=self.create_publisher(
            ServoTargetArray,
            "/vision/down/servo_targets",
            10
        )
        self.timer=self.create_timer(
            0.05,
            self.camera_callback
        )

    def camera_callback(self):
        ret,frame=self.cap.read()
        if not ret:
            self.get_logger().warning(
                "Camera frame failed"
            )
            return

        h,w=frame.shape[:2]

        crop_size=300
        crop_size=min(crop_size,h,w)

        x1=(w-crop_size)//2
        y1=(h-crop_size)//2
        x2=x1+crop_size
        y2=y1+crop_size

        frame=frame[y1:y2,x1:x2]

        h,w=frame.shape[:2]
        cx=w//2
        cy=h//2

        if self.model is None:
            return

        results=self.model.track(
            frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            max_det=self.max_det,
            device=self.device,
            persist=True,
            tracker="bytetrack.yaml",
            verbose=False
        )

        target_array=ServoTargetArray()

        for result in results:

            if result.boxes.id is None:
                continue

            boxes=result.boxes

            ids=boxes.id.cpu().numpy()

            for box,track_id in zip(boxes,ids):

                x1,y1,x2,y2=map(
                    int,
                    box.xyxy[0].tolist()
                )

                conf=float(
                    box.conf[0]
                )

                cls=int(
                    box.cls[0]
                )

                bx=(x1+x2)//2
                by=(y1+y2)//2

                animal_name=self.class_names.get(
                    cls,
                    "unknown"
                )

                track_id=int(track_id)

                self.get_logger().info(
                    f"Detected {animal_name}, ID:{track_id}, confidence:{conf:.2f}"
                )

                dx=float(
                    bx-cx
                )/10

                dy=float(
                    by-cy
                )/10

                target=ServoTarget()

                target.x=float(
                    track_id
                )

                target.y=float(
                    by
                )

                target.dx=dx

                target.dy=dy

                target.type=int(
                    cls
                )

                target.score=1000*float(
                    conf
                )

                target_array.targets.append(
                    target
                )

        self.pub.publish(
            target_array
        )

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node=YOLONode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()