import rclpy
from rclpy.node import Node
import cv2
import time
from std_msgs.msg import Bool

class VideoRecordNode(Node):
    def __init__(self):
        super().__init__('video_record_node')
        self.cap=cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")
        self.enabled=True
        self.recording=False
        self.save_path="/home/jetson/ros2_ws_px4/src/vision_pkg/flight_record.mp4"
        self.writer=None
        self.fps=30
        self.create_subscription(
            Bool,
            '/vision/down/enable',
            self.enable_callback,
            10
        )
        self.start_record()
        self.timer=self.create_timer(
            1.0/30.0,
            self.record_loop
        )
        self.get_logger().info("VideoRecordNode started")
    def enable_callback(self,msg):
        enable=bool(msg.data)
        if enable and not self.recording:
            self.start_record()
        elif not enable and self.recording:
            self.stop_record()
        self.enabled=enable
    def start_record(self):
        if self.recording:
            return
        if self.save_path=="":
            path="flight_record_"+str(int(time.time()))+".mp4"
        else:
            path=self.save_path
        ret,frame=self.cap.read()
        if not ret:
            self.get_logger().error("Camera frame failed")
            return
        h,w=frame.shape[:2]
        fourcc=cv2.VideoWriter_fourcc(*'mp4v')
        self.writer=cv2.VideoWriter(
            path,
            fourcc,
            self.fps,
            (w,h)
        )
        if not self.writer.isOpened():
            self.get_logger().error("VideoWriter open failed")
            self.writer=None
            return
        self.recording=True
        self.get_logger().info(
            f"Start recording: {path}, resolution={w}x{h}"
        )
    def stop_record(self):
        if not self.recording:
            return
        if self.writer:
            self.writer.release()
            self.writer=None
        self.recording=False
        self.get_logger().info("Recording stopped")
    def record_loop(self):
        ret,frame=self.cap.read()
        if not ret:
            self.get_logger().warning("Camera frame failed")
            return
        if self.recording and self.writer:
            self.writer.write(frame)
    def destroy_node(self):
        self.stop_record()
        if self.cap:
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node=VideoRecordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()