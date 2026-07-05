import pyrealsense2 as rs
import numpy as np
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image ,CameraInfo
class D435Node(Node):
    def __init__(self):
        super().__init__('d435_node')
#工具
        self.bridge=CvBridge()
#创建发布
        
        self.color_pub=self.create_publisher(
            Image,
            '/d435/color',
            10
        )
        self.depth_pub=self.create_publisher(
            Image,
            '/d435/depth',
            10
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/d435/camera_info', 
            10
        )

#d435部分
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        self.profile = self.pipeline.start(self.config)

        self.align = rs.align(rs.stream.color)

        depth_sensor = self.profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()

        #深度滤波
        self.spatial=rs.spatial_filter()
        self.spatial.set_option(rs.option.filter_magnitude, 2)
        self.spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self.spatial.set_option(rs.option.filter_smooth_delta, 20)
        self.hole = rs.hole_filling_filter()

        color_stream = self.profile.get_stream(rs.stream.color)
        self.intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        self.timer = self.create_timer(0.03, self.publish_frames)  #30Hz

        self.get_logger().info("D435Node have started")

    def publish_frames(self):

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            return
        
        #滤波
        
        depth_frame=self.spatial.process(depth_frame)
        depth_frame=self.hole.process(depth_frame)
        
        depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32) * self.depth_scale
        color_image = np.asanyarray(color_frame.get_data())
        
        #ros时间
        stamp = self.get_clock().now().to_msg()

        color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = "camera_link"
        self.color_pub.publish(color_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='32FC1')
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = "camera_link"
        self.depth_pub.publish(depth_msg)

        # 发布相机内参
        cam_info = CameraInfo()
        cam_info.header.stamp = stamp
        cam_info.header.frame_id = "camera_link"

        cam_info.width = self.intrinsics.width
        cam_info.height = self.intrinsics.height

        #内参发布格式矩阵
        cam_info.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        """
        应用：
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]
        X = (x - cx) * Z / fx
        Y = (y - cy) * Z / fy
        Z = depth[y, x](取区域平均最好)
        """
        self.camera_info_pub.publish(cam_info)

def main(args=None):
    rclpy.init(args=args)
    node = D435Node()
    rclpy.spin(node)

    node.pipeline.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()