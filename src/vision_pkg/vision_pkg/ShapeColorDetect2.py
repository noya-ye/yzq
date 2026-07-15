import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from custom_vision_msgs.msg import ServoTarget, ServoTargetArray

# def get_map_roi(frame):
#     hsv=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
#     lower=np.array([0,0,140])
#     upper=np.array([180,70,255])
#     mask=cv2.inRange(hsv,lower,upper)
#     kernel=np.ones((7,7),np.uint8)
#     mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
#     mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
#     contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
#     roi_mask=np.zeros_like(mask)
#     if contours:
#         cnt=max(contours,key=cv2.contourArea)
#         hull=cv2.convexHull(cnt)
#         cv2.drawContours(roi_mask,[hull],-1,255,-1)
#     return roi_mask


class ShapeColorNode(Node):
    def __init__(self):
        super().__init__('shape_color_down_node')
        self.bridge=CvBridge()

        self.cap=cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT,480)
        self.cap.set(cv2.CAP_PROP_FPS,30)

        if not self.cap.isOpened():
            self.get_logger().error("Camera open failed")

        self.enabled=True
        self.create_subscription(Bool,'/vision/down/enable',self.enable_callback,10)

        self.pub=self.create_publisher(
            ServoTargetArray,
            '/vision/down/servo_targets',
            10
        )

        # self.debug_pub=self.create_publisher(
        #     Image,
        #     '/vision/down/debug_image',
        #     10
        # )

        self.timer=self.create_timer(0.02,self.loop)

        self.get_logger().info("ShapeColorDetect_Node started")


    def enable_callback(self,msg):
        self.enabled=bool(msg.data)


    def encode_type(self,color,shape):
        table={
            ('red','circle'):1,('red','4'):2,('red','3'):3,('red','5'):4,('red','6'):5,
            ('green','circle'):6,('green','4'):7,('green','3'):8,('green','5'):9,('green','6'):10,
            ('yellow','circle'):11,('yellow','4'):12,('yellow','3'):13,('yellow','5'):14,('yellow','6'):15,
            ('blue','circle'):16,('blue','4'):17,('blue','3'):18,('blue','5'):19,('blue','6'):20,
            ('irregular','irregular'):21
        }
        return table.get((color,shape),0)


    # def draw_center_cross(self,img):
    #     h,w=img.shape[:2]
    #     cx=w//2
    #     cy=h//2
    #     cv2.line(img,(cx-20,cy),(cx+20,cy),(255,255,255),2)
    #     cv2.line(img,(cx,cy-20),(cx,cy+20),(255,255,255),2)
    #     cv2.circle(img,(cx,cy),5,(255,255,255),-1)


    # def draw_text_bg(self,img,text,org,color=(255,255,255)):
    #     font=cv2.FONT_HERSHEY_SIMPLEX
    #     scale=0.55
    #     thickness=2
    #     (tw,th),base=cv2.getTextSize(text,font,scale,thickness)
    #     x,y=org
    #     cv2.rectangle(img,(x-3,y-th-5),(x+tw+3,y+base+3),(0,0,0),-1)
    #     cv2.putText(img,text,org,font,scale,color,thickness,cv2.LINE_AA)


    def loop(self):
        if not self.cap.isOpened():
            return

        ret,img=self.cap.read()
        if not ret:
            return

        # debug=img.copy()
        h,w=img.shape[:2]
        # self.draw_center_cross(debug)

        if not self.enabled:
            # self.draw_text_bg(debug,"ENABLE: false",(10,30),(0,0,255))
            # msg=self.bridge.cv2_to_imgmsg(cv2.resize(debug,(640,480)),"bgr8")
            # msg.header.stamp=self.get_clock().now().to_msg()
            # self.debug_pub.publish(msg)
            return

        img=cv2.GaussianBlur(img,(5,5),0)
        # roi_mask=get_map_roi(img)
        # img=cv2.bitwise_and(img,img,mask=roi_mask)
        hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)

        target_array=ServoTargetArray()

        # 临时保存检测结果
        detected_objects=[]

        color_ranges={
            "red":[([0,60,40],[20,255,255]),([150,60,40],[180,255,255])],
            "yellow":[([22,70,170],[36,255,255])],
            "green":[([40,50,80],[85,255,255])],
            "blue":[([85,60,40],[145,255,255])]
        }


        for color_name,ranges in color_ranges.items():
            mask=None

            for lower,upper in ranges:
                temp=cv2.inRange(hsv,np.array(lower),np.array(upper))
                mask=temp if mask is None else cv2.bitwise_or(mask,temp)

            contours,_=cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

            for cnt in contours:
                hull=cv2.convexHull(cnt)
                area=cv2.contourArea(hull)

                if area<3700:
                    continue

                perimeter=cv2.arcLength(hull,True)

                if perimeter==0:
                    continue

                circularity=4*np.pi*area/(perimeter*perimeter)
                approx=cv2.approxPolyDP(hull,0.02*perimeter,True)
                vertices=len(approx)

                if circularity>0.976:
                    shape="circle"
                elif 3<=vertices<=6:
                    shape=str(vertices)
                else:
                    continue

                M=cv2.moments(hull)

                if M["m00"]==0:
                    continue

                cx=int(M["m10"]/M["m00"])
                cy=int(M["m01"]/M["m00"])

                dx=(cx-w*0.5)/float(w)
                dy=(cy-h*0.5)/float(h)

                detected_objects.append({
                    "cx":cx,
                    "cy":cy,
                    "dx":dx,
                    "dy":dy,
                    "type":self.encode_type(color_name,shape),
                    "score":float(area),
                    "color":color_name,
                    "shape":shape
                })

                # cv2.drawContours(debug,[hull],-1,(0,255,0),2)
                # cv2.circle(debug,(cx,cy),5,(0,255,255),-1)
                        # ==========================
        # 临时ID分配
        # ==========================
        detected_objects.sort(key=lambda obj:obj["cx"])

        for idx,obj in enumerate(detected_objects):

            t=ServoTarget()

            # 左到右分配ID
            # 左边=1 右边=2
            t.x=float(idx+1)

            t.y=0.0

            t.dx=float(obj["dx"])
            t.dy=float(obj["dy"])

            t.type=int(obj["type"])

            t.score=float(obj["score"])

            target_array.targets.append(t)


            # debug显示ID
            # self.draw_text_bg(
            #     debug,
            #     f"ID:{idx+1} {obj['color']} {obj['shape']}",
            #     (obj["cx"],obj["cy"]-15),
            #     (255,255,255)
            # )


        # ==========================
        # 发布目标
        # ==========================
        self.pub.publish(target_array)


        # self.draw_text_bg(
        #     debug,
        #     f"targets:{len(target_array.targets)}",
        #     (10,30),
        #     (0,255,0)
        # )


        # ==========================
        # 发布debug图像
        # ==========================
        # debug_show=cv2.resize(
        #     debug,
        #     (640,480)
        # )


        # msg=self.bridge.cv2_to_imgmsg(
        #     debug_show,
        #     "bgr8"
        # )

        # msg.header.stamp=self.get_clock().now().to_msg()
        # msg.header.frame_id="camera"

        # self.debug_pub.publish(msg)



    # ==========================
    # 释放摄像头
    # ==========================
    def destroy_node(self):

        if self.cap:
            self.cap.release()

        super().destroy_node()



def main(args=None):

    rclpy.init(args=args)

    node=ShapeColorNode()

    try:
        rclpy.spin(node)

    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__=="__main__":
    main()