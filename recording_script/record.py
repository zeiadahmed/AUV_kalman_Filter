#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge

class VisualOdometry(Node): 
    def __init__(self):
        super().__init__('Node')
        #print('HIIIIIIIIIIIIIIIIIIIi')
        self.get_logger().info('Node init')
        self.imageSubscriber = self.create_subscription(Image,"/rexrov/rexrov/camera/image_raw",self.image_receive_callback,10)
        #self.imageLeftSubscriber = self.create_subscription(Image,"/rexrov/rexrov/cameraleft/image_raw",self.imageleft_receive_callback,10)
        #self.imageRightSubscriber = self.create_subscription(Image,"/rexrov/rexrov/cameraright/image_raw",self.imageright_receive_callback,10)
        self.i = 0
        self.flag = 0
        #self.out = cv2.VideoWriter('output{}.avi'.format(self.i), cv2.VideoWriter_fourcc(*'MJPG'), 10, (768,492))
        self.get_logger().info('Subscriber init')
        self.bridge = CvBridge()
        

    def image_receive_callback(self,img):
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        cv2.imshow("middle",cv_img)
        #self.out.write(cv_img)
        #self.i+=1
        key = cv2.waitKey(1)
        if key == ord("s"):
            self.out.release()
            self.get_logger().info("Done release")
            self.i+=1
            self.flag = 0
        elif key == ord("r"):
            #self.out = cv2.VideoWriter('output{}.avi'.format(self.i), cv2.VideoWriter_fourcc(*'MJPG'), 10, (768,492))
            self.out = cv2.VideoWriter('output{}.avi'.format(self.i), cv2.VideoWriter_fourcc('F','M','P','4'), 10, (768,492))
            self.get_logger().info("Record started")
            self.flag = 1
        if self.flag == 1:
            self.out.write(cv_img)

        
        

    def imageleft_receive_callback(self,img):
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        cv2.imshow("left",cv_img)
        cv2.waitKey(3)

    def imageright_receive_callback(self,img):
        cv_img = self.bridge.imgmsg_to_cv2(img,desired_encoding="bgr8")
        cv2.imshow("right",cv_img)
        cv2.waitKey(3)
 

def main(args=None):
    rclpy.init(args=args)
    node = VisualOdometry()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()