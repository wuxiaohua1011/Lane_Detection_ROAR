import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image

import message_filters
from message_filters import TimeSynchronizer, Subscriber

from rclpy.clock import ROSClock
from rclpy.duration import Duration
from rclpy.time import Time
import unittest
import time

GROUND_MASK_LOWER = np.array([0,0,80],dtype='uint8')
GROUND_MASK_UPPER = np.array([255,50,200],dtype='uint8')
GRASS_MASK_LOWER = np.array([43,50,20],dtype='uint8')
GRASS_MASK_UPPER = np.array([128,255,255],dtype='uint8')

class RGDetect(Node):

    def __init__(self):
        super().__init__('rg_detect')
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            liveliness=QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            depth=1,
        )

        self.img_sub = self.create_subscription(
            Image,
            '/rgb_image',
            self.img_callback,
            qos_profile)
        self.img_sub  # prevent unused variable warning

        # self.img_sub = Subscriber(self, Image, "/floodfill_passthrough", qos_profile=qos_profile)
        # self.mask_pass_sub = Subscriber(self, Image, "/floodfill_mask", qos_profile=qos_profile)

        # sync_msg = TimeSynchronizer([self.img_sub, self.mask_pass_sub], 10)
        # sync_msg.registerCallback(self.img_callback)

        # self.floodfill_mask_pub = self.create_publisher(Image, "/floodfill_mask_image", qos_profile=qos_profile)
        self.rg_mask_pub = self.create_publisher(Image, "/rg_mask", qos_profile=qos_profile)

    def img_callback(self, msg):

        # Converting ROS image message to RGB
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # use color of gound and grass to mask the road
        rg_detection_mask = self.road_and_grass_detection(image)
        rg_edge = self.edge_mask(rg_detection_mask)

        rg_mask_msg = self.bridge.cv2_to_imgmsg(rg_edge, encoding="mono8")
        # rg_mask_msg.header.stamp = mask_pass.header.stamp
        rg_mask_msg.header.stamp = msg.header.stamp

        # self.floodfill_mask_pub.publish(mask_pass)
        self.rg_mask_pub.publish(rg_mask_msg)
    
    def groundAndGrassMask(self, image):
    
        dilkernel = np.ones((5,5), np.uint8)
        erokernel = np.ones((6,6), np.uint8)
        erokernel[:2,:] = 0
        erokernel[-2:,:] = 0
        erokernel[:,:2]=0
        erokernel[:,-2:]=0

        hsvImg = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        ground = cv2.inRange(hsvImg,GROUND_MASK_LOWER,GROUND_MASK_UPPER)
        grass = cv2.inRange(hsvImg,GRASS_MASK_LOWER,GRASS_MASK_UPPER)
        
        grass = cv2.erode(grass,erokernel,iterations=2)
        ground = cv2.erode(ground,erokernel,iterations=2)
        ground = cv2.dilate(ground,dilkernel,iterations=10)
        grass = cv2.dilate(grass, dilkernel, iterations=10)
        
        combined = cv2.bitwise_and(ground,cv2.bitwise_not(grass))
        combined = cv2.erode(combined,erokernel,iterations=4)
        
        return combined

    def getContour(self, alpha):
        #https://stackoverflow.com/questions/66753026/opencv-smoother-contour
        contours,hierachy = cv2.findContours(alpha,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        big_contour = max(contours, key=cv2.contourArea)
        contour_img = np.zeros_like(alpha)
        cv2.drawContours(contour_img, [big_contour], 0, 255, -1)
        # apply dilate to connect the white areas in the alpha channel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (40,40))
        dilate = cv2.morphologyEx(contour_img, cv2.MORPH_DILATE, kernel)
        return dilate

    def road_and_grass_detection(self, frame):
        comb = self.groundAndGrassMask(frame)
        dilatedContour = self.getContour(comb)
        return dilatedContour
        return comb

    def edge_mask(self, mask):
        # gx, gy = np.gradient(mask)
        # edge = gy * gy + gx * gx
        # edge[edge != 0.0] = 255
        # edge = edge.astype(np.uint8
        edge = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((3,3))).astype(np.uint8)

        return edge




def main(args=None):
    rclpy.init(args=args)

    rg_detect = RGDetect()

    rclpy.spin(rg_detect)

    # Destroy the node explicitly
    rg_detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()