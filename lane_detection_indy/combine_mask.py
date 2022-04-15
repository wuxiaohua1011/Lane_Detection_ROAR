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


class MergeMask(Node):

    def __init__(self):
        super().__init__('merge_mask')
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            liveliness=QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            depth=1,
        )

        self.rg_mask_sub = Subscriber(self, Image, "/rg_mask", qos_profile=qos_profile)
        self.flood_mask_sub = Subscriber(self, Image, "/floodfill_mask", qos_profile=qos_profile)

        sync_msg = TimeSynchronizer([self.rg_mask_sub, self.flood_mask_sub], 10)
        sync_msg.registerCallback(self.img_callback)

        self.final_mask_pub = self.create_publisher(Image, "/final_mask", qos_profile=qos_profile)

    def img_callback(self, rg_mask_msg, flood_mask_msg):
        
        ff_mask = self.bridge.imgmsg_to_cv2(flood_mask_msg, desired_encoding='mono8')
        rg_mask = self.bridge.imgmsg_to_cv2(rg_mask_msg, desired_encoding='mono8')

        # merge two masks 
        floodfill_kernel = np.ones((20, 20))
        rg_kernel = np.ones((4, 4))
        dil_floodfill_edge = cv2.dilate(ff_mask, floodfill_kernel)
        dil_rg_edge = cv2.dilate(rg_mask, rg_kernel)
        final_edge = np.bitwise_and(dil_floodfill_edge, dil_rg_edge)
        
        self.final_mask_pub.publish(self.bridge.cv2_to_imgmsg(final_edge, encoding="mono8"))
        # return final_lane


def main(args=None):
    rclpy.init(args=args)

    merge_mask = MergeMask()

    rclpy.spin(merge_mask)

    # Destroy the node explicitly
    merge_mask.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()