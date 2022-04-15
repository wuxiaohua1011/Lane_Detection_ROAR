import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image


class FloodFill(Node):

    def __init__(self):
        super().__init__('flood_fill')
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

        # self.img_pub = self.create_publisher(Image, "/floodfill_passthrough", qos_profile=qos_profile)

        self.mask_pub = self.create_publisher(Image, "/floodfill_mask", qos_profile=qos_profile)

    def img_callback(self, msg):

        # Converting ROS image message to RGB
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        # Convert to HSV
        image_hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        # Floodfill the road
        floodfill_mask = self.floodfill(image_hsv)
        floodfill_mask = cv2.morphologyEx(floodfill_mask, cv2.MORPH_CLOSE, np.ones((7,7)))
        floodfill_edge = self.edge_mask(floodfill_mask)

        floodfill_edge_msg = self.bridge.cv2_to_imgmsg(floodfill_edge, encoding="mono8")
        floodfill_edge_msg.header.stamp = msg.header.stamp

        self.mask_pub.publish(floodfill_edge_msg)
        # return final_lane

    def floodfill(self, img):

        height = img.shape[0]
        width  = img.shape[1]

        # Smooth the image
        img_hsv = cv2.GaussianBlur(img, (5,5), 0)

        # Choose bottom centre as starting point of floodfill
        seed = (height-1, width//2)
        
        # Create a box around seed and get average
        box = (200, 200)
        x = (seed[0] - box[0] - box[1], seed[0])
        y = (seed[1] - box[1], seed[1] + box[1])

        mean = np.mean(img_hsv[x[0]:x[1], y[0]:y[1], :], axis = (0,1))

        # TODO: Tune this as per requirement
        thre = [50, 50, 200]

        # Change the seed to calculated mean
        img_hsv[seed[0], seed[1]] = mean
        
        mask = np.zeros((img_hsv.shape[0] + 2, img_hsv.shape[1] + 2)).astype(np.uint8)
        cv2.floodFill(
            img_hsv,
            mask,
            seedPoint=seed,
            newVal=(255, 0, 0),
            loDiff=tuple(thre),
            upDiff=tuple(thre),
            flags= cv2.FLOODFILL_FIXED_RANGE
        )

        # Resize after floodfill extension
        mask =  mask[1:-1, 1:-1] * 255
        return mask

    def edge_mask(self, mask):
        edge = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((3,3))).astype(np.uint8)
        return edge




def main(args=None):
    rclpy.init(args=args)

    flood_fill = FloodFill()

    rclpy.spin(flood_fill)

    # Destroy the node explicitly
    flood_fill.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()