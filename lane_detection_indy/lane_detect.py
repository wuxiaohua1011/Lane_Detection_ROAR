import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy

import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image

GROUND_MASK_LOWER = np.array([0,0,80],dtype='uint8')
GROUND_MASK_UPPER = np.array([255,50,200],dtype='uint8')
# GROUND_MASK_LOWER = np.array([0,0,40],dtype='uint8')
# GROUND_MASK_UPPER = np.array([180,30,200],dtype='uint8')
GRASS_MASK_LOWER = np.array([43,50,20],dtype='uint8')
GRASS_MASK_UPPER = np.array([128,255,255],dtype='uint8')


class LaneDetect(Node):

    def __init__(self):
        super().__init__('lane_detect')
        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            liveliness=QoSLivelinessPolicy.RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
            depth=10,
        )
        self.img_sub = self.create_subscription(
            Image,
            '/rgb_image',
            self.img_callback,
            qos_profile)

        self.mask_pub = self.create_publisher(Image, "/mask_image", qos_profile=qos_profile)
        self.img_sub  # prevent unused variable warning

    def img_callback(self, msg):

        # Converting ROS image message to RGB
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')

        # height = image.shape[0]
        # width = image.shape[1]
        # image = image[height//4:height//2, :width]

        # use color of gound and grass to mask the road
        rg_detection_mask = self.road_and_grass_detection(image)
        rg_edge = self.edge_mask(rg_detection_mask)
        
        # Floodfill the road
        floodfill_mask = self.floodfill(image)
        floodfill_mask = cv2.morphologyEx(floodfill_mask, cv2.MORPH_CLOSE, np.ones((7,7)))
        floodfill_edge = self.edge_mask(floodfill_mask)
        
        # merge two masks 
        floodfill_kernel = np.ones((20, 20))
        rg_kernel = np.ones((4, 4))
        dil_floodfill_edge = cv2.dilate(floodfill_edge, floodfill_kernel)
        dil_rg_edge = cv2.dilate(rg_edge, rg_kernel)
        final_edge = np.bitwise_and(dil_floodfill_edge, dil_rg_edge)
        #final_edge = cv2.dilate(final_edge, np.ones((4, 4)))


        
        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(final_edge, encoding="mono8"))
        # return final_lane

    def floodfill(self, img):

        height = img.shape[0]
        width  = img.shape[1]

        # Convert to HSV space
        img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)

        # Smooth the image
        img_hsv = cv2.GaussianBlur(img_hsv, (5,5), 0)

        # Choose bottom centre as starting point of floodfill
        seed = (height-1, width//2)
        # print(seed, height, width)
        
        # Create a box around seed and get average
        box = (200, 200)
        x = (seed[0] - box[0] - box[1], seed[0])
        y = (seed[1] - box[1], seed[1] + box[1])

        mean = np.mean(img_hsv[x[0]:x[1], y[0]:y[1], :], axis = (0,1))

        # TODO: Tune this as per requirement
        thre = [50, 50, 200]

        # Change the seed to calculated mean
        img_hsv[seed[0], seed[1]] = mean
        
        # print(img_hsv.shape, seed)

        seed = (width//2, height-1)
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
    
    def groundAndGrassMask(self, image):
    
        dilkernel = np.ones((5,5), np.uint8)
        erokernel = np.ones((6,6), np.uint8)
        erokernel[:2,:] = 0
        erokernel[-2:,:] = 0
        erokernel[:,:2]=0
        erokernel[:,-2:]=0

        hsvImg = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        ground = cv2.inRange(hsvImg,GROUND_MASK_LOWER,GROUND_MASK_UPPER)
        # cv2.imshow("Ground", ground)
        # cv2.waitKey(1)

        # groundd = cv2.dilate(ground,np.ones((10,10)))
        # grounde = cv2.erode(ground,np.ones((21,21)))
        # cv2.imshow("Grounde", grounde)
        # cv2.waitKey(1)
        grass = cv2.inRange(hsvImg,GRASS_MASK_LOWER,GRASS_MASK_UPPER)
        # cv2.imshow("Grass", grass)
        # cv2.waitKey(1)
        
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

    def edge_mask(self, mask):
        # gx, gy = np.gradient(mask)
        # edge = gy * gy + gx * gx
        # edge[edge != 0.0] = 255
        # edge = edge.astype(np.uint8
        edge = cv2.morphologyEx(mask, cv2.MORPH_GRADIENT, np.ones((3,3))).astype(np.uint8)

        return edge




def main(args=None):
    rclpy.init(args=args)

    lane_detect = LaneDetect()

    rclpy.spin(lane_detect)

    # Destroy the node explicitly
    lane_detect.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()