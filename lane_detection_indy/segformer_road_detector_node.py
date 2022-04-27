import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
)

import cv2
from cv_bridge import CvBridge
import numpy as np
from sensor_msgs.msg import Image
import paddle
from paddleseg.cvlibs import Config
from paddleseg.transforms import Compose
from paddleseg import utils
from paddleseg.core import infer
from paddleseg.utils import visualize
from paddleseg.utils.visualize import get_pseudo_color_map
from PIL import Image as PILImage


class SegformerRoadDetectorNode(Node):
    """Filter out road from the image. Output a mask of road

    Args:
        Node (rclpy.node): Node that output mask of road
    """

    def __init__(self):
        super().__init__("segformer_road_detector_node")
        self.bridge = CvBridge()
        self.declare_parameter("rgb_camera_topic", "rgb_image")

        self.rgb_camera_topic = (
            self.get_parameter("rgb_camera_topic").get_parameter_value().string_value
        )
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.SYSTEM_DEFAULT,
            durability=QoSDurabilityPolicy.SYSTEM_DEFAULT,
            liveliness=QoSLivelinessPolicy.SYSTEM_DEFAULT,
            depth=1,
        )
        self.img_sub = self.create_subscription(
            Image, self.rgb_camera_topic, self.img_callback, qos_profile=qos_profile
        )
        self.get_logger().info(f"Listening on [{self.img_sub.topic}]")
        self.mask_pub = self.create_publisher(
            Image, f"{self.rgb_camera_topic}/road_mask", qos_profile=qos_profile
        )
        self.img_sub  # prevent unused variable warning

        ### paddle paddle
        self.config_path = "/home/roar/Desktop/projects/roar-indy-ws/src/Lane_Detection_ROAR/configs/segformer_b0_cityscapes_1024x1024_160k.yml"
        self.model_path = "/home/roar/Desktop/projects/roar-indy-ws/src/Lane_Detection_ROAR/configs/model.pdparams"
        paddle.set_device("gpu")

        self.cfg = Config(path=self.config_path)
        msg = "\n---------------Config Information---------------\n"
        msg += str(self.cfg)
        msg += "------------------------------------------------"
        self.get_logger().info(msg)
        self.model = self.cfg.model
        self.transforms = Compose(self.cfg.val_transforms)
        utils.utils.load_entire_model(self.model, self.model_path)
        self.model.eval()  # turn on eval mode
        self.color_map = visualize.get_color_map_list(256, custom_color=None)

    def img_callback(self, msg: Image):
        # Converting ROS image message to RGB
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        with paddle.no_grad():
            im, _ = self.transforms(image)
            im = im[np.newaxis, ...]
            im = paddle.to_tensor(im)
            pred, _ = infer.inference(
                self.model,
                im,
                ori_shape=(2160, 3840),
                transforms=self.transforms.transforms,
                is_slide=True,
                stride=[768, 768],
                crop_size=[1024, 1024],
            )
            pred = paddle.squeeze(pred)
            pred = pred.numpy().astype("uint8")

            # save pseudo color prediction
            added_image = self.visualize(image, pred, self.color_map, weight=0.6)
            mask_msg = self.bridge.cv2_to_imgmsg(added_image)
            mask_msg.header.frame_id = msg.header.frame_id
            mask_msg.header.stamp = msg.header.stamp
            self.mask_pub.publish(mask_msg)

    def visualize(self, image, result, color_map, weight=0.6):
        """
        Convert predict result to color image, and save added image.
        Args:
            image (str): The path of origin image.
            result (np.ndarray): The predict result of image.
            color_map (list): The color used to save the prediction results.
            save_dir (str): The directory for saving visual image. Default: None.
            weight (float): The image weight of visual image, and the result weight is (1 - weight). Default: 0.6
        Returns:
            vis_result (np.ndarray): If `save_dir` is None, return the visualized result.
        """

        color_map = [color_map[i : i + 3] for i in range(0, len(color_map), 3)]
        color_map = np.array(color_map).astype("uint8")
        # Use OpenCV LUT for color mapping
        c1 = cv2.LUT(result, color_map[:, 0])
        c2 = cv2.LUT(result, color_map[:, 1])
        c3 = cv2.LUT(result, color_map[:, 2])
        pseudo_img = np.dstack((c3, c2, c1))

        im = image
        vis_result = cv2.addWeighted(im, weight, pseudo_img, 1 - weight, 0)
        return vis_result


def main(args=None):
    rclpy.init(args=args)
    node = SegformerRoadDetectorNode()
    rclpy.spin(node)
    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
