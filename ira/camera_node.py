
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import logging
import cv2

from camera import Camera

class CameraNode(Node):
        
    def __init__(self):
        super().__init__('camera_node')

        self.logger = logging.getLogger("main_logger")

        self.camera = Camera()
        self.bridge = CvBridge()

        # Initialise publishers
        self.latest_image_publisher = self.create_publisher(Image, 'latest_image', 10) #TODO create a custom message type for this?

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

    
    def timer_callback(self):
        """
        Read from the camera and publish
        an image to the latest_image topic.
        """
        self.logger().info('In timer_callback')
        latest_image = self.camera.read()
        self.latest_image_publisher.publish(self.bridge.cv2_to_imgmsg(latest_image, "bgr8"))


def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()