
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import logging

from eye_control import EyeControl

from ira_interfaces.msg import SystemState
from ira_interfaces.msg import FoiCoord

class EyeNode(Node):
        
    def __init__(self):
        super().__init__('eye_node')
        self.declare_parameter('sim', False)
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value

        self.eyes = EyeControl()

        self.eye_state = "default"
        self.foi_coordinates = [512,512]

        timer_period = 0.4  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
        self.system_state_subscription = self.create_subscription(
            SystemState,
            'system_state', 
            self.system_state_callback, 
            10
        )
        self.foi_cooridnates_subscription = self.create_subscription(
            FoiCoord,
            'foi_coordinates', 
            self.foi_coordinates_callback, 
            10
        )
        self.get_logger().info("Eye node initialised")

    def foi_coordinates_callback(self, msg):
        """
        Update the coordinates of the face of interest,
        so that the eyes can point towards the face.
        """
        self.get_logger().debug("In foi_coordinates_callback")
        self.foi_coordinates = [msg.x, msg.y]

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        # Display the message on the console
        self.get_logger().debug("In system_state_callback")

        # TODO check seq ?
        if msg.state == 'scanning':
            self.eye_state = "default"
        elif msg.state == 'found_noone':
            self.eye_state = "default"
        elif msg.state == 'found_unknown':
            self.eye_state = "default"
        elif msg.state == 'found_known':
            self.eye_state = "default"
        elif msg.state == 'say_painted_recently':
            self.eye_state = "focus"
        elif msg.state == 'too_far':
            self.eye_state = "default"
        elif msg.state == 'interaction_unknown':
            self.eye_state = "focus"
        elif msg.state == 'interaction_known':
            self.eye_state = "focus"
        elif msg.state == 'interaction_known_recent':
            self.eye_state = "focus"
        elif msg.state == 'disappeared':
            self.eye_state = "default"
        elif msg.state == 'interaction_returned':
            self.eye_state = "focus"
        elif msg.state == 'gone':
            self.eye_state = "default"
        elif msg.state == 'painting':
            self.eye_state = "straight"
        elif msg.state == 'completed':
            self.eye_state = "default"
        else:
            self.eye_state = "default"

    def timer_callback(self):
        """
        Every x seconds, update the command being run by the eyes.
        """
        self.get_logger().debug("In timer_callback")
        if self.eye_state == "default":
            # Just look around randomly
            self.eyes.default_movement()
        if self.eye_state == "straight":
            # Look straight ahead (while painting)
            self.eyes.straight()
        if self.eye_state == "focus":
            # Focus on the foi
            self.eyes.focus(self.foi_coordinates[0], self.foi_coordinates[1])


def main(args=None):
    rclpy.init(args=args)

    eye_node = EyeNode()

    rclpy.spin(eye_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    eye_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    