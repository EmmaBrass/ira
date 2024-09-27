
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from ira_common.eye_control import EyeControl

from ira_interfaces.msg import SystemState
from ira_interfaces.msg import FoiCoord

import time

class EyeNode(Node):
        
    def __init__(self):
        super().__init__('eye_node')
        self.declare_parameter('sim', False)
        self.declare_parameter('eyes_port', '/dev/ttyACM0')
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value
        self.eyes_port = '/dev/ttyACM0'

        self.eyes = EyeControl(com_port=self.eyes_port)

        self.swirl_complete = False
        self.eye_state = "default"

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
        self.system_state_subscription = self.create_subscription(
            SystemState,
            'system_state', 
            self.system_state_callback, 
            10
        )

        time.sleep(10)
        self.get_logger().info("Eye node initialised")
        self.get_logger().info(f"Simulation mode: {self.sim_mode}")


    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        self.get_logger().info("In system state callback for eyes!")
        if msg.state == 'startup':
            self.eye_state = "default"
        elif msg.state == 'your_turn':
            self.eye_state = "default"
        elif msg.state == 'looking':
            self.eye_state = "down_straight"
        elif msg.state == 'comment':
            self.eye_state = "default"
        elif msg.state == 'my_turn':
            self.eye_state = "default"
        elif msg.state == 'ask_done':
            self.eye_state = "straight"
        elif msg.state == 'completed':
            self.eye_state = "swirl"
        else:
            self.eye_state = "default"

    def timer_callback(self):
        """
        Every x seconds, update the command being run by the eyes.
        """
        self.get_logger().info("In timer_callback")
        if self.eye_state == "default":
            # Just look around randomly
            self.eyes.default_movement()
            self.swirl_complete = False
        if self.eye_state == "swirl" and self.swirl_complete == False:
            # Swirl the eyes in opposite directions
            self.eyes.swirl()
            self.swirl_complete = True
        if self.eye_state == "straight":
            # Look straight ahead (while painting)
            self.eyes.straight()
            self.swirl_complete = False
        if self.eye_state == "straight":
            # Look down for 4 seconds and then straight
            self.eyes.down_straight()
            self.swirl_complete = False


    def map_value(self, x, in_min, in_max, out_min, out_max):
        return out_min + ((x - in_min) * (out_max - out_min) / (in_max - in_min))


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
    