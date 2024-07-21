
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from ira.eye_control import EyeControl

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

        self.eye_state = "default"
        self.foi_coordinates = [512,512]
        self.image_dimensions = [1023, 1023]

        timer_period = 2  # seconds
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

        time.sleep(10)
        self.get_logger().info("Eye node initialised")
        self.get_logger().info(f"Simulation mode: {self.sim_mode}")

    def foi_coordinates_callback(self, msg):
        """
        Update the coordinates of the face of interest,
        so that the eyes can point towards the face.
        """
        self.get_logger().info("In foi_coordinates_callback")
        self.foi_coordinates = [msg.x, msg.y]
        self.image_dimensions = [msg.image_x, msg.image_y]

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        # Display the message on the console
        self.get_logger().info("In system_state_callback")

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
        self.get_logger().info("In timer_callback")
        if self.eye_state == "default":
            # Just look around randomly
            self.eyes.default_movement()
        if self.eye_state == "straight":
            # Look straight ahead (while painting)
            self.eyes.straight()
        if self.eye_state == "focus":
            # Focus on the foi
            foi_x = self.map_value(self.foi_coordinates[0],0,self.image_dimensions[0],1023,0)
            foi_y = self.map_value(self.foi_coordinates[1],0,self.image_dimensions[1],1023,0)
            self.eyes.focus(foi_x, foi_y)

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
    