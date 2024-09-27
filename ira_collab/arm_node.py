

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from ira_common.arm_movements import ArmMovements
from ira_common.arm_outline import Outline

from ira_interfaces.msg import ArmComplete
from ira_interfaces.msg import SystemState
from ira_interfaces.msg import CanvasImage

import time


class ArmNode(Node):
        
    def __init__(self):
        super().__init__('arm_node')
        self.declare_parameter('sim', False)
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.movements = ArmMovements()
        self.outline = Outline()
        self.before_canvas_image = None # before human mark
        self.after_canvas_image = None # after humam mark
        self.state_seq = -1

        # Initialise publishers
        self.arm_complete_publisher = self.create_publisher(ArmComplete, 'arm_complete', 10) #TODO create a custom message type for this?

        # Initialise subscribers
        self.canvas_image_subscription = self.create_subscription(
            CanvasImage,
            'canvas_image',
            self.canvas_image_callback, 
            10
        )
        self.system_state_subscription = self.create_subscription(
            SystemState,
            'system_state',
            self.system_state_callback, 
            10
        )

        time.sleep(3)
        self.get_logger().info("Arm node initialised")
        self.get_logger().info(f"Simulation mode: {self.sim_mode}")


    def canvas_image_callback(self, msg):
        """
        Save the most recent cropped foi image.
        """
        self.get_logger().info("In canvas_image_callback")
        if msg.type == "before":
            self.before_canvas_image = self.bridge.imgmsg_to_cv2(msg.image)
        if msg.type == "after":
            self.after_canvas_image = self.bridge.imgmsg_to_cv2(msg.image)

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        if msg.seq > self.state_seq:
            self.state_seq = msg.seq
            if self.sim_mode:
                self.arm_complete(msg.seq)
            else:
                if msg.state == 'startup':
                    self.arm_complete(msg.seq)
                if msg.state == 'your_turn':
                    self.movements.lift_up()
                    self.arm_complete(msg.seq)
                if msg.state == 'looking':
                    self.movements.look_at_canvas()
                    self.arm_complete(msg.seq)
                if msg.state == 'comment':
                    self.movements.look_forward()
                    self.arm_complete(msg.seq)
                if msg.state == 'my_turn':
                    self.movements.paint_abstract_mark()
                    self.arm_complete(msg.seq)
                if msg.state == 'ask_done':
                    self.arm_complete(msg.seq)
                if msg.state == 'completed':
                    self.movements.acknowledge()
                    self.arm_complete(msg.seq)


    def arm_complete(self, seq):
        self.get_logger().info("In arm_complete")
        msg = ArmComplete()
        msg.seq = seq
        msg.complete = True
        for i in range(5):
            self.arm_complete_publisher.publish(msg)

        

def main(args=None):
    rclpy.init(args=args)
    arm_node = ArmNode()
    rclpy.spin(arm_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_node.destroy_node()
    rclpy.shutdown()