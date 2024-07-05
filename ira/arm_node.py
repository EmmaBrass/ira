

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

from ira.arm_movements import ArmMovements
from ira.arm_outline import Outline

from ira_interfaces.msg import ArmComplete
from ira_interfaces.msg import SystemState


class ArmNode(Node):
        
    def __init__(self):
        super().__init__('arm_node')
        self.declare_parameter('sim', False)
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value

        self.bridge = CvBridge()
        self.movements = ArmMovements()
        self.outline = Outline()
        self.cropped_face = None
        self.state_seq = 0
        self.arm_state = "scan"

        # Initialise publishers
        self.arm_complete_publisher = self.create_publisher(ArmComplete, 'arm_complete', 10) #TODO create a custom message type for this?

        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
        self.latest_image_subscription = self.create_subscription(
            Image,
            'cropped_face',
            self.cropped_face_callback, 
            10
        )
        self.system_state_subscription = self.create_subscription(
            SystemState,
            'system_state',
            self.system_state_callback, 
            10
        )

        self.get_logger().info("Arm node initialised")
        self.get_logger().info(f"Simulation mode: {self.sim_mode}")

        #TODO subscribe to a topic with the cropped face image;
        # on this node, do the processing for turning that into and outline and then 
        # into a path for the robot arm to follow.

    def cropped_face_callback(self, msg):
        """
        Save the most recent cropped foi image.
        """
        self.get_logger().info("In cropped_face_callback")
        self.cropped_face = self.bridge.imgmsg_to_cv2(msg)

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        # Display the message on the console
        self.get_logger().info("In system_state_callback")
        
        if msg.seq > self.state_seq:
            self.state_seq = msg.seq
            if self.sim_mode:
                self.arm_complete(msg.seq)
            else:
                if msg.state == 'scanning':
                    self.arm_state = "scan" # move around at random
                    self.arm_complete(msg.seq)
                if msg.state == 'found_noone':
                    self.arm_state = "stop" # stop moving
                    self.arm_complete(msg.seq)
                if msg.state == 'found_unknown':
                    self.arm_state = "stop"
                    self.arm_complete(msg.seq)
                if msg.state == 'found_known':
                    self.arm_state = "stop"
                    self.arm_complete(msg.seq)
                if msg.state == 'say_painted_recently':
                    self.arm_state = "stop"
                    self.arm_complete(msg.seq)
                if msg.state == 'too_far':
                    self.arm_state = "scan"
                    self.arm_complete(msg.seq)
                if msg.state == 'interaction_unknown':
                    self.arm_state = "stop"
                    self.arm_complete(msg.seq)
                if msg.state == 'interaction_known':
                    self.arm_state = "stop"
                    self.arm_complete(msg.seq)
                if msg.state == 'interaction_known_recent':
                    self.arm_state = "stop"
                    self.arm_complete(msg.seq)
                if msg.state == 'disappeared':
                    self.arm_state = "stop"
                    self.arm_complete(msg.seq)
                if msg.state == 'interaction_returned':
                    self.arm_state = "stop"
                    self.arm_complete(msg.seq)
                if msg.state == 'gone':
                    self.arm_state = "scan"
                    self.arm_complete(msg.seq)
                if msg.state == 'painting':
                    path_points, image_x, image_y = self.outline.find_contours_coordinates(self.cropped_face)
                    self.movements.paint(path_points, image_x, image_y)
                    self.arm_complete(msg.seq)
                if msg.state == 'completed':
                    self.arm_state = "scan"
                    self.arm_complete(msg.seq)

    def arm_complete(self, seq):
        self.get_logger().info("In arm_complete")
        msg = ArmComplete()
        msg.seq = seq
        msg.complete = True
        for i in range(5):
            self.arm_complete_publisher.publish(msg)

    def timer_callback(self):
        """
        Depending on the system state, perform different actions.
        """
        self.get_logger().info('In timer_callback')
        if self.sim_mode:
            self.get_logger().info('Simulated arm motion')
        else:
            if self.arm_state == "scan":
                # Move around slowly and randomly in viewing plane
                self.movements.scan()
            if self.arm_state == "stop":
                # Don't move
                self.movements.stop()
        


def main(args=None):
    rclpy.init(args=args)
    arm_node = ArmNode()
    rclpy.spin(arm_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arm_node.destroy_node()
    rclpy.shutdown()