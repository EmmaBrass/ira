

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import logging

from arm_movements import ArmMovements

class ArmNode(Node):
        
    def __init__(self):
        super().__init__('gpt_node')

        self.logger = logging.getLogger("main_logger")

        self.bridge = CvBridge()
        self.movements = ArmMovements()
        self.cropped_face = None
        self.state_seq = 0
        self.arm_state = "scan"

        # Initialise publishers
        self.arm_complete_publisher = self.create_publisher(String, 'arm_complete', 10) #TODO create a custom message type for this?

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
            String,
            'system_state',
            self.system_state_callback, 
            10
        )

        #TODO subscribe to a topic with the cropped face image;
        # on this node, do the processing for turning that into and outline and then 
        # into a path for the robot arm to follow.

    def cropped_face_callback(self, msg):
        """
        Save the most recent cropped foi image.
        """
        rclpy.loginfo("Receiving cropped face image")
        self.cropped_face = self.bridge.imgmsg_to_cv2(msg)

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        # Display the message on the console
        self.logger().debug("Inside system_state_callback")

        if msg.seq > self.state_seq:
            self.state_seq = msg.seq
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
                # TODO function for making the outline and then the arm path
                self.movements.paint()
                self.arm_complete(msg.seq)
            if msg.state == 'completed':
                self.arm_state = "scan"
                self.arm_complete(msg.seq)

    def arm_complete(self, seq):
        for i in range(5):
            self.arm_complete_publisher.publish(msg.seq = seq, msg.complete = True)

    def timer_callback(self):
        """
        Depending on the system state, perform different actions.
        """
        self.logger().info('In timer_callback')
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