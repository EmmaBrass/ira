import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

import cv2, pickle, os, time
from datetime import datetime
import numpy as np
import face_recognition
from ira_common.face import Face
from ira.interaction_state_machine import InterationStateMachine

from ira_interfaces.msg import SystemState
from ira_interfaces.msg import ArmComplete
from ira_interfaces.msg import GptComplete
from ira_interfaces.msg import FoiCoord

from ament_index_python.packages import get_package_share_directory

# TODO may need different callback groups for everything going on here? https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html

class InteractionNode(Node):

    def __init__(self):
        super().__init__('interaction_node')
        
        self.declare_parameter('sim', False)
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value

        # Get the path to the data file
        package_share_directory = get_package_share_directory('ira_collab')
        self.data_file_path = os.path.join(package_share_directory, 'resource', 'dataset_faces.dat')
        
        self.state_machine = InterationStateMachine()

        self.latest_image = None
        self.cropped_image = None

        self.num_turns = 0

        self.seq = 0
        self.prev_gpt_complete = [False]
        self.prev_arm_complete = [False]
        
        # Initialise publishers
        self.system_state_publisher = self.create_publisher(SystemState, 'system_state', 10)
        self.cropped_face_publisher = self.create_publisher(Image, 'cropped_face', 10)

        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
        self.latest_image_subscription = self.create_subscription(
            Image,
            'latest_image', 
            self.latest_image_callback, 
            10
        )
        self.arm_complete_subscription = self.create_subscription(
            ArmComplete,
            'arm_complete', 
            self.arm_complete_callback, 
            10
        )
        self.gpt_complete_subscription = self.create_subscription(
            GptComplete,
            'gpt_complete', 
            self.gpt_complete_callback, 
            10
        )
        # Prevent unused variable warnings
        self.latest_image_subscription 
        self.arm_complete_subscription 
        self.gpt_complete_subscription 

        time.sleep(8)
        self.get_logger().info("Interaction node initialised")
        self.get_logger().info(f"Simulation mode: {self.sim_mode}")

    def latest_image_callback(self, msg):
        """
        Callback function for receving image from camera.
        Loads it in as the latest image.
        """
        # Display the message on the console
        self.latest_image = bridge.imgmsg_to_cv2(msg, 'bgr8')

    def arm_complete_callback(self, msg):
        """
        Callback function for receving completion status of an arm command.
        """
        if msg.seq == self.seq:
            if msg.complete == True:
                self.prev_arm_complete[self.seq] = True

    def gpt_complete_callback(self, msg):
        """
        Callback function for receving completion status of a gpt command.
        """
        if msg.seq == self.seq:
            if msg.complete == True:
                self.prev_gpt_complete[self.seq] = True

    def publish_state(self, state: str):
        msg = SystemState()
        msg.seq = self.seq
        msg.state = state
        for i in range(5):
            self.system_state_publisher.publish(msg)

    def timer_callback(self):
        """
        Checks the state of the state machine every timer_period,
        and based on the state, publishes and/or runs a function to move
        it on to the next state.
        Publishes to system_state topic.
        and publishes to other topics if in the appropriate state.
        """
        self.get_logger().info(f'Current system state: {self.state_machine.state}')
        # Publish the current state, to start arm and GPT processes.
        self.publish_state(str(self.state_machine.state))
        
        # If arm and GPT processes completed, tick the state forward.
        if self.prev_arm_complete[self.seq] == True and self.prev_gpt_complete[self.seq] == True:
            self.seq += 1 
            self.prev_arm_complete.append(False)
            self.prev_gpt_complete.append(False)
            self.get_logger().info(f'Current self.seq: {self.seq}')
            # Run a function for the given state, to tick the state machine forwards by one.
            if self.state_machine.state == 'startup':
                self.startup()
            elif self.state_machine.state == 'your_turn':
                self.your_turn()
            elif self.state_machine.state == 'looking':
                self.looking()
            elif self.state_machine.state == 'comment':
                self.comment()
            elif self.state_machine.state == 'my_turn':
                self.my_turn()
            elif self.state_machine.state == 'ask_done':
                self.ask_done()
            elif self.state_machine.state == 'completed':
                self.completed()

    def startup(self):
        """
        Method for the 'startup' state of the state machine.
        Ask the GPT to give startup instructions.
        """
        self.get_logger().info(f'In scanning method')
        self.state_machine.to_your_turn()

    def your_turn(self):
        """
        Method for it being the human's turn.
        Wait for them to press a key to signify they are done.
        """
        self.get_logger().info(f'In your_turn method')
        self.num_turns += 1
        self.state_machine.to_looking()

    def looking(self):
        """
        Method for IRA to look at the canvas, take a pic,
        extract human mark.
        """
        self.get_logger().info(f'In looking method')
        self.state_machine.to_comment()

    def comment(self):
        """
        Method for commenting on the human mark.
        """
        self.get_logger().info(f'In comment method')
        self.state_machine.to_my_turn()

    def my_turn(self):
        """ 
        Method for IRA taking her turn, saying something,
        and taking a reference pic at the end.
        """
        self.get_logger().info(f'In my_turn method')
        if self.num_turns > 7:
            self.state_machine.to_ask_done()
        else:
            self.state_machine.to_your_turn()

    def ask_done(self):
        """
        Method for asking the human if the painting is done.
        """
        self.get_logger().info(f'In ask_done method')
        # TODO get user input for if done or not (keyboard).
        self.state_machine.to_your_turn()
        self.state_machine.to_completed()

    def completed(self):
        """
        Method for a finished piece.
        GPT comment on it.
        """
        self.get_logger().info(f'In completed method')
        self.state_machine.to_startup()

def main(args=None):
    rclpy.init(args=args)

    interaction_node = InteractionNode()

    rclpy.spin(interaction_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    interaction_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()