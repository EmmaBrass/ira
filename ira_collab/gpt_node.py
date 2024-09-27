

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

from ira_common.general_gpt import GPT

from ira_interfaces.msg import GptComplete
from ira_interfaces.msg import SystemState
from ira_interfaces.msg import CanvasImage

import time, cv2, os

class GPTNode(Node):
        
    def __init__(self):
        super().__init__('gpt_node')
        self.declare_parameter('sim', False)
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value

        self.gpt = GPT()
        self.state_seq = -1
        
        self.canvas_image = None

        # Initialise publishers
        self.gpt_complete_publisher = self.create_publisher(GptComplete, 'gpt_complete', 10)

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

        time.sleep(10)
        self.get_logger().info("GPT node initialised")
        self.get_logger().info(f"Simulation mode: {self.sim_mode}")

    def canvas_image_callback(self, msg):
        """
        Save the most recent cropped foi image.
        """
        self.get_logger().info("In canvas_image_callback")
        self.canvas_image = self.bridge.imgmsg_to_cv2(msg.image)

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        if msg.seq > self.state_seq:
            self.state_seq = msg.seq
            if msg.state == 'startup':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <startup>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'your_turn':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <your_turn>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'looking':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <looking>")
                self.gpt_complete(msg.seq)
            elif msg.state == 'comment':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <comment>")
                self.gpt_complete(msg.seq)
            elif msg.state == 'my_turn':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <my_turn>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
                time.sleep(30)
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <still_my_turn>") #TODO way to check this rather than just a timer...
                # Like don't work about the seq for painting, just check some timer or an extra counter or something so we can come back here again.  
                # Will be useful for the normal IRa project as well whilst still painting.
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'ask_done':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <ask_done>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'completed':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <completed>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            else:
                self.gpt_complete(msg.seq)

    def gpt_complete(self, seq):
        self.get_logger().info("In gpt_complete")
        msg = GptComplete()
        msg.seq = seq
        msg.complete = True
        for i in range(5):
            self.gpt_complete_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    gpt_node = GPTNode()

    rclpy.spin(gpt_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gpt_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    