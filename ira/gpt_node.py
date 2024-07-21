

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from ira.general_gpt import GPT

from ira_interfaces.msg import GptComplete
from ira_interfaces.msg import SystemState

import time

class GPTNode(Node):
        
    def __init__(self):
        super().__init__('gpt_node')
        self.declare_parameter('sim', False)
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value

        self.gpt = GPT()
        self.state_seq = 0

        # Initialise publishers
        self.gpt_complete_publisher = self.create_publisher(GptComplete, 'gpt_complete', 10) #TODO create a custom message type for this?

        # Initialise subscribers
        self.latest_image_subscription = self.create_subscription(
            Image,
            'latest_image', 
            self.latest_image_callback, 
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

    def latest_image_callback(self, msg):
        """
        Callback function for receving image from camera.
        Loads it in as the latest image.
        """
        # TODO maybe this needs to be just the cropped FOI instead ?? To comment on the person in particular! 
        # Display the message on the console
        self.get_logger().info("In lastest_image_callback")
        self.latest_image = msg #TOOD format... message is just an image

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        self.get_logger().info("In system_state_callback")
        self.get_logger().info(f"{msg.seq=}")
        self.get_logger().info(f"{self.state_seq=}")

        if msg.seq > self.state_seq:
            self.get_logger().info("HERE GPT")
            self.state_seq = msg.seq
            if msg.state == 'scanning':
                self.get_logger().info("GPT received scanning state")
                self.gpt_complete(msg.seq)
            elif msg.state == 'found_noone':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <found_noone>") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'found_unknown':
                self.gpt_complete(msg.seq)
            elif msg.state == 'found_known':
                self.gpt_complete(msg.seq)
            elif msg.state == 'say_painted_recently':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <say_painted_recently>") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'too_far':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <too_far>") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'interaction_unknown':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_unknown>.  The image path : self.latest_image") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'interaction_known':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_known>.  The image path : self.latest_image") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'interaction_known_recent':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_known_recent>.  The image path : self.latest_image") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'disappeared':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <disappeared>") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'interaction_returned':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_returned>") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'gone':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <gone>") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'painting':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <painting>") # TODO is image in right format?
                self.gpt_complete(msg.seq)
            elif msg.state == 'completed':
                self.gpt.add_user_message_and_get_response_and_speak("The command is: <completed>") # TODO is image in right format?
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
    