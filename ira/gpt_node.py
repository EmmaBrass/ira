

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

from ira_common.general_gpt import GPT

from ira_interfaces.msg import GptComplete
from ira_interfaces.msg import SystemState

import time, cv2, os

class GPTNode(Node):
        
    def __init__(self):
        super().__init__('gpt_node')
        self.declare_parameter('sim', False)
        self.sim_mode = self.get_parameter('sim').get_parameter_value().bool_value

        self.gpt = GPT()
        self.state_seq = -1

        # Initialise publishers
        self.gpt_complete_publisher = self.create_publisher(GptComplete, 'gpt_complete', 10)

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
        self.latest_image = bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Ensure the 'images' directory exists one level up
        parent_dir = os.path.abspath(os.path.join(os.getcwd(), os.pardir))
        image_dir = os.path.join(parent_dir, "images")
        if not os.path.exists(image_dir):
            self.get_logger().info(f"Directory {image_dir} does not exist. Creating it.")
            try:
                os.makedirs(image_dir)
            except Exception as e:
                self.get_logger().error(f"Failed to create directory {image_dir}: {str(e)}")
                return

        # Save the image to the specified path
        image_path = os.path.join(image_dir, "latest_image.png")
        try:
            cv2.imwrite(image_path, self.latest_image)
        except Exception as e:
            self.get_logger().error(f"Failed to save image: {str(e)}")

    def system_state_callback(self, msg):
        """
        Callback function for the system state.
        """
        if msg.seq > self.state_seq:
            self.state_seq = msg.seq
            if msg.state == 'scanning':
                self.get_logger().info("GPT received scanning state")
                self.gpt_complete(msg.seq)
            elif msg.state == 'found_noone':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <found_noone>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'found_unknown':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <found>")
                self.gpt_complete(msg.seq)
            elif msg.state == 'found_known':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <found>")
                self.gpt_complete(msg.seq)
            elif msg.state == 'say_painted_recently':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <say_painted_recently>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'too_far':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <too_far>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'interaction_unknown':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_unknown>.  The image path: /home/emma/ira_ws/src/ira/ira/images/latest_image.png")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'interaction_known':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_known>.  The image path: /home/emma/ira_ws/src/ira/ira/images/latest_image.png")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'interaction_known_recent':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_known_recent>.  The image path: /home/emma/ira_ws/src/ira/ira/images/latest_image.png")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'disappeared':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <disappeared>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'interaction_returned':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <interaction_returned>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'gone':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <gone>")
                self.get_logger().info(response)
                self.gpt_complete(msg.seq)
            elif msg.state == 'painting':
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <painting>")
                self.get_logger().info(response)
                time.sleep(30)
                response = self.gpt.add_user_message_and_get_response_and_speak("The command is: <continue_painting>")
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
    