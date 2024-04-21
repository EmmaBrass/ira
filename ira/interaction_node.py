# Subscribe to faces_info topic
# Keep track of what system state we are in
# Publish state to system_state topic

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray

import cv2, math, time, logging, pickle
import numpy as np
import face_recognition
from face import Face

from interaction_state_machine import InterationStateMachine

# TODO add in state machine for this node! Using transitions library.
# TODO may need different callback groups for everything going on here? https://docs.ros.org/en/humble/How-To-Guides/Using-callback-groups.html

class InteractionNode(Node):

    def __init__(self):
        super().__init__('interaction_node')

        self.logger = logging.getLogger("main_logger")

        # Load known face objects from .dat file
        with open('dataset_faces.dat', 'rb') as f:
            self.all_faces = pickle.load(f)

        self.state_machine = InterationStateMachine()
        self.face_of_interest = None
        self.scan_counter = 0
        
        # Initialise publishers
        self.system_state_publisher = self.create_publisher(String, 'system_state', 10) #TODO create a custom message type for this?
        self.cropped_face_publisher = self.create_publisher(String, 'face_of_interest', 10)
        self.arm_commands_publisher = self.create_publisher() #TODO arm message type
        self.eye_data_publisher = self.create_publisher(String, 'eye_data', 10) #TODO maybe a seperate timercallback (with a shorter time period)(with a different callback group) for sending lots of eye data v quickly.
        self.gpt_data_publisher = self.create_publisher(String, 'gpt_data', 10)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback) # Publishing happens within the timer_callback

        # Initialise subscribers
        self.latest_image_subscription = self.create_subscription(
            Image,
            'latest_image', 
            self.latest_image_callback, 
            10
        )
        self.arm_complete_subscription = self.create_subscription(
            bool,
            'arm_complete', 
            self.arm_complete_callback, 
            10
        )
        self.latest_image_subscription 
        self.arm_complete_subscription # prevent unused variable warning

    def latest_image_callback(self, msg):
        """
        Callback function for receving image from camera
        """
        # Display the message on the console
        self.logger().debug("Inside lastest_image_callback")
        self.latest_image = msg.data

    def arm_complete_subscription(self, msg):
        """
        Callback function for receving completion status of an arm command.
        """
        # Display the message on the console
        self.logger().debug("Inside arm_completion_callback")
        # TODO 

    def timer_callback(self):
        """
        Checks the state of the state machine every timer_period,
        and based on the state, publishes and/or runs a function to move
        it on to the next state.
        Publishes to system_state topic.
        and publishes to other topics if in the appropriate state.
        """
        self.logger().info('In timer_callback')
        self.logger().info('Current system state: %s.', self.state_machine.state)

        self.system_state_publisher.publish(self.state_machine.state)
        self.eye_data_publisher.publish()#TODO eye data message type TODO will vary with system state
        self.gpt_data_publisher.publish()#TODO gpt data message type TODO will vary with system state

        # Run a function for the given state, to tick the state machine forwards by one.
        if self.state_machine.state == 'scanning':
            self.scanning()
        if self.state_machine.state == 'found_unknown':
            self.found_unknown()


        if self.system_state == "painting" and self.cropped_image_sent == False:
            for i in range(5):
                self.cropped_face_publisher.publish(self.cropped_face_image) # TODO make cropped face image TODO custom mssage w/ seq_id?
            self.cropped_image_sent = True       
        if self.system_state == "finished_painting":
            self.cropped_image_sent == False

    def found_unknown(self):
        """
        Method for the 'found_unknown' state of the state machine.
        Check if they are close enough.
        """
        # TODO publish to arm to go STILL when a face is found initially.
        # (then it will centre the face in the interaction state)
        frame_face_objects = self.find_faces()

        if self.foi not in frame_face_objects:
            # FOI has disappeared... back to scanning
            self.state_machine.to_scanning()
        else:
            if self.foi.close == True:
                # Face is present and close enough
                self.state_machine.to_interaction_unknown()
            else:
                if self.scan_counter > 2:
                    # Face is too far away and have scanned >2 times already
                    self.scan_counter = 0
                    self.state_machine.to_too_far()
                else:
                    # Face is too far away and have scanned <3 times
                    self.scan_counter += 1
                    self.state_machine.to_scanning()

    def scanning(self):
        """
        Method for the 'scanning' state of the state machine.
        """
        frame_face_objects = self.find_faces()

        known_list = [face.known for face in frame_face_objects]
        size_list = [face.size for face in frame_face_objects]

        if False in known_list:
            max_size = -1
            max_index = -1
            # Choose largest unknown face
            for i, (known, size) in enumerate(zip(known_list, size_list)):
                if not known and size > max_size:
                    max_size = size
                    max_index = i
            self.face_of_interest = frame_face_objects[max_index].encoding
            # An unknown face found
            self.state_machine.to_found_unknown()
        else:
            max_size = -1
            max_index = -1
            # Choose largest known face
            for i, (known, size) in enumerate(zip(known_list, size_list)):
                if known and size > max_size:
                    max_size = size
                    max_index = i
            self.face_of_interest = frame_face_objects[max_index].encoding
            # A known face found
            self.state_machine.to_found_known()


    def find_faces(self):
        # Take input image from the node and look for faces
        """
        General method to find all faces in the camera image.
        Uses the latest_image delivered to the node.
        Checks if they have an existing Face object and creates one if not.
        Updates all properties of the Face object (known, close, centred, etc.)
        Updates the FOI if present in the image.

        :returns frame_face_objects: list of Face objects foun in the frame.
        """

        image = self.latest_image
        frame_face_objects = []

        # Resize frame of video to 1/4 size for 
        # faster face recognition processing.
        small_frame = cv2.resize(image, (0, 0), fx=0.25, fy=0.25)

        # Convert the image from BGR color (which OpenCV uses) to RGB color 
        # (which face_recognition uses).
        # rgb_small_frame = small_frame[:, :, ::-1]
        rgb_small_frame = np.ascontiguousarray(small_frame[:, :, ::-1])
        
        # Find all the faces and face encodings in the current frame of video.
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(
            rgb_small_frame, 
            face_locations
        )

        if not face_locations:
            # No faces found - continue scanning
            self.state_machine.to_scanning()
        else:
            # For all faces in the frame, update or make a Face object and 
            # add it to frame_face_objects
            for (top, right, bottom, left), encoding in zip(face_locations, face_encodings):
                # Mutiple frame size back up
                top *= 4
                right *= 4
                bottom *= 4
                left *= 4
                size = (right-left)*(bottom-top)
                location = [top, right, bottom, left]
                # Don't make a new face if one already exists... 
                # just update attributes of existing object.
                matches = face_recognition.compare_faces((face.encoding for face in self.all_faces), encoding)
                # If multiple matches found in self.all_faces, just use the first one.
                if True in matches:
                    # Update the properties of the face object
                    first_match_idx = matches.index(True)
                    face = self.all_faces[first_match_idx]
                    face.location = location
                    face.size = size
                    face.encoding = encoding
                    face.known = True
                    face.centred = self.check_if_centred(image.shape[1], image.shape[0], face.location)
                    face.close = self.check_if_close(image.shape[1], image.shape[0], face.size)
                    frame_face_objects.append(face)
                else: 
                    # Create new face object and fill in its properties
                    new_face = Face(location, size, encoding)
                    new_face.centred = self.check_if_centred(image.shape[1], image.shape[0], new_face.location)
                    new_face.close = self.check_if_close(image.shape[1], image.shape[0], new_face.size)
                    new_face.known = False
                    frame_face_objects.append(new_face)
                    self.all_faces.append(new_face)
                self.remember_faces()

        # Update the FOI if present in the image
        for face in frame_face_objects:
            if self.foi != None and face.encoding == self.foi.encoding:
                self.foi = face
                break

        return frame_face_objects

    def check_if_centred(self, image_width, image_height, location):
        """
        Checks if the centre of the face is close enough 
        to the centre of the image.
        Get face within central 1/3 of the image width
        and central 2/4 of the image height.

        :param image_width: width (in pixels) of the whole image.
        :param image_height: height (in pixels) of the whole image.
        :param location: location of the face as [top, right, bottom, left]
        :returns: True if centred, else a list of bools for where the face is, 
        in order [top,right,bottom,left]
        """
        top = location[0]
        right = location[1]
        bottom = location[2]
        left = location[3]
        face_centre_x = (right+left)/2
        face_centre_y = (top+bottom)/2

        image_x_third_left = image_width/3
        image_x_third_right = (image_width/3)*2

        image_y_half_top = image_height/4 # TODO check top & bottom right way around
        image_y_half_bottom = (image_height/4)*3

        top = False
        right = False
        bottom = False
        left = False
        if face_centre_x < image_x_third_left:
            left = True
        if face_centre_x > image_x_third_right:
            right = True
        if face_centre_y < image_y_half_top:
            top = True
        if face_centre_y > image_y_half_bottom:
            bottom = True

        if [top,right,bottom,left].all() == False:
            return True
        else:
            return [top,right,bottom,left]

    def check_if_close(self, image_width, image_height, size):
        """
        Checks if the person is close enough to the camera by comparing the 
        size of their face to the size of the whole image.

        :param image_width: the width of the whole image.
        :param image_height: the height of the whole image.
        :param size: the size of the box enclosing the face.
        :returns: True if face is larger/'closer' than the threshold, False if not.
        """
        whole_image_size = image_width*image_height

        if size >= (whole_image_size/6):
            return True
        else:
            return False

    def remember_faces(self):
        """
        Save the updated face objects list into the pickle .dat file.
        """
        with open('dataset_faces.dat', 'wb') as f:
            pickle.dump(self.all_faces, f)

    




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