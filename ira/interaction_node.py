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

        # Load known face objects from .dat file

        # Get the path to the data file
        package_share_directory = get_package_share_directory('ira')
        self.data_file_path = os.path.join(package_share_directory, 'resource', 'dataset_faces.dat')
        
        # Load the pickled data file
        with open(self.data_file_path, 'rb') as file:
            self.all_faces = pickle.load(file)
        
        self.get_logger().info(f"Data from file: {self.all_faces}")

        self.state_machine = InterationStateMachine()
        self.foi = None # face of interest
        self.scan_counter = 0
        self.noone_counter = 0

        self.latest_image = None
        self.cropped_image = None

        self.seq = 0
        self.prev_gpt_complete = [False]
        self.prev_arm_complete = [False]
        
        # Initialise publishers
        self.system_state_publisher = self.create_publisher(SystemState, 'system_state', 10)
        self.cropped_face_publisher = self.create_publisher(Image, 'cropped_face', 10)
        self.foi_coordinates_publisher = self.create_publisher(FoiCoord, 'foi_coordinates', 10) #TODO this isn't published anywhere yet

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
        if self.state_machine.state == 'painting':
            for i in range(5):
                self.cropped_face_publisher.publish(self.cropped_image) 
            time.sleep(2)
        # Publish the current state, to start arm and GPT processes.
        self.publish_state(str(self.state_machine.state))
        
        # If arm and GPT processes completed, tick the state forward.
        if self.prev_arm_complete[self.seq] == True and self.prev_gpt_complete[self.seq] == True:
            self.seq += 1 
            self.prev_arm_complete.append(False)
            self.prev_gpt_complete.append(False)
            self.get_logger().info(f'Current self.seq: {self.seq}')
            # Run a function for the given state, to tick the state machine forwards by one.
            if self.state_machine.state == 'scanning':
                self.scanning()
            elif self.state_machine.state == 'found_noone':
                self.found_noone()
            elif self.state_machine.state == 'found_unknown':
                self.found_unknown()
            elif self.state_machine.state == 'found_known':
                self.found_known()
            elif self.state_machine.state == 'say_painted_recently':
                self.say_painted_recently()
            elif self.state_machine.state == 'too_far':
                self.too_far()
            elif self.state_machine.state == 'painting':
                self.painting()
            elif self.state_machine.state == 'completed':
                self.completed()
            # Publish coordinates of foi if there is one
            if self.foi != None:
                top = self.foi.location[0]
                right = self.foi.location[1]
                bottom = self.foi.location[2]
                left = self.foi.location[3]
                face_centre_x = (right+left)/2
                face_centre_y = (top+bottom)/2
                # Send message
                msg = FoiCoord()
                msg.x = int(face_centre_x)
                msg.y = int(face_centre_y)
                msg.image_x = int(self.latest_image.shape[1])
                msg.image_y = int(self.latest_image.shape[0])
                self.foi_coordinates_publisher.publish(msg)


    def scanning(self):
        """
        Method for the 'scanning' state of the state machine.
        """
        self.get_logger().info(f'In scanning method')

        frame_face_objects = self.find_faces()

        known_list = [face.known for face in frame_face_objects]
        size_list = [face.size for face in frame_face_objects]

        if len(frame_face_objects) > 0 and self.scan_counter>1:
            self.noone_counter = 0
            # If there is an unknown face present
            if False in known_list:
                self.get_logger().info('Unknown face present')
                max_size = -1
                max_index = -1
                # Choose largest unknown face
                for i, (known, size) in enumerate(zip(known_list, size_list)):
                    if not known and size > max_size:
                        max_size = size
                        max_index = i
                self.foi= frame_face_objects[max_index]
                self.state_machine.to_found_unknown()
            # If there is no unknown face present
            else:
                self.get_logger().info('No unknown face present')
                max_size = -1
                max_index = -1
                # Choose largest known face
                for i, (known, size) in enumerate(zip(known_list, size_list)):
                    if known and size > max_size:
                        max_size = size
                        max_index = i
                self.foi = frame_face_objects[max_index]
                # A known face found
                self.state_machine.to_found_known()
        elif len(frame_face_objects) > 0 and self.scan_counter<=1:
            self.scan_counter += 1
            self.state_machine.to_scanning()
        else:
            self.foi = None
            if self.noone_counter > 1:
                self.noone_counter = 0
                self.state_machine.to_found_noone()
            else:
                self.noone_counter += 1
                self.state_machine.to_scanning()

    def found_noone(self):
        """
        Method for if noone can be seen by the robot.
        Move back to scanning.
        """
        self.state_machine.to_scanning()

    def found_unknown(self):
        """
        Method for the 'found_unknown' state of the state machine.
        Check if they are close enough.
        """
        frame_face_objects = self.find_faces()

        if self.foi not in frame_face_objects:
            # FOI has disappeared... back to scanning
            self.scan_counter = 0
            self.noone_counter += 1
            self.state_machine.to_scanning()
        else:
            if self.foi.close == True:
                # Face is present and close 
                self.scan_counter = 0
                # Find the foi in self.all_faces
                # Update the object with the interaction
                for face in self.all_faces:
                    if np.array_equal(face.encoding, self.foi.encoding):
                        face.add_interaction(datetime.now(), "found_unknown")
                # Paint them!
                self.state_machine.to_painting()
            else:
                if self.scan_counter > 3:
                    # Face is too far away and have scanned >3 times already
                    self.scan_counter = 0
                    self.state_machine.to_too_far()
                else:
                    # Face is too far away and have scanned <2 times
                    self.scan_counter += 1
                    self.noone_counter += 1
                    self.state_machine.to_scanning()

    def found_known(self):
        """
        Method for the 'found_known' state.
        Check if painted recently, if close enough to paint,
        painted at some point before.
        """
        frame_face_objects = self.find_faces()

        if self.foi not in frame_face_objects:
            # FOI has disappeared... back to scanning
            self.scan_counter = 0
            self.noone_counter += 1
            self.state_machine.to_scanning()
        else:
            if self.foi.close == True:
                # Face is present and close enough
                self.scan_counter = 0
                # Find the foi in self.all_faces
                # Update the object with the interaction
                for face in self.all_faces:
                    if np.array_equal(face.encoding, self.foi.encoding):
                        face.add_interaction(datetime.now(), "found_known")
                last_painting =  datetime(1900, 1, 1)
                last_interaction = None
                if len(self.foi.past_interactions) > 0:
                    last_interaction = self.foi.past_interactions[-1]
                    for interaction in self.foi.past_interactions:
                        if interaction.outcome == 'painting' and interaction.date_time > last_painting:
                            last_painting = interaction.date_time
                duration = datetime.now() - last_painting
                duration_in_s = duration.total_seconds()        
                if duration_in_s < 10 and last_interaction != 'gone': #TODO edit the duration_is_s to be whatever needed - add to constants.py
                    # Painted too recently to paint again
                    self.state_machine.to_say_painted_recently()
                else:
                    # Paint them!
                    self.state_machine.to_painting()
            else:
                if self.scan_counter > 3:
                    # Face is too far away and have scanned >3 times already
                    self.scan_counter = 0
                    self.state_machine.to_too_far()
                else:
                    # Face is too far away and have scanned <1 times
                    self.scan_counter += 1
                    self.noone_counter += 1
                    self.state_machine.to_scanning()

    def say_painted_recently(self):
        """ 
        Method for person that has been painted very recently,
        and hence won't be painted again just yet.
        Go back to scanning.
        """
        self.state_machine.to_scanning()

    def too_far(self):
        """
        Method for when a person is too far away.
        Go back to scanning.
        """
        self.state_machine.to_scanning()

    def painting(self):
        """
        Method for painting the face.
        Go to completed.
        """
        # Find the foi in self.all_faces
        # Update the object with the interaction
        for face in self.all_faces:
            if np.array_equal(face.encoding, self.foi.encoding):
                face.add_interaction(datetime.now(), "painting")
        self.state_machine.to_completed()

    def completed(self):
        """
        Method for completed.
        Go back to scanning for the face.
        """
        self.state_machine.to_scanning()

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
        self.get_logger().info(f'In find_faces method')
        frame_face_objects = []

        if self.latest_image is not None:

            image = self.latest_image

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
                self.get_logger().info(f'No faces found!')
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
                    matches = face_recognition.compare_faces([face.encoding for face in self.all_faces], encoding)
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

            # Update the FOI if there is one and it is present in the image
            # Publish the foi coordinates
            for face in frame_face_objects:
                if self.foi != None and np.array_equal(face.encoding, self.foi.encoding):
                    self.foi = face
                    top = face.location[0]
                    right = face.location[1]
                    bottom = face.location[2]
                    left = face.location[3]
                    height = bottom-top
                    width = right-left
                    cropped_top = int(top-(height/3))
                    cropped_bottom = int(bottom+(height/4))
                    cropped_left = int(left-(width/4))
                    cropped_right = int(right+(width/4))
                    if cropped_top < 0:
                        cropped_top = 0
                    if cropped_bottom > image.shape[0]:
                        cropped_bottom = image.shape[0]
                    if cropped_left < 0:
                        cropped_left = 0
                    if cropped_right > image.shape[1]:
                        cropped_right = image.shape[1]
                    self.cropped_image_array = image[cropped_top:cropped_bottom, cropped_left:cropped_right]
                    self.cropped_image = bridge.cv2_to_imgmsg(self.cropped_image_array, encoding="bgr8")
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

        if all(value == False for value in [top, right, bottom, left]):
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

        if size >= (whole_image_size/15):
            return True
        else:
            return False

    def remember_faces(self):
        """
        Save the updated face objects list into the pickle .dat file.
        """
        with open(self.data_file_path, 'wb') as f: #TODO check if this file exists before saving?
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