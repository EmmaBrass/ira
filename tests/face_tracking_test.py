
import cv2
import numpy as np
import face_recognition
from subprocess import PIPE, run
from face import Face

from ament_index_python.packages import get_package_share_directory

# Test that implements the facial_recog and face tracking from interaction_node ?

class FaceRecog():

    def __init__(self) -> None:
        pass
        self.all_faces = []
        self.foi = None

    def find_faces(self, image):
        # Take input image from the node and look for faces
        """
        General method to find all faces in the camera image.
        Uses the latest_image delivered to the node.
        Checks if they have an existing Face object and creates one if not.
        Updates all properties of the Face object (known, close, centred, etc.)
        Updates the FOI if present in the image.

        :returns frame_face_objects: list of Face objects foun in the frame.
        """
        global all_faces

        print(f'In find_faces method')

        frame_face_objects = []

        if image is not None:

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

            img_copy = image

            if not face_locations:
                print('No faces found!')
            else:
                print('A FACE FOUND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
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
                    top_left = (left, top)
                    bottom_right = (right, bottom)
                    cv2.rectangle(img_copy, top_left, bottom_right, (0, 255, 0), 2)
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
                        print(f"face.location: {face.location}")
                        print(f"face.size: {face.size}")
                        print(f"face.encoding: {face.encoding}")
                        print(f"face.known: {face.known}")
                        print(f"face.centred: {face.centred}")
                        print(f"face.close: {face.close}")
                        frame_face_objects.append(face)
                    else: 
                        # Create new face object and fill in its properties
                        new_face = Face(location, size, encoding)
                        new_face.centred = self.check_if_centred(image.shape[1], image.shape[0], new_face.location)
                        new_face.close = self.check_if_close(image.shape[1], image.shape[0], new_face.size)
                        new_face.known = False
                        print(f"new_face.known: {new_face.known}")
                        print(f"new_face.centred: {new_face.centred}")
                        print(f"new_face.close: {new_face.close}")
                        frame_face_objects.append(new_face)
                        self.all_faces.append(new_face)
                    self.remember_faces()

            cv2.imshow('faces', img_copy)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            self.foi = frame_face_objects[0]

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
                    cropped_top = top-int(height/2.8)
                    cropped_bottom = bottom+int(height/8)
                    cropped_left = left-int(width/15)
                    cropped_right = right+int(width/15)
                    if cropped_top < 0:
                        cropped_top = 0
                    if cropped_bottom > image.shape[0]:
                        cropped_bottom = image.shape[0]
                    if cropped_left < 0:
                        cropped_left = 0
                    if cropped_right > image.shape[1]:
                        cropped_right = image.shape[1]
                    self.cropped_image = image[cropped_top:cropped_bottom, cropped_left:cropped_right]
                    cv2.imshow('Cropped Image', self.cropped_image)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()
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

        if size >= (whole_image_size/8):
            return True
        else:
            return False
        
    def remember_faces(self):
        print("Remembering faces...")



facial_recog = FaceRecog()

# Load camera video feed.   
camera_name = "FHD Camera"
command = ['ffmpeg','-f', 'avfoundation','-list_devices','true','-i','""']
result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True)
cam_id = 4

# # Ensure we are using the right camera.
# for item in result.stderr.splitlines():
#     if (camera_name in item) and ("Microphone" not in item):
#         cam_id = int(item.split("[")[2].split(']')[0])
# self.logger.info("FHD Camera ID is: %s", cam_id)

cam = cv2.VideoCapture(cam_id)
#cam.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
print("Have turned on camera now")

while True:
    print("Taking picture")
    for i in range(10):
        ret, frame = cam.read() 

    cv2.imshow('Image Window', frame)
    cv2.waitKey(0) 
    cv2.destroyAllWindows()

    print("Looking for faces")
    facial_recog.find_faces(frame)