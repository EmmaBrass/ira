import logging
import numpy as np
import face_recognition
import cv2
import pickle
from face import Face

class FacialRecog():
    """
    We will image the human always exists behind a plane next to the robot.
    Hence the face is always in one direction, and the robot will only have it's 
    end effector moving around in this plane, with the camera axis 
    perpendicualar to the plane, when it is searching for people.

    If the face is not centred in the plane, the robot will move 
    left/right/up/down to try and get the face centred.

    A picture is taken and processed if the face is central, large enough, and 
    straight-on enough.  (Robot could tell the person to 'come closer' or 'look
    at me' if these criteria are not fufilled.)
    Processed to extract outline and create an .svg frome this.  
    Then the .svg converted into robot commmands via gcode.  

    Could do distance estimation from face size.
    Could do
    
    """
    def __init__(self):
        self.logger = logging.getLogger("main_logger")

        # Load known face objects
        with open('dataset_faces.dat', 'rb') as f:
            self.all_faces = pickle.load(f)
 

    def find_faces(self, image):
        """
        Find all faces in the camera image.
        Return the location of face centre, face size, and encoding of all the faces.

        :param image: the image to find the face in.
        :returns faces_list: A list of face locations and face sizes in format 
        [[location, size, encoding], [location, size, encoding], [location, size, encoding]],
        with location is format [top, right, bottom, left].
        """

        face_objects = []

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

        for (top, right, bottom, left), encoding in zip(face_locations, face_encodings):
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            size = (right-left)*(bottom-top)
            location = [top, right, bottom, left]
            # Don't make a new face if one already exists... 
            # just update attributes of existing object.
            matches = face_recognition.compare_faces((face.encoding for face in self.all_faces), encoding) # move this to interaction node?  put ALL this code in interaction node?
            # If a match was found in self.all_faces, just use the first one.
            if True in matches:
                first_match_idx = matches.index(True)
                face = self.all_faces[first_match_idx]
                face.location = location
                face.size = size
                face.encoding = encoding
                face.known = True
                face.centred = self.check_if_centred(image.shape[1], image.shape[0], face.location)
                face.close = self.check_if_close(image.shape[1], image.shape[0], face.size)
                face_objects.append(face)
            else: 
                # Create new face object
                new_face = Face(location, size, encoding)
                new_face.centred = self.check_if_centred(image.shape[1], image.shape[0], new_face.location)
                new_face.close = self.check_if_close(image.shape[1], image.shape[0], new_face.size)
                new_face.known = False
                face_objects.append(new_face)
                self.all_faces.append(new_face)
            self.remember_faces()

        return face_objects

    def cropped_image(self, image, location):
        """
        Given an image and the location of the face of interest, returns
        a cropped image for converting into an outline for the robot to paint.
        
        :param image: the image containing the face of interest.
        :param location: the [top, right, bottom, left] location of the face 
        of interest in the image.
        :returns cropped_image: the cropped image of the face of interest.
        """

        top = location[0]
        right = location[1]
        bottom = location[2]
        left = location[3]
        height_add = int((bottom-top)/5)
        if bottom+height_add < image.shape[0]:
            bottom += height_add
        else:
            bottom = image.shape[0]
        if top-height_add > 0:
            top -= height_add
        else:
            top = 0
        width_add = int((right-left)/7)
        if right+width_add < image.shape[1]:
            right += width_add
        else:
            right = image.shape[1]
        if left-width_add > 0:
            left -= width_add
        else:
            left = 0
        crop_img = image[top:bottom, left:right]
        if self.logger.level<= 10:
            cv2.imshow("cropped", crop_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        return crop_img

    def check_if_known(self, face_encoding):
        """
        Check if a found face is known or not.
        Note this method currently DOES NOT create a new face object if the 
        face is unknown, hence use with caution.

        :param face_encoding: the encoding to check against known encodings.
        :returns: True if the face is known, otherwise False. 
        """
        # See if the face matches any known face
        matches = face_recognition.compare_faces((face.encoding for face in self.all_faces), face_encoding)

        # If a match was found in known_face_encodings, return True
        # If a match was found in self.all_faces, just use the first one.
        if True in matches:
            first_match_idx = matches.index(True)
            self.all_faces[first_match_idx].known = True
            return True
        else:
            self.remember_faces()
            return False

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
        Save the (updated) face_encodings list into the pickle .dat file.
        """
        with open('dataset_faces.dat', 'wb') as f:
            pickle.dump(self.all_faces, f)

    def find_orientation(self):
        """
        Return if the face if turned left or right, and by how much.
        TODO
        """
        pass


        

