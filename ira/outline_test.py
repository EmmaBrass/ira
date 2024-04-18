# Script for testing

from facial_recog import FacialRecog
from camera import Camera
from log_config import setup_logger
import cv2
import face_recognition
import numpy as np
from outline_svg import SVG


# Facial recog to start with

# Create a logger for the current module
logger = setup_logger("main_logger")
facial_recog = FacialRecog()
# Create camera object
# Get a reference to webcam #0 (the default one)
video_capture = cv2.VideoCapture(0)

outline = SVG()

logger.info("A test log!")

# # Load a sample picture and learn how to recognize it.
# emma_image = face_recognition.load_image_file("obama.jpg")
# emma_face_encoding = face_recognition.face_encodings(emma_image)[0]

# # Create arrays of known face encodings and their names
# known_face_encodings = [
#     emma_face_encoding
# ]
# known_face_names = [
#     "Emma Brass"
# ]

# # Initialize some variables
# face_locations = []
# face_encodings = []
# face_names = []
# process_this_frame = True

# face_image = False
# while type(face_image) == bool:
#     # Grab a single frame of video
#     ret, frame = video_capture.read()

#     # Only process every other frame of video to save time
#     if process_this_frame:
#         face_image = facial_recog.find_face(frame)

#     process_this_frame = not process_this_frame

#     # Hit 'q' on the keyboard to quit!
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# Grab a single frame of video
for i in range(20):
    ret, frame = video_capture.read()

# Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()

outline.find_outline(frame)
