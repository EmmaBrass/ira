# Script for testing

# from log_config import setup_logger
import cv2
from outline_svg import SVG
import time


# Facial recog to start with

# Create a logger for the current module
# logger = setup_logger("main_logger")
#facial_recog = FacialRecog()
# Create camera object
# Get a reference to webcam #0 (the default one)
video_capture = cv2.VideoCapture(0)
while not video_capture.isOpened():
    print('no')
time.sleep(2)

outline = SVG()

# logger.info("A test log!")


# Grab a single frame of video
for i in range(20):
    ret, frame = video_capture.read()

# Saving the image 
cv2.imwrite("images/test.png", frame) 


# # Release handle to the webcam
video_capture.release()
cv2.destroyAllWindows()
print("HERE")
outline.find_outline(frame)