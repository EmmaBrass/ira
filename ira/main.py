from facial_recog import FacialRecog
from eye_control import EyeControl
from interaction_gpt import GPT
from camera import Camera
from log_config import setup_logger
import cv2
import face_recognition
import numpy as np
from outline_svg import SVG


# Just going to outline the logical flow here for now


# Create a logger for the current module
logger = setup_logger("main_logger")

# Get cam feed.  Move robot about in search for a face.
# Stop robot arm periodically and take a pic to check to faces.


# For either 1 face of many faces, try to get them centred in the image.  
# Get image of face and encode it.  Then move the arm a bit, based on the face location,
# to try and get a more straight-on view.  Take another image, and try to find the same face again.
# Repeat with the same face until that face is central, or until a joint limit reached, or until have tried
# say 5 times.  Have some limit for painting... face must be a certain size.  If too far away, tell them to come closer. 
# If they don't come closer after x tries, then tell them at increasingly large intervals until eventually
# you ignore them.  
# Then, start the painting.
# So the chatpgt can have a few different stages: 
# 1) first sighting of the person
# 2) telling them you are trying to get them centered (tell them to stay still or move?)
# 3) They are centred and you have a good, big picture; tell them you are about to paint them.
# Non-ideal scenario options:
# 4) You are trying to centre them but then they disappear; Oh! That handsome person I was eyeing up has disappeared, what a shame.
# 5) No one there.
# 6) They are too far away 
# 7) The person who was close enough, who you were trying to centre, moves too far away.  
# -> will somehow need to enclose the info that they WERE close enough but then decided to walk away.

# Take cam image.
# If one face in it, start interact session with that one face
# If > one face, choose largest unknown one, or largest known, and start interact session.