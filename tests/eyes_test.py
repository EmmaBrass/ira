import serial
import random
import time

from ira_common.eye_control import EyeControl


eyes_port = '/dev/ttyACM0'
eyes = EyeControl(com_port=eyes_port)

eye_state = "swirl"

if eye_state == "default":
    # Just look around randomly
    eyes.default_movement()
if eye_state == "swirl":
    eyes.swirl()

