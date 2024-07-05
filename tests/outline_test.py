from arm_outline import Outline
from arm_movements import ArmMovements
from subprocess import PIPE, run
import cv2

outliner = Outline()
movements = ArmMovements()

# Load camera video feed.   
camera_name = "FHD Camera"
command = ['ffmpeg','-f', 'avfoundation','-list_devices','true','-i','""']
result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True)
cam_id = 0

# # Ensure we are using the right camera.
# for item in result.stderr.splitlines():
#     if (camera_name in item) and ("Microphone" not in item):
#         cam_id = int(item.split("[")[2].split(']')[0])
# self.logger.info("FHD Camera ID is: %s", cam_id)

cam = cv2.VideoCapture(cam_id)
#cam.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
print("Have turned on camera now")

print("Taking picture")
for i in range(10):
    ret, frame = cam.read() 

cv2.imshow('Image Window', frame)
cv2.waitKey(0) 
cv2.destroyAllWindows()

frame_copy = frame.copy()

coordinates, image_x, image_y = outliner.find_contours_coordinates(frame)
#outliner.find_facial_features(frame, frame_copy)

movements.paint_image(coordinates, image_x, image_y)
