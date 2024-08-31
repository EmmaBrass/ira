from ira.arm_outline import Outline
from ira.arm_movements import ArmMovements
from subprocess import PIPE, run
import cv2

outliner = Outline()
# movements = ArmMovements()

# Load camera video feed.   
cam_id = 0
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

#frame = cv2.imread("./images/beard_test.jpeg")

# Make the outline from the original image
coordinates, image_x, image_y = outliner.find_contours_coordinates(frame, False)

# # PAINT the image
# movements.paint_image(coordinates, image_x, image_y)
