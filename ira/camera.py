# Setup and initialisation for the camera
import logging
import cv2
from subprocess import PIPE, run

class Camera():

    def __init__(self, port_num) -> None:
        self.port_num = port_num
        self.start_up()
    
    def start_up(self):
        # Load camera video feed.   
        camera_name = "FHD Camera"
        command = ['ffmpeg','-f', 'avfoundation','-list_devices','true','-i','""']
        result = run(command, stdout=PIPE, stderr=PIPE, universal_newlines=True)
        cam_id = self.port_num

        # # Ensure we are using the right camera.
        # for item in result.stderr.splitlines():
        #     if (camera_name in item) and ("Microphone" not in item):
        #         cam_id = int(item.split("[")[2].split(']')[0])
        # self.logger.info("FHD Camera ID is: %s", cam_id)

        self.cam = cv2.VideoCapture(cam_id)
        #cam.set(cv2.CAP_PROP_AUTOFOCUS, 0) 
        print("Have turned on camera now")

    def read(self):
        for i in range(10):
            ret, frame = self.cam.read() 

        cv2.imshow('Image Window', frame)
        cv2.waitKey(0) 
        cv2.destroyAllWindows()

        return frame

    def release(self):
        return self.cam.release()
