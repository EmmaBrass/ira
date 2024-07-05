from xarm.wrapper import XArmAPI #TODO install
import cv2

class ArmMovements():
    """
    Arm movement commands for the UFactory XArm 6.
    """
    def __init__(self) -> None:
        self.arm = XArmAPI('192.168.1.200')
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.arm.reset(wait=True)
        # TODO Turn on collision detection
        # TODO set velocity
        # TODO set_mount_direction()

        self.previous_scan_x = None
        self.vertical = True # If true, vertical surface. If False, horizontal
        self.vertical_dist = 40 # Distance in cm of the vertical surace from the robot base
        self.vertical_height = 0 # Distance in cm of the bottom of the vertical surface from the bottom of the robot base
        self.vertical_painting_width = 650 # max reach of robot
        self.vertical_painting_height = 700 # max reach of robot
        self.light = False # False = using a pen/paintbrush instead (hence need x movement away from canvas between contours)

    def scan(self):
        """
        Move to a new random position within the viewing plane.
        """
        pass #TODO

    def stop(self, x, y, z):
        """
        Don't move.
        """
        pass #TODO

    def resize_and_center_image(self, image_x, image_y, target_width, target_height):
        """
        Resizes and centre image dimensions for the drawin space
        """
        image_height, image_width = image_y, image_x
        
        # Calculate the scaling factor
        width_ratio = target_width / image_width
        height_ratio = target_height / image_height
        scaling_factor = min(width_ratio, height_ratio)
        
        # Calculate the new size to maintain aspect ratio
        new_width = int(image_width * scaling_factor)
        new_height = int(image_height * scaling_factor)

        # Calculate offsets
        offset_x = (target_width - new_width) // 2
        offset_y = (target_height - new_height) // 2
        
        return offset_x, offset_y, scaling_factor
    
    def map_coordinates(self, coordinates, offset_x, offset_y, scaling_factor):
        new_coordinates = []
        for contour in coordinates:
            new_contour = []
            for x, y in contour:
                new_x = int(x * scaling_factor) + offset_x
                new_y = int(y * scaling_factor) + offset_y
                new_contour.append((new_x, new_y))
            new_coordinates.append(new_contour)
        return new_coordinates

    def paint_image(self, coordinates, image_x, image_y):
        """
        Paint the image!
        Currently only set for vertical painting.
        Vertical painting space of the xArm6 is about 650mm wide and 700mm high.

        :param coordinates: 2D array of contours coordinates
        :param image_x: x height of original image
        :param image_y: y width of original image
        """

        # TODO order contours so shortest travel distance between them.


        if self.vertical == True:
            # Move to origin point for vertical painting: top-left (0,0)
            self.arm.set_servo_angle(servo_id=1, angle=218.2, speed=50, relative=False, wait=True)
            self.arm.set_servo_angle(servo_id=3, angle=-130.5, speed=20, relative=False, wait=True)
            self.arm.set_servo_angle(servo_id=2, angle=18.8, speed=10, relative=False, wait=True)
            self.arm.set_servo_angle(servo_id=4, angle=-66.2, speed=20, relative=False, wait=True)
            self.arm.set_servo_angle(servo_id=5, angle=43.4, speed=20, relative=False, wait=True)
            self.arm.set_servo_angle(servo_id=6, angle=0, is_radian=False, wait=True)

            # Map the contour coordinates into the vertical drawing space
            offset_x, offset_y, scaling_factor = self.resize_and_center_image(image_x, image_y, self.vertical_painting_width, self.vertical_painting_height)
            mapped_coordinates = self.map_coordinates(coordinates, offset_x, offset_y, scaling_factor)

            prev_x, prev_y = 0, 0
            contour_start = True
            for contour in mapped_coordinates:
                contour_start = True
                for pair in contour:
                    x, y = pair
                    # TODO mapping from image coordinates to drawing surface coordinates...
                    x_rel = x - prev_x
                    y_rel = y - prev_y
                    if contour_start == True:
                        if self.light == False:
                            # Lift up the pen (if using a pen)
                            self.arm.set_position(x=10, y=0, z=0, roll=None, pitch=None, yaw=None, speed=20, relative=True, wait=True)
                            # Correct servo 6 angle and do the movement
                            self.arm.set_servo_angle(servo_id=6, angle=0, is_radian=False, wait=True)
                            self.arm.set_position(x=0, y=x_rel, z=-y_rel, roll=None, pitch=None, yaw=None, speed=20, relative=True, wait=True)
                            # Put the pen down
                            self.arm.set_position(x=-10, y=0, z=0, roll=None, pitch=None, yaw=None, speed=20, relative=True, wait=True)
                        else:
                            # Turn off the light
                            # Do the movement
                            # Turn on the light
                            pass
                    else:
                        # Do the movement
                        self.arm.set_position(x=0, y=x_rel, z=-y_rel, roll=None, pitch=None, yaw=None, speed=20, relative=True, wait=True)
                    contour_start = False
                    prev_x = x
                    prev_y = y
        else:
            print("Horizontal painting not set up yet!")
            # TODO

