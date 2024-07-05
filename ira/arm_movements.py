from xarm.wrapper import XArmAPI #TODO install


class ArmMovements():
    """
    Arm movement commands for the UFactory XArm 6.
    """
    def __init__(self) -> None:
        self.arm = XArmAPI('192.168.1.200')
        self.arm.connect()
        # TODO Turn on collision detection
        # TODO set velocity
        # TODO set_mount_direction()
        self.previous_scan_x = None
        self.vertical = True # If true, vertical surface. If False, horizontal
        self.vertical_dist = 40 # Distance in cm of the vertical surace from the robot base
        self.vertical_height = 0 # Distance in cm of the bottom of the vertical surface from the bottom of the robot base

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

    def paint_image(self, coordinates, image_x, image_y):
        """
        Paint the image!

        :param coordinates: 2D array of contours coordinates
        :param image_x: x height of original image
        :param image_y: y width of original image
        """

        self.arm.get_is_moving()
        #TODO 
        self.arm.set_position()
        


