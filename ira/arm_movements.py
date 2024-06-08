

class ArmMovements():
    """
    Arm movement commands for the UFactory XArm 6.
    """
    def __init__(self) -> None:
        self.previous_scan_x = None

    def scan(self):
        """
        Move left and right while looking for a face.
        """
        pass #TODO

    def stop(self, x, y, z):
        """
        Don't move.
        """
        pass #TODO

    def paint(self, x, y, z):
        """
        Paint the face!
        """
        pass #TODO
