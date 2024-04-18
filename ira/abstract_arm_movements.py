

class AbstractArmMovements():
    """
    Arm movement types required by this codebase. 
    For overloading when writing a driver for a specific arm.
    """
    def __init__(self) -> None:
        self.previous_scan_x = None

    def scan(self):
        """
        Move left and right while looking for a face.
        """
        pass

    def study_point(self, x, y, z):
        """
        For a 'curious' movement around a point, such as a human face.
        Takes a central position (where the face is location on the search plane)
        and moves around it, with the end effector pointing at it,to create the
        effect that the eyes are 'looking' at it.
        
        :param x: x coordinate of central position
        :param y: y coordinate of central position
        :param z: z coordinate of central position
        """
        pass

    def move_towards_in_plane(self, x, y, z):
        """
        Will be used for moving towards a face, to make it central
        in the camera view.
        Will move just a little bit, then the camera assessment can be 
        done again.

        :param x: x movement
        :param y: y movement
        :param z: z movement
        """
        pass

    def look_up_and_wave(self):
        """
        The robot arm end effector goes up towards to the sky and the whole
        body does a kind of snake-shake.
        """
        pass
    
