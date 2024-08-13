from xarm.wrapper import XArmAPI #TODO install
import cv2
import math, random
import ira.configuration as config

class ArmMovements():
    """
    Arm movement commands for the UFactory XArm 6.
    """
    def __init__(self) -> None:
        self.arm = XArmAPI('192.168.1.200')
        self.arm.motion_enable(enable=True)
        self.arm.set_mode(0)
        self.arm.set_state(state=0)
        self.initial_position()
        # TODO Turn on collision detection
        # TODO set velocity
        # TODO set_mount_direction()

        self.vertical = config.VERTICAL # If true, vertical surface. If False, horizontal

        self.vertical_dist = 40 # Distance in cm of the vertical surace from the robot base
        self.vertical_height = 0 # Distance in cm of the bottom of the vertical surface from the bottom of the robot base
        self.vertical_painting_width = 650 # max reach of robot
        self.vertical_painting_height = 700 # max reach of robot

        self.horizontal_painting_width = 450 # max reach of robot
        self.horizontal_painting_height = 450 # max reach of robot

        self.light = config.LIGHT # False = using a pen/paintbrush instead (hence need x movement away from canvas between contours)

        # Speed settings
        self.painting_speed = 110 # speed when making a mark
        self.between_speed = 130 # speed when moving between marks/pots

        # For tracking distance travelled - for brush reload
        self.reload_dist = 500 #mm, how much to paint before a reload
        self.travelled_dist = self.reload_dist+1 # so it reloads on first movement

        # For paint reload pot
        self.x_dist_reload = 150 # x distance in mm from the top-left origin corner
        self.y_dist_reload = -150 # y distance in mm from the top-left origin corner

        # For brush lifts
        self.brush_lift = 15 # the amount to lift brush off page between lines, mm
        self.pot_lift = 70 # distance in mm to lift when going to paint pot

        # Start position for HORIZONTAL
        # Set the values here if you want to change start position
        self.hor_x_start = config.HOR_X_START
        self.hor_y_start = config.HOR_Y_START
        self.hor_z_start = config.HOR_Z_START
        self.hor_roll_start = config.HOR_ROLL_START
        self.hor_pitch_start = config.HOR_PITCH_START
        self.hor_yaw_start = config.HOR_YAW_START

        # Start position for VERTICAL
        # Set the values here if you want to change start position
        self.ver_x_start = config.VER_X_START
        self.ver_y_start = config.VER_Y_START
        self.ver_z_start = config.VER_Z_START
        self.ver_roll_start = config.VER_ROLL_START
        self.ver_pitch_start = config.VER_PITCH_START
        self.ver_yaw_start = config.VER_YAW_START

    def initial_position(self):
        """
        Move to initial position, cartesian motion.
        """
        # self.arm.set_servo_angle(servo_id=2, angle=0, speed=10, relative=False, wait=True)
        # self.arm.set_servo_angle(servo_id=1, angle=0, speed=50, relative=False, wait=True)
        # self.arm.set_servo_angle(servo_id=3, angle=-150, speed=20, relative=False, wait=True)
        # self.arm.set_servo_angle(servo_id=4, angle=0, speed=20, relative=False, wait=True)
        # self.arm.set_servo_angle(servo_id=5, angle=20, speed=20, relative=False, wait=True)
        # self.arm.set_servo_angle(servo_id=6, angle=0, is_radian=False, wait=True)

        self.arm.set_position(x=278, y=2.5, z=93.7, roll=179.4, pitch=0, yaw =-0.1, speed=50, wait=True, motion_type=2)
        print("Done returning to initial position")

    def straight_position(self):
        """
        Move to a straight up position, useful when moving 
        to vertical locations.
        """
        self.arm.set_position(x=-157.4, y=-23, z=1011, roll=3.2, pitch=-83.4, yaw =0.7, speed=50, wait=True, motion_type=2)
        self.arm.set_servo_angle(servo_id=1, angle=180, speed=60, relative=False, wait=True)
        self.arm.set_position(x=-157.4, y=-23, z=1011, roll=3.2, pitch=-83.4, yaw =0.7, speed=50, wait=True, motion_type=2)
        print("Done moving to straight position")

    def scan(self):
        """
        Move to a new random position within the viewing plane.
        """
        self.arm.set_position(
            x=self.hor_x_start+100, 
            y=self.hor_y_start+random.randint(100,400),
            z=self.hor_z_start+random.randint(200,400),
            roll=None, 
            pitch=None, 
            yaw=None, 
            speed=self.between_speed, 
            relative=False, 
            wait=True
        )

    def stop(self, x, y, z):
        """
        Don't move.
        """
        pass

    def resize_and_center_image(self, image_x, image_y, target_width, target_height):
        """
        Resizes and centre image dimensions for the drawing space
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
    
    def calculate_distance(self, point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)

    def reorder_paths_greedy(self, mapped_coordinates):
        """
        Use greedy algo to reorder paths to have the shortest
        travel distances between end of one and start of another.
        """

        if not mapped_coordinates:
            return []
        
        # Initialize the reordered list with the first path
        reordered_paths = [mapped_coordinates.pop(0)]
        
        while mapped_coordinates:
            last_point = reordered_paths[-1][-1]
            closest_index = None
            closest_distance = float('inf')
            
            for i, path in enumerate(mapped_coordinates):
                start_point = path[0]
                distance = self.calculate_distance(last_point, start_point)
                if distance < closest_distance:
                    closest_distance = distance
                    closest_index = i
            
            # Append the closest path to the reordered list and remove it from the original list
            reordered_paths.append(mapped_coordinates.pop(closest_index))
        
        return reordered_paths

    def paint_image(self, coordinates, image_x, image_y):
        """
        Paint the image!
        Currently only set for vertical painting.
        Vertical painting space of the xArm6 is about 650mm wide and 700mm high.

        :param coordinates: 2D array of contours coordinates
        :param image_x: x height of original image
        :param image_y: y width of original image
        """

        if self.vertical == True:
            print("Doing vertical painting of face")
            # Move to origin point for vertical painting: top-left (0,0)
            print("Setting start postion by x y z.")
            self.arm.set_position(
                x=self.ver_x_start+self.brush_lift, 
                y=self.ver_y_start,
                z=self.ver_z_start, 
                roll=self.ver_roll_start, 
                pitch=self.ver_pitch_start, 
                yaw=self.ver_yaw_start, 
                speed=self.between_speed,
                relative=False, 
                wait=False,
            )
            print("Finished setting postion by x y z.")

            # Map the contour coordinates into the vertical drawing space
            # Map the contour coordinates into the horizontal drawing space
            offset_x, offset_y, scaling_factor = self.resize_and_center_image(
                image_x, 
                image_y, 
                self.vertical_painting_width, 
                self.vertical_painting_height
            )
            mapped_coordinates = self.map_coordinates(coordinates, offset_x, offset_y, scaling_factor)
            reordered_paths = self.reorder_paths_greedy(mapped_coordinates)

            print(f"{mapped_coordinates=}")
            print(f"{reordered_paths=}")

            prev_x, prev_y = 0, 0
            for contour in reordered_paths:
                if self.travelled_dist >= self.reload_dist:
                    self.straight_position()
                    self.reload_brush()
                    self.travelled_dist = 0
                path = []
                start_y_abs = -1
                start_z_abs = -1
                for pair in contour:
                    x, y = pair
                    y_abs = self.ver_y_start+x
                    z_abs = self.ver_z_start-y
                    if start_x_abs == -1 and start_y_abs == -1:
                        start_y_abs = y_abs
                        start_z_abs = z_abs
                    path.append([self.hor_x_start, y_abs, z_abs, None, None, None, 50])
                    if prev_x != 0 and prev_y != 0:
                        x_change = x - prev_x
                        y_change = y - prev_y
                    else:
                        x_change = 0
                        y_change = 0
                    prev_x = x
                    prev_y = y
                    dist_change = math.sqrt(x_change**2+y_change**2)
                    self.travelled_dist += dist_change
                # Move to the start of the path
                if self.light == False:
                    # Lift up the pen
                    self.arm.set_position(
                        x=self.ver_x_start+self.brush_lift,  
                        y=None, 
                        z=None,
                        roll=None, 
                        pitch=None, 
                        yaw=None, 
                        speed=self.between_speed, 
                        relative=False, 
                        wait=True
                    )
                    # Correct servo 6 angle
                    self.arm.set_servo_angle(
                        servo_id=6, 
                        angle=0, 
                        speed=120,
                        is_radian=False, 
                        wait=True
                    )
                    # Do the movement
                    self.arm.set_position(
                        x=self.ver_x_start+self.brush_lift, 
                        y=start_y_abs, 
                        z=start_z_abs,
                        roll=None, 
                        pitch=None, 
                        yaw=None, 
                        speed=self.between_speed, 
                        relative=False, 
                        wait=True
                    )
                    # Put the pen down
                    self.arm.set_position(
                        x=self.ver_x_start,
                        y=None, 
                        z=None,
                        roll=None, 
                        pitch=None, 
                        yaw=None, 
                        speed=self.between_speed, 
                        relative=False, 
                        wait=True
                    )
                else:
                    # Turn off the light
                    # Do the movement to next path
                    # Turn on the light
                    pass
                
                # Do the movement for the path
                self.arm.move_arc_lines(
                    path, 
                    is_radian=False, 
                    times=1, 
                    first_pause_time=0.1, 
                    repeat_pause_time=0, 
                    automatic_calibration=True, 
                    speed=100, 
                    mvacc=1500, 
                    wait=True
                )

            # Return to initial position
            self.straight_position()
            self.initial_position()
                    
        else:
            print("Doing horizontal painting of face")
            # Move to origin point for horizontal painting: top-left (0,0)
            print("Setting start postion by x y z.")
            self.arm.set_position(
                x=self.hor_x_start, 
                y=self.hor_y_start, 
                z=self.hor_z_start+20, 
                roll=self.hor_roll_start, 
                pitch=self.hor_pitch_start, 
                yaw=self.hor_yaw_start, 
                speed=self.between_speed,
                relative=False, 
                wait=False,
            )
            print("Finished setting postion by x y z.")

            # Map the contour coordinates into the horizontal drawing space
            offset_x, offset_y, scaling_factor = self.resize_and_center_image(
                image_x, 
                image_y, 
                self.horizontal_painting_width, 
                self.horizontal_painting_height
            )
            mapped_coordinates = self.map_coordinates(coordinates, offset_x, offset_y, scaling_factor)
            reordered_paths = self.reorder_paths_greedy(mapped_coordinates)

            print(f"{mapped_coordinates=}")
            print(f"{reordered_paths=}")

            prev_x, prev_y = 0, 0
            for contour in reordered_paths:
                if self.travelled_dist >= self.reload_dist:
                    self.reload_brush()
                    self.travelled_dist = 0
                path = []
                start_x_abs = -1
                start_y_abs = -1
                for pair in contour:
                    x, y = pair
                    y_abs = self.hor_y_start+x
                    x_abs = self.hor_x_start+y
                    if start_x_abs == -1 and start_y_abs == -1:
                        start_x_abs = x_abs
                        start_y_abs = y_abs
                    path.append([x_abs, y_abs, self.hor_z_start, None, None, None, 50])
                    if prev_x != 0 and prev_y != 0:
                        x_change = x - prev_x
                        y_change = y - prev_y
                    else:
                        x_change = 0
                        y_change = 0
                    prev_x = x
                    prev_y = y
                    dist_change = math.sqrt(x_change**2+y_change**2)
                    self.travelled_dist += dist_change
                # Move to the start of the path
                if self.light == False:
                    # Lift up the pen
                    self.arm.set_position(
                        x=None, 
                        y=None, 
                        z=self.hor_z_start+self.brush_lift, 
                        roll=None, 
                        pitch=None, 
                        yaw=None, 
                        speed=self.between_speed, 
                        relative=False, 
                        wait=True
                    )
                    # Correct servo 6 angle
                    self.arm.set_servo_angle(
                        servo_id=6, 
                        angle=0, 
                        speed=120,
                        is_radian=False, 
                        wait=True
                    )
                    # Do the movement
                    self.arm.set_position(
                        x=start_x_abs, 
                        y=start_y_abs, 
                        z=self.hor_z_start+self.brush_lift, 
                        roll=None, 
                        pitch=None, 
                        yaw=None, 
                        speed=self.between_speed, 
                        relative=False, 
                        wait=True
                    )
                    # Put the pen down
                    self.arm.set_position(
                        x=None, 
                        y=None, 
                        z=self.hor_z_start, 
                        roll=None, 
                        pitch=None, 
                        yaw=None, 
                        speed=self.between_speed, 
                        relative=False, 
                        wait=True
                    )
                else:
                    # Turn off the light
                    # Do the movement to next path
                    # Turn on the light
                    pass
                
                # Do the movement for the path
                self.arm.move_arc_lines(
                    path, 
                    is_radian=False, 
                    times=1, 
                    first_pause_time=0.1, 
                    repeat_pause_time=0, 
                    automatic_calibration=True, 
                    speed=100, 
                    mvacc=1500, 
                    wait=True
                )

        # Return to initial position
        self.initial_position()


    def reload_brush(self):
        """
        Load the brush with more paint from reload pot.
        """
        # Lift up brush
        self.arm.set_position(
            x=None, 
            y=None, 
            z=self.hor_z_start+self.pot_lift, 
            roll=None, 
            pitch=None, 
            yaw=None, 
            speed=self.between_speed, 
            relative=False, 
            wait=True
        )
        # Move to paint pot
        self.arm.set_position(
            x=self.hor_x_start+self.x_dist_reload, 
            y=self.hor_y_start+self.y_dist_reload, 
            z=None, 
            roll=None, 
            pitch=None, 
            yaw=None, 
            speed=self.between_speed, 
            relative=False, 
            wait=True
        )
        # Put brush down
        self.arm.set_position(
            x=None, 
            y=None, 
            z=self.hor_z_start, 
            roll=None, 
            pitch=None, 
            yaw=None, 
            speed=self.painting_speed, 
            relative=False, 
            wait=True
        ) 
        # Do square in the paint
        self.arm.set_position(x=-20, y=0, z=0, roll=None, pitch=None, yaw=None, speed=self.painting_speed, relative=True, wait=True)
        self.arm.set_position(x=0, y=-20, z=0, roll=None, pitch=None, yaw=None, speed=self.painting_speed, relative=True, wait=True)
        self.arm.set_position(x=20, y=0, z=0, roll=None, pitch=None, yaw=None, speed=self.painting_speed, relative=True, wait=True)
        self.arm.set_position(x=0, y=20, z=0, roll=None, pitch=None, yaw=None, speed=self.painting_speed, relative=True, wait=True)
        # Lift up again
        self.arm.set_position(
            x=None, 
            y=None, 
            z=self.hor_z_start+self.pot_lift, 
            roll=None, 
            pitch=None, 
            yaw=None, 
            speed=self.between_speed, 
            relative=False, 
            wait=True
        )
        # Move back to above start postion
        self.arm.set_position(
            x=self.hor_x_start, 
            y=self.hor_y_start, 
            z=self.hor_z_start+self.pot_lift, 
            roll=self.hor_roll_start, 
            pitch=self.hor_pitch_start, 
            yaw=self.hor_yaw_start, 
            speed=self.between_speed,
            relative=False, 
            wait=False
        )

    def wash_brush(self):
        pass

            
