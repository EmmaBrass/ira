import serial
import random
import time

class EyeControl():
    """
    Commands for eye movements.
    """

    def __init__(self, com_port):
        """
        Serial commands must be sent in format <int, int, int>
        These three values correspond to x_val, y_val, and blink in that order.
        x_val and y_val must be from 0 to 1023, blink must be 0 (false) or 1 (true).
        """
        # self.incoming as a way to assess when the default movements need to 
        # stop to execute a special movement.
        self.incoming = False
        self.connection = serial.Serial(port=com_port, baudrate=9600, timeout=None)

    def default_movements(self):
        """
        Eye movements performed when no specific movement is required.
        """
        while self.incoming == False:
            # Do something, wait a variable length of time, do something else.
            # Maybe will need to change this when threading is implemented.
            x_val = random.randint(0,1023)
            y_val = random.randint(0,1023)
            self.connection.write((bytes("<x_val, y_val, 0>", 'utf-8')))
            # TODO is this okay or do we need a more gradual transiton between locations?
            # TODO add in blinking (a human blink is about 0.15 seconds long)

            sleep_time = random.uniform(0.1, 3)
            sleep_counter = 0
            while (sleep_counter < sleep_time) and self.incoming == False:
                time.sleep(0.05)
                sleep_counter += 0.05

        
    def wide_eyed(self, seconds):
        """
        Expand the eyes for a few seconds for a look of suprise or awe.

        :param seconds: How many seconds to open wide for.
        """
        pass

    def sleep(self):
        """
        Close the eyes.
        """
        pass

    def wake_up (self):
        """
        Open the eyes.
        """
        pass

    def fast_left_right(self):
        """
        Move the eyes rapidly left and right in a suspicious way.
        """
        pass

    def eye_roll(self):
        """
        Self-explanatory; do an eye roll.
        """
        pass

    def up_right(self):
        """
        Look up and to the right for a few seconds with some blinking, 
        as if thinking.
        """
        pass

    def stare(self, seconds):
        """
        Stare straight forward, with occasional blinking.
        
        :param seconds: How many seconds to stare for.
        """
        pass