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
        self.movements = 0
        self.connection = serial.Serial(port=com_port, baudrate=9600, timeout=2)

    def send_command(self, command):
        self.connection.write((bytes(command, 'utf-8')))
        time.sleep(0.03)
        # Read response from Arduino
        response = self.connection.readline().decode('utf-8').strip()
        # Flush input buffer to clear any leftover data
        self.connection.flushInput()
        self.connection.flushOutput()

    def default_movement(self):
        """
        Random eye movements performed when no specific movement is required.
        """     
        x_val = random.randint(0,1023)
        y_val = random.randint(0,1023)
        command = f"<{x_val}, {y_val}, 0>"
        self.send_command(command)
        time.sleep(random.uniform(0,3))
        self.movements += 1
        # TODO is this okay or do we need a more gradual transiton between locations?
        blink_count = random.randint(2,4)
        if self.movements >= blink_count:
            # do a blink
            command = f"<{x_val}, {y_val}, 1>"
            self.send_command(command)
            time.sleep(0.15)
            command = f"<{x_val}, {y_val}, 0>"
            self.send_command(command)
            self.movements = 0

    def straight(self):
        """
        Look straight ahead.
        """
        x_val = 512
        y_val = 512
        command = f"<{x_val}, {y_val}, 0>"
        self.send_command(command)
        self.movements += 1
        blink_count = random.randint(2,5)
        if self.movements >= blink_count:
            # do a blink
            command = f"<{x_val}, {y_val}, 1>"
            self.send_command(command)
            time.sleep(0.15)
            command = f"<{x_val}, {y_val}, 0>"
            self.send_command(command)
            self.movements = 0

    def focus(self, foi_x, foi_y):
        """
        Look at the foi
        """
        x_val = foi_x
        y_val = foi_y
        command = f"<{x_val}, {y_val}, 0>"
        self.send_command(command)
        self.movements += 1
        blink_count = random.randint(2,5)
        if self.movements >= blink_count:
            # do a blink
            command = f"<{x_val}, {y_val}, 1>"
            self.send_command(command)
            time.sleep(0.15)
            command = f"<{x_val}, {y_val}, 0>"
            self.send_command(command)
            self.movements = 0