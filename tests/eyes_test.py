import serial
import random
import time

connection = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=1)
time.sleep(4)
movements = 0

def send_command(command):
    global connection
    connection.write((bytes(command, 'utf-8')))
    time.sleep(0.03)
    # Read response from Arduino
    response = connection.readline().decode('utf-8').strip()
    # Flush input buffer to clear any leftover data
    connection.flushInput()
    connection.flushOutput()

while True:
    print("Beginning of while loop")
    x_val = random.randint(50,1000)
    y_val = random.randint(50,1000)
    command = f"<{x_val}, {y_val}, 0>"
    send_command(command)
    movements += 1
    blink_count = random.randint(3,5)
    if movements >= blink_count:
        print("Doing a blink")
        # do a blink
        command = f"<{x_val}, {y_val}, 1>"
        send_command(command)
        time.sleep(0.15)
        command = f"<{x_val}, {y_val}, 0>"
        send_command(command)
        movements = 0
    time.sleep(random.uniform(0,2.5))
    print("End of while loop")