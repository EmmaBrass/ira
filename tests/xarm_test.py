#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Description: Move line(linear motion)
"""

import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI

arm = XArmAPI('192.168.1.200')
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

# Move to origin point for vertical painting: top-left (0,0)
arm.set_servo_angle(servo_id=1, angle=218.2, speed=50, relative=False, wait=True)
arm.set_servo_angle(servo_id=3, angle=-130.5, speed=20, relative=False, wait=True)
arm.set_servo_angle(servo_id=2, angle=18.8, speed=10, relative=False, wait=True)
arm.set_servo_angle(servo_id=4, angle=-66.2, speed=20, relative=False, wait=True)
arm.set_servo_angle(servo_id=5, angle=43.4, speed=20, relative=False, wait=True)
arm.set_servo_angle(servo_id=6, angle=0, is_radian=False, wait=True)

time.sleep(2)

# Test the motion in columns through the space:
for i in range(7): # (50x2)x7 = 700
    arm.set_position(x=0, y=50, z=0, roll=None, pitch=None, yaw=None, speed=20, relative=True, wait=True)
    arm.set_servo_angle(servo_id=6, angle=0, is_radian=False, wait=True)
    arm.set_position(x=0, y=0, z=-700, roll=None, pitch=None, yaw=None, speed=20, relative=True, wait=True)
    arm.set_servo_angle(servo_id=6, angle=0, is_radian=False, wait=True)
    arm.set_position(x=0, y=50, z=0, roll=None, pitch=None, yaw=None, speed=20, relative=True, wait=True)
    arm.set_servo_angle(servo_id=6, angle=0, is_radian=False, wait=True)
    arm.set_position(x=0, y=0, z=700, roll=None, pitch=None, yaw=None, speed=20, relative=True, wait=True)
    arm.set_servo_angle(servo_id=6, angle=0, is_radian=False, wait=True)

# arm.set_position(x=300, y=0, z=150, roll=0, pitch=0, yaw=0, speed=100, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(x=300, y=200, z=250, roll=-180, pitch=0, yaw=0, speed=100, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(x=500, y=200, z=150, roll=-180, pitch=0, yaw=0, speed=100, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(x=500, y=-200, z=250, roll=-180, pitch=0, yaw=0, speed=100, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(x=300, y=-200, z=150, roll=-180, pitch=0, yaw=0, speed=100, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))
# arm.set_position(x=300, y=0, z=250, roll=-180, pitch=0, yaw=0, speed=100, wait=True)
# print(arm.get_position(), arm.get_position(is_radian=True))

# GI


# OPENCV origin is top left... so mine should be to!!

# Add error handling... if hits a singularity... 'reset' itself and skip that contour.

#arm.reset(wait=True)#
arm.disconnect()