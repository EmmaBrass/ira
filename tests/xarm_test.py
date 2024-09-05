#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Test script for xArm6
"""

from ira_common.arm_movements import ArmMovements
import ira_common.configuration as config

movements = ArmMovements()
movements.straight_bw_position(20)
movements.straight_fw_position(20)



