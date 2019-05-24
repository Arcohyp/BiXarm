#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2019, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

"""
Move line(linear motion),
    set_position:
        1. explicit setting relative=True to relative move
        2. explicit setting wait=True to wait for the arm to complete
    get_position:
        1. explicit setting is_radian=False, the returned value(roll/pitch/yaw) unit is degree (°)
"""

import os
import sys
import time
sys.path.append(os.path.join(os.path.dirname(__file__), '../../..'))

from xarm.wrapper import XArmAPI
from configparser import ConfigParser
parser = ConfigParser()
parser.read('../robot.conf')
try:
    ip = parser.get('xArm', 'ip')
except:
    ip = input('Please input the xArm ip address[192.168.1.194]:')
    if not ip:
        ip = '192.168.1.194'

arm = XArmAPI(ip)
arm.motion_enable(enable=True)
arm.set_mode(0)
arm.set_state(state=0)

arm.reset(wait=True)

arm.set_position(x=100, relative=True, wait=True)
print(arm.get_position(is_radian=False))
arm.set_position(y=200, z=100, relative=True, wait=True)
print(arm.get_position(is_radian=False))
arm.set_position(x=200, z=-100, relative=True, wait=True)
print(arm.get_position(is_radian=False))
arm.set_position(y=-400, z=100, relative=True, wait=True)
print(arm.get_position(is_radian=False))
arm.set_position(x=-200, z=-100, relative=True, wait=True)
print(arm.get_position(is_radian=False))
arm.set_position(y=200, z=100, relative=True, wait=True)
print(arm.get_position(is_radian=False))

arm.disconnect()
