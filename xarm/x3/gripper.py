#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>

import time
from .utils import xarm_is_connected
from ..core.config.x_config import XCONF
from ..core.utils.log import logger


class Gripper(object):
    def __init__(self):
        self._gripper_error_code = 0

    # @xarm_is_connected(_type='set')
    # def set_gripper_addr_16(self, addr, value):
    #     ret = self.arm_cmd.gripper_addr_w16(addr, value)
    #     return ret[0]
    #
    # @xarm_is_connected(_type='get')
    # def get_gripper_addr_16(self, addr):
    #     ret = self.arm_cmd.gripper_addr_r16(addr)
    #     return ret[0], ret[1]
    #
    # @xarm_is_connected(_type='set')
    # def set_gripper_addr_32(self, addr, value):
    #     ret = self.arm_cmd.gripper_addr_w32(addr, value)
    #     return ret[0]
    #
    # @xarm_is_connected(_type='get')
    # def get_gripper_addr_32(self, addr):
    #     ret = self.arm_cmd.gripper_addr_r32(addr)
    #     return ret[0], ret[1]

    @xarm_is_connected(_type='set')
    def set_gripper_enable(self, enable):
        ret = self.arm_cmd.gripper_modbus_set_en(int(enable))
        _, err = self.get_gripper_err_code()
        logger.info('API -> set_gripper_enable -> ret={}, enable={}, ret2={}, err={}'.format(ret[0], enable, _, err))
        return ret[0] if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] else _

    @xarm_is_connected(_type='set')
    def set_gripper_mode(self, mode):
        ret = self.arm_cmd.gripper_modbus_set_mode(mode)
        _, err = self.get_gripper_err_code()
        logger.info('API -> set_gripper_mode -> ret={}, mode={}, ret2={}, err={}'.format(ret[0], mode, _, err))
        return ret[0] if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] else _

    @xarm_is_connected(_type='set')
    def set_gripper_speed(self, speed):
        ret = self.arm_cmd.gripper_modbus_set_posspd(speed)
        _, err = self.get_gripper_err_code()
        logger.info('API -> set_gripper_speed -> ret={}, speed={}, ret2={}, err={}'.format(ret[0], speed, _, err))
        return ret[0] if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] else _

    @xarm_is_connected(_type='get')
    def get_gripper_position(self):
        ret = self.arm_cmd.gripper_modbus_get_pos()
        _, err = self.get_gripper_err_code()
        if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] or len(ret) <= 1:
            return ret[0], None
        elif _ == 0 and err == 0:
            return ret[0], int(ret[1])
        else:
            return _ if err == 0 else XCONF.UxbusState.ERR_CODE, None

    @xarm_is_connected(_type='set')
    def set_gripper_position(self, pos, wait=False, speed=None, auto_enable=False, timeout=None):
        if auto_enable:
            self.arm_cmd.gripper_modbus_set_en(True)
        if speed is not None:
            self.arm_cmd.gripper_modbus_set_posspd(speed)
        ret = self.arm_cmd.gripper_modbus_set_pos(pos)
        logger.info('API -> set_gripper_position -> ret={}, pos={}'.format(ret[0], pos))
        if wait:
            is_add = True
            last_pos = 0
            _, p = self.get_gripper_position()
            if _ == 0 and p is not None:
                last_pos = int(p)
                if last_pos == pos:
                    return 0
                is_add = True if pos > last_pos else False
            count = 0
            count2 = 0
            start_time = time.time()
            if not timeout or not isinstance(timeout, (int, float)):
                timeout = 10
            while time.time() - start_time < timeout:
                _, p = self.get_gripper_position()
                if _ == 0 and p is not None:
                    cur_pos = int(p)
                    if abs(pos - cur_pos) <= 1:
                        last_pos = cur_pos
                        break
                    if is_add:
                        if cur_pos <= last_pos:
                            count += 1
                        elif cur_pos <= pos:
                            last_pos = cur_pos
                            count = 0
                            count2 = 0
                        else:
                            count2 += 1
                            if count2 >= 10:
                                break
                    else:
                        if cur_pos >= last_pos:
                            count += 1
                        elif cur_pos >= pos:
                            last_pos = cur_pos
                            count = 0
                            count2 = 0
                        else:
                            count2 += 1
                            if count2 >= 10:
                                break
                    if count >= 5:
                        # print('gripper target: {}, current: {}'.format(pos, cur_pos))
                        break
                    time.sleep(0.2)
                else:
                    return _
            # print('gripper, pos: {}, last: {}'.format(pos, last_pos))
            return ret[0]
        else:
            _, err = self.get_gripper_err_code()
            return ret[0] if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] else _
            # return err if _ == 0 and err != 0 else ret[0]

    @xarm_is_connected(_type='get')
    def get_gripper_err_code(self):
        ret = self.arm_cmd.gripper_modbus_get_errcode()
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE]:
            self._gripper_error_code = ret[1]
            if ret[0] == XCONF.UxbusState.ERR_CODE:
                self.get_err_warn_code()
                if self.error_code == 19 or self.error_code == 28:
                    print('gripper/tgpio error, code=C{}, G{}'.format(self.error_code, ret[1]))
                    return ret[0], ret[1]
            ret[0] = 0
            return ret[0], ret[1]
        return ret[0], 0

    @xarm_is_connected(_type='set')
    def clean_gripper_error(self):
        ret = self.arm_cmd.gripper_modbus_clean_err()
        _, err = self.get_gripper_err_code()
        logger.info('API -> clean_gripper_error -> ret={}, ret2={}, err={}'.format(ret[0], _, err))
        return ret[0] if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] else _
        # return err if _ == 0 and err != 0 else ret[0]

    @xarm_is_connected(_type='set')
    def set_gripper_zero(self):
        """
        Warnning, do not use, may cause the arm to be abnormal,  just for debugging
        :return: 
        """
        ret = self.arm_cmd.gripper_modbus_set_zero()
        _, err = self.get_gripper_err_code()
        logger.info('API -> set_gripper_zero -> ret={}, ret2={}, err={}'.format(ret[0], _, err))
        return ret[0] if ret[0] not in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] else _
        # return err if _ == 0 and err != 0 else ret[0]
