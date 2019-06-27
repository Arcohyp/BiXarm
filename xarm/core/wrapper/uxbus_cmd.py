#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2018, UFACTORY, Inc.
# All rights reserved.
#
# Author: Jimy Zhang <jimy.zhang@ufactory.cc> <jimy92@163.com>
# Author: Vinman <vinman.wen@ufactory.cc> <vinman.cub@gmail.com>


import threading
import functools
from ..utils import convert
from ..config.x_config import XCONF


def lock_require(func):
    @functools.wraps(func)
    def decorator(*args, **kwargs):
        with args[0].lock:
            return func(*args, **kwargs)
    return decorator


class UxbusCmd(object):
    def __init__(self):
        self._has_error = False
        self._has_warn = False
        self._error_code = 0
        self._warn_code = 0
        self._cmd_num = 0
        self.lock = threading.Lock()

    def check_xbus_prot(self, data, funcode):
        raise NotImplementedError

    def send_pend(self, funcode, num, timeout):
        raise NotImplementedError

    def send_xbus(self, funcode, txdata, num):
        raise NotImplementedError

    @lock_require
    def set_nu8(self, funcode, datas, num):
        ret = self.send_xbus(funcode, datas, num)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.send_pend(funcode, 0, XCONF.UxbusConf.SET_TIMEOUT)

    @lock_require
    def set_get_nu8(self, funcode, datas, num_send, num_get):
        ret = self.send_xbus(funcode, datas, num_send)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.send_pend(funcode, num_get, XCONF.UxbusConf.SET_TIMEOUT)

    @lock_require
    def get_nu8(self, funcode, num):
        ret = self.send_xbus(funcode, 0, 0)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num + 1)
        return self.send_pend(funcode, num, XCONF.UxbusConf.GET_TIMEOUT)

    @lock_require
    def set_nu16(self, funcode, datas, num):
        hexdata = convert.u16s_to_bytes(datas, num)
        ret = self.send_xbus(funcode, hexdata, num * 2)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP]
        ret = self.send_pend(funcode, 0, XCONF.UxbusConf.SET_TIMEOUT)
        return ret

    @lock_require
    def get_nu16(self, funcode, num):
        ret = self.send_xbus(funcode, 0, 0)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num * 2 + 1)
        ret = self.send_pend(funcode, num * 2, XCONF.UxbusConf.GET_TIMEOUT)
        data = [0] * (1 + num)
        data[0] = ret[0]
        data[1:num] = convert.bytes_to_u16s(ret[1:num * 2 + 1], num)
        return data

    @lock_require
    def set_nfp32(self, funcode, datas, num):
        hexdata = convert.fp32s_to_bytes(datas, num)
        ret = self.send_xbus(funcode, hexdata, num * 4)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP]
        return self.send_pend(funcode, 0, XCONF.UxbusConf.SET_TIMEOUT)

    @lock_require
    def get_nfp32(self, funcode, num):
        ret = self.send_xbus(funcode, 0, 0)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (num * 4 + 1)
        ret = self.send_pend(funcode, num * 4, XCONF.UxbusConf.GET_TIMEOUT)
        data = [0] * (1 + num)
        data[0] = ret[0]
        data[1:num] = convert.bytes_to_fp32s(ret[1:num * 4 + 1], num)
        return data

    @lock_require
    def swop_nfp32(self, funcode, datas, txn, rxn):
        hexdata = convert.fp32s_to_bytes(datas, txn)
        ret = self.send_xbus(funcode, hexdata, txn * 4)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (rxn + 1)
        ret = self.send_pend(funcode, rxn * 4, XCONF.UxbusConf.GET_TIMEOUT)
        data = [0] * (1 + rxn)
        data[0] = ret[0]
        data[1:rxn] = convert.bytes_to_fp32s(ret[1:rxn * 4 + 1], rxn)
        return data

    @lock_require
    def is_nfp32(self, funcode, datas, txn):
        hexdata = convert.fp32s_to_bytes(datas, txn)
        ret = self.send_xbus(funcode, hexdata, txn * 4)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * 2
        return self.send_pend(funcode, 1, XCONF.UxbusConf.GET_TIMEOUT)

    def get_version(self):
        return self.get_nu8(XCONF.UxbusReg.GET_VERSION, 40)

    def get_robot_sn(self):
        return self.get_nu8(XCONF.UxbusReg.GET_ROBOT_SN, 40)

    def check_verification(self):
        # txdata = signature, 175: signature length if use 14-character SN for plain text, do not miss '\n's
        return self.get_nu8(XCONF.UxbusReg.CHECK_VERIFY, 1)

    def shutdown_system(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SHUTDOWN_SYSTEM, txdata, 1)

    def motion_en(self, axis_id, enable):
        txdata = [axis_id, int(enable)]
        return self.set_nu8(XCONF.UxbusReg.MOTION_EN, txdata, 2)

    def set_state(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SET_STATE, txdata, 1)

    def get_state(self):
        return self.get_nu8(XCONF.UxbusReg.GET_STATE, 1)

    def get_cmdnum(self):
        return self.get_nu16(XCONF.UxbusReg.GET_CMDNUM, 1)

    def get_err_code(self):
        return self.get_nu8(XCONF.UxbusReg.GET_ERROR, 2)

    def clean_err(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_ERR, 0, 0)

    def clean_war(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_WAR, 0, 0)

    def set_brake(self, axis_id, enable):
        txdata = [axis_id, int(enable)]
        return self.set_nu8(XCONF.UxbusReg.SET_BRAKE, txdata, 2)

    def set_mode(self, mode):
        txdata = [mode]
        return self.set_nu8(XCONF.UxbusReg.SET_MODE, txdata, 1)

    def move_line(self, mvpose, mvvelo, mvacc, mvtime):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_LINE, txdata, 9)

    def move_lineb(self, mvpose, mvvelo, mvacc, mvtime, mvradii):
        txdata = [mvpose[i] for i in range(6)]
        txdata += [mvvelo, mvacc, mvtime, mvradii]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_LINEB, txdata, 10)

    def move_joint(self, mvjoint, mvvelo, mvacc, mvtime):
        txdata = [mvjoint[i] for i in range(7)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_JOINT, txdata, 10)

    def move_gohome(self, mvvelo, mvacc, mvtime):
        txdata = [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_HOME, txdata, 3)

    def move_servoj(self, mvjoint, mvvelo, mvacc, mvtime):
        txdata = [mvjoint[i] for i in range(7)]
        txdata += [mvvelo, mvacc, mvtime]
        return self.set_nfp32(XCONF.UxbusReg.MOVE_SERVOJ, txdata, 10)

    def sleep_instruction(self, sltime):
        txdata = [sltime]
        return self.set_nfp32(XCONF.UxbusReg.SLEEP_INSTT, txdata, 1)

    def move_circle(self, pose1, pose2, mvvelo, mvacc, mvtime, percent):
        txdata = [0] * 16
        for i in range(6):
            txdata[i] = pose1[i]
            txdata[6 + i] = pose2[i]
        txdata[12] = mvvelo
        txdata[13] = mvacc
        txdata[14] = mvtime
        txdata[15] = percent
        ret = self.set_nfp32(XCONF.UxbusReg.MOVE_CIRCLE, txdata, 16)
        return ret

    def set_tcp_jerk(self, jerk):
        txdata = [jerk]
        return self.set_nfp32(XCONF.UxbusReg.SET_TCP_JERK, txdata, 1)

    def set_tcp_maxacc(self, acc):
        txdata = [acc]
        return self.set_nfp32(XCONF.UxbusReg.SET_TCP_MAXACC, txdata, 1)

    def set_joint_jerk(self, jerk):
        txdata = [jerk]
        return self.set_nfp32(XCONF.UxbusReg.SET_JOINT_JERK, txdata, 1)

    def set_joint_maxacc(self, acc):
        txdata = [acc]
        return self.set_nfp32(XCONF.UxbusReg.SET_JOINT_MAXACC, txdata, 1)

    def set_tcp_offset(self, pose_offset):
        return self.set_nfp32(XCONF.UxbusReg.SET_TCP_OFFSET, pose_offset, 6)

    def set_tcp_load(self, load_mass, load_com):
        param_list = [load_mass]
        param_list.extend(load_com)
        return self.set_nfp32(XCONF.UxbusReg.SET_LOAD_PARAM, param_list, 4)

    def set_collis_sens(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SET_COLLIS_SENS, txdata, 1)

    def set_teach_sens(self, value):
        txdata = [value]
        return self.set_nu8(XCONF.UxbusReg.SET_TEACH_SENS, txdata, 1)

    def set_gravity_dir(self, gravity_dir):
        return self.set_nfp32(XCONF.UxbusReg.SET_GRAVITY_DIR, gravity_dir, 3)

    def clean_conf(self):
        return self.set_nu8(XCONF.UxbusReg.CLEAN_CONF, 0, 0)

    def save_conf(self):
        return self.set_nu8(XCONF.UxbusReg.SAVE_CONF, 0, 0)

    def get_joint_pos(self):
        return self.get_nfp32(XCONF.UxbusReg.GET_JOINT_POS, 7)

    def get_tcp_pose(self):
        return self.get_nfp32(XCONF.UxbusReg.GET_TCP_POSE, 6)

    def get_ik(self, pose):
        return self.swop_nfp32(XCONF.UxbusReg.GET_IK, pose, 6, 7)

    def get_fk(self, angles):
        return self.swop_nfp32(XCONF.UxbusReg.GET_FK, angles, 7, 6)

    def is_joint_limit(self, joint):
        return self.is_nfp32(XCONF.UxbusReg.IS_JOINT_LIMIT, joint, 7)

    def is_tcp_limit(self, pose):
        return self.is_nfp32(XCONF.UxbusReg.IS_TCP_LIMIT, pose, 6)

    def gripper_addr_w16(self, addr, value):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_W16B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_W16B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def gripper_addr_r16(self, addr):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_R16B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_R16B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]

    def gripper_addr_w32(self, addr, value):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_W32B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_W32B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def gripper_addr_r32(self, addr):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_R32B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_R32B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]

    def gripper_set_en(self, value):
        return self.gripper_addr_w16(XCONF.ServoConf.CON_EN, value)

    def gripper_set_mode(self, value):
        return self.gripper_addr_w16(XCONF.ServoConf.CON_MODE, value)

    def gripper_set_zero(self):
        return self.gripper_addr_w16(XCONF.ServoConf.MT_ZERO, 1)

    def gripper_get_pos(self):
        return self.gripper_addr_r32(XCONF.ServoConf.CURR_POS)

    def gripper_set_pos(self, pulse):
        return self.gripper_addr_w32(XCONF.ServoConf.TAGET_POS, pulse)

    def gripper_set_posspd(self, speed):
        return self.gripper_addr_w16(XCONF.ServoConf.POS_SPD, speed)

    def gripper_get_errcode(self):
        ret = self.get_nu8(XCONF.UxbusReg.TGPIO_ERR, 2)
        return ret

    def gripper_clean_err(self):
        return self.gripper_addr_w16(XCONF.ServoConf.RESET_ERR, 1)

    def tgpio_addr_w16(self, addr, value):
        txdata = bytes([XCONF.TGPIO_ID])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_W16B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_W16B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def tgpio_addr_r16(self, addr):
        txdata = bytes([XCONF.TGPIO_ID])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_R16B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_R16B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        ret1[1] = convert.bytes_to_long_big(ret[1:5])
        return ret1

    def tgpio_addr_w32(self, addr, value):
        txdata = bytes([XCONF.TGPIO_ID])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_W32B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_W32B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def tgpio_addr_r32(self, addr):
        txdata = bytes([XCONF.TGPIO_ID])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_R32B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_R32B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        ret1[1] = convert.bytes_to_long_big(ret[1:5])
        return ret1

    def tgpio_get_digital(self):
        ret = self.tgpio_addr_r16(XCONF.ServoConf.DIGITAL_IN)
        value = [0] * 3
        value[0] = ret[0]
        value[1] = ret[1] & 0x0001
        value[2] = (ret[1] & 0x0002) >> 1
        return value

    def tgpio_set_digital(self, ionum, value):
        tmp = 0
        if ionum == 1:
            tmp = tmp | 0x0100
            if value:
                tmp = tmp | 0x0001
        elif ionum == 2:
            tmp = tmp | 0x0200
            if value:
                tmp = tmp | 0x0002
        else:
            return [-1, -1]
        return self.tgpio_addr_w16(XCONF.ServoConf.DIGITAL_OUT, tmp)

    def tgpio_get_analog1(self):
        ret = self.tgpio_addr_r16(XCONF.ServoConf.ANALOG_IO1)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1] * 3.3 / 4096.0
        return value

    def tgpio_get_analog2(self):
        ret = self.tgpio_addr_r16(XCONF.ServoConf.ANALOG_IO2)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1] * 3.3 / 4096.0
        return value

    def tgpio_set_modbus(self, modbus_t, len_t):
        txdata = bytes([XCONF.TGPIO_ID])
        txdata += bytes(modbus_t)
        ret = self.send_xbus(XCONF.UxbusReg.TGPIO_MODBUS, txdata, len_t + 1)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.TGPIO_MODBUS, -1, XCONF.UxbusConf.GET_TIMEOUT)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        ret1[1] = ret[1:]
        return ret

    def gripper_modbus_w16s(self, addr, value, length):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += bytes([0x10])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.u16_to_bytes(length)
        txdata += bytes([length * 2])
        txdata += value
        ret = self.tgpio_set_modbus(txdata, length * 2 + 7)
        return ret

    def gripper_modbus_r16s(self, addr, length):
        txdata = bytes([XCONF.GRIPPER_ID])
        txdata += bytes([0x03])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.u16_to_bytes(length)
        ret = self.tgpio_set_modbus(txdata, 6)
        return ret

    def gripper_modbus_set_en(self, value):
        value = convert.u16_to_bytes(int(value))
        return self.gripper_modbus_w16s(XCONF.ServoConf.CON_EN, value, 1)

    def gripper_modbus_set_mode(self, value):
        value = convert.u16_to_bytes(int(value))
        return self.gripper_modbus_w16s(XCONF.ServoConf.CON_MODE, value, 1)

    def gripper_modbus_set_zero(self):
        value = convert.u16_to_bytes(int(1))
        return self.gripper_modbus_w16s(XCONF.ServoConf.MT_ZERO, value, 1)

    def gripper_modbus_get_pos(self):
        ret = self.gripper_modbus_r16s(XCONF.ServoConf.CURR_POS, 2)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) == 9:
            ret1[1] = convert.bytes_to_long_big(ret[5:9])
        else:
            if ret1[0] == 0:
                ret1[0] = XCONF.UxbusState.ERR_LENG
            # print('gripper_modbus_get_pos:', len(ret), ret)
        # print(ret1, ret)
        return ret1

    def gripper_modbus_set_pos(self, pulse):
        value = bytes([(int(pulse) >> 24) & 0xFF])
        value += bytes([(int(pulse) >> 16) & 0xFF])
        value += bytes([(int(pulse) >> 8) & 0xFF])
        value += bytes([int(pulse) & 0xFF])
        return self.gripper_modbus_w16s(XCONF.ServoConf.TAGET_POS, value, 2)

    def gripper_modbus_set_posspd(self, speed):
        speed = convert.u16_to_bytes(int(speed))
        return self.gripper_modbus_w16s(XCONF.ServoConf.POS_SPD, speed, 1)

    def gripper_modbus_get_errcode(self):
        ret = self.gripper_modbus_r16s(XCONF.ServoConf.ERR_CODE, 1)
        ret1 = [0] * 2
        ret1[0] = ret[0]
        if ret[0] in [0, XCONF.UxbusState.ERR_CODE, XCONF.UxbusState.WAR_CODE] and len(ret) == 7:
            ret1[1] = convert.bytes_to_u16(ret[5:7])
        else:
            if ret1[0] == 0:
                ret1[0] = XCONF.UxbusState.ERR_LENG
            # print('gripper_modbus_get_errcode:', len(ret), ret)
        # print(ret1, ret)
        return ret1

    def gripper_modbus_clean_err(self):
        value = convert.u16_to_bytes(int(1))
        return self.gripper_modbus_w16s(XCONF.ServoConf.RESET_ERR, value, 1)

    def servo_set_zero(self, axis_id):
        txdata = [int(axis_id)]
        ret = self.set_nu8(XCONF.UxbusReg.SERVO_ZERO, txdata, 1)
        return ret

    def servo_get_dbmsg(self):
        ret = self.get_nu8(XCONF.UxbusReg.SERVO_DBMSG, 16)
        return ret

    def servo_addr_w16(self, axis_id, addr, value):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.SERVO_W16B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.SERVO_W16B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def servo_addr_r16(self, axis_id, addr):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.SERVO_R16B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.SERVO_R16B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]
        # return [ret[0], convert.bytes_to_long_big(ret[1:5])[0]]

    def servo_addr_w32(self, axis_id, addr, value):
        txdata = bytes([axis_id])
        txdata += convert.u16_to_bytes(addr)
        txdata += convert.fp32_to_bytes(value)
        ret = self.send_xbus(XCONF.UxbusReg.SERVO_W32B, txdata, 7)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.SERVO_W32B, 0, XCONF.UxbusConf.GET_TIMEOUT)
        return ret

    def servo_addr_r32(self, axis, addr):
        txdata = bytes([axis])
        txdata += convert.u16_to_bytes(addr)
        ret = self.send_xbus(XCONF.UxbusReg.SERVO_R32B, txdata, 3)
        if ret != 0:
            return [XCONF.UxbusState.ERR_NOTTCP] * (7 + 1)

        ret = self.send_pend(XCONF.UxbusReg.SERVO_R32B, 4, XCONF.UxbusConf.GET_TIMEOUT)
        return [ret[0], convert.bytes_to_long_big(ret[1:5])]
        # return [ret[0], convert.bytes_to_long_big(ret[1:5])[0]]

    # -----------------------------------------------------
    # controler gpio
    # -----------------------------------------------------
    def cgpio_get_auxdigit(self):
        ret = self.get_nu16(XCONF.UxbusReg.CGPIO_GET_DIGIT, 1)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1]
        return value

    def cgpio_get_analog1(self):
        ret = self.get_nu16(XCONF.UxbusReg.CGPIO_GET_ANALOG1, 1)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1] * 10.0 / 4096.0
        return value

    def cgpio_get_analog2(self):
        ret = self.get_nu16(XCONF.UxbusReg.CGPIO_GET_ANALOG2, 1)
        value = [0] * 2
        value[0] = ret[0]
        value[1] = ret[1] * 10.0 / 4096.0
        return value

    def cgpio_set_auxdigit(self, ionum, value):
        """
        ionum: 0~7
        value: 0 / 1
        """
        if ionum > 7:
            return [-1, -1]

        tmp = [0] * 1
        tmp[0] = tmp[0] | (0x0100 << ionum)
        if value:
            tmp[0] = tmp[0] | (0x0001 << ionum)
        return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_DIGIT, tmp, 1)

    def cgpio_set_analog1(self, value):
        txdata = [int(value / 10.0 * 4096.0)]
        return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_ANALOG1, txdata, 1)

    def cgpio_set_analog2(self, value):
        txdata = [int(value / 10.0 * 4096.0)]
        return self.set_nu16(XCONF.UxbusReg.CGPIO_SET_ANALOG2, txdata, 1)

    def cgpio_set_infun(self, num, fun):
        txdata = [int(num), int(fun)]
        return self.set_nu8(XCONF.UxbusReg.CGPIO_SET_IN_FUN, txdata, 2)

    def cgpio_set_outfun(self, num, fun):
        txdata = [int(num), int(fun)]
        return self.set_nu8(XCONF.UxbusReg.CGPIO_SET_OUT_FUN, txdata, 2)

    def cgpio_get_state(self):
        ret = self.get_nu8(XCONF.UxbusReg.CGPIO_GET_STATE, 34)
        msg = [0] * 13
        msg[0] = ret[0]
        msg[1] = ret[1]
        msg[2] = ret[2]

        msg[3:11] = convert.bytes_to_u16s(ret[3:19], 8)
        msg[7] = msg[7] / 4096.0 * 10.0
        msg[8] = msg[8] / 4096.0 * 10.0
        msg[9] = msg[9] / 4096.0 * 10.0
        msg[10] = msg[10] / 4096.0 * 10.0
        msg[11] = ret[19:27]
        msg[12] = ret[27:35]

        return msg
