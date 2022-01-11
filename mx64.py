#!/usr/bin/python3 -u

import time
from dynamixel_sdk import *

class DynamixelInterface:
    def __init__(self, control_table):
        self.control_table = control_table

    def open_port(self, DEVICENAME, BAUDRATE, PROTOCOL_VERSION):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if self.portHandler.openPort():
            print("[DXL] Succeeded to open port "+str(DEVICENAME))
        else:
            print("[DXL] Failed to open port "+str(DEVICENAME))

        if self.portHandler.setBaudRate(BAUDRATE):
            print("[DXL] Succeeded to set the baudrate to "+str(BAUDRATE))
        else:
            print("[DXL] Failed to set the baudrate to "+str(BAUDRATE))

    def _any_dxl_error(self, dxl_comm_result, dxl_error):
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return 1
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return 1
        return 0

    def _2c(self, value, bits=32):
        if value&(1<<(bits-1)):
            return value-(1<<bits)
        return value

    def set_torque_state(self, DXL_ID, TORQUE_STATE, debug=1):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.control_table['torque_enable'], TORQUE_STATE)
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Torque set to "+str(TORQUE_STATE)+" for ID "+str(DXL_ID))

    def set_goal_position(self, DXL_ID, GOAL_POSITION, debug=0):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, self.control_table['goal_position'], GOAL_POSITION)
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Goal position set to "+str(GOAL_POSITION)+" for ID "+str(DXL_ID))

    def get_position(self, DXL_ID, debug=0):
        position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, self.control_table['position'])
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Current position is "+str(position)+" for ID "+str(DXL_ID))
            return self._2c(position)

    def get_position_limits(self, DXL_ID):
        pos_min, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, self.control_table['min_position'])
        if self._any_dxl_error(dxl_comm_result, dxl_error): return
        pos_max, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, self.control_table['max_position'])
        if self._any_dxl_error(dxl_comm_result, dxl_error): return
        return [pos_min, pos_max]

    def set_goal_velocity(self, DXL_ID, GOAL_VELOCITY, debug=0):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, self.control_table['goal_velocity'], GOAL_VELOCITY)
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Goal velocity set to "+str(GOAL_VELOCITY)+" for ID "+str(DXL_ID))

    def get_velocity(self, DXL_ID, debug=0):
        velocity, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, self.control_table['velocity'])
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Current velocity is "+str(velocity)+" for ID "+str(DXL_ID))
            return self._2c(velocity)

    def get_velocity_limit(self, DXL_ID, debug=0):
        velocity_limit, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, self.control_table['velocity_limit'])
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Current velocity is "+str(velocity_limit)+" for ID "+str(DXL_ID))
            return self._2c(velocity_limit)

    def get_load(self, DXL_ID, debug=0):
        """unit is 3.36mA"""
        load, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(self.portHandler, DXL_ID, self.control_table['load'])
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Current load is "+str(load)+" for ID "+str(DXL_ID))
            return self._2c(load)

    def set_goal_current(self, DXL_ID, GOAL_CURRENT, debug=0):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, DXL_ID, self.control_table['goal_current'], GOAL_CURRENT)
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Goal load set to "+str(GOAL_CURRENT)+" for ID "+str(DXL_ID))

    def get_operating_mode(self, DXL_ID):
        mode, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, DXL_ID, self.control_table['operating_mode'])
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            return mode

    def get_voltage(self, DXL_ID):
        """unit is 0.1V"""
        v, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(self.portHandler, DXL_ID, self.control_table['present_voltage'])
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            return v

    def set_operating_mode(self, DXL_ID, OPERATING_MODE, debug=1):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, DXL_ID, self.control_table['operating_mode'], OPERATING_MODE)
        if not self._any_dxl_error(dxl_comm_result, dxl_error):
            if debug:
                print("[DXL] Operating mode set to "+r_operating_modes[OPERATING_MODE]+":"+str(OPERATING_MODE)+" for ID "+str(DXL_ID))


mx64_control_table = {
    'operating_mode':11,
    'pwm_limit':36,
    'current_limit':38,
    'torque_enable':64,
    'goal_PWM':100,
    'goal_current':102,
    'goal_velocity':104,
    'goal_position':116,
    'load':126, # unit is 3.36mA
    'velocity':128,
    'position':132,
    'max_position':48,
    'min_position':52,
    'velocity_limit':44,
    'present_voltage':42 #unit is 0.1V
}

operating_modes = {
    'current_control_mode':0,
    'velocity_control_mode':1,
    'position_control_mode':3,
    'extended_position_control_mode':4,
    'current-based_position_control_mode':5,
    'pwm_control_mode':16,
}
r_operating_modes = {value:key for key, value in operating_modes.items()}

