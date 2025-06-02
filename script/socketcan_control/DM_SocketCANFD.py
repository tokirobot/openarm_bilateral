# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# This is a derivative of the following software.
# https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py
#
# The original file is released under the MIT License:
#
# Copyright (c) 2024 cmjang
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

## CAN FD ver. you can choose CLASSIC CAN or CAN FD

import can
from time import sleep, time
import numpy as np
from enum import IntEnum
from struct import unpack
from struct import pack

import pdb
import logging

class Motor:
    def __init__(self, MotorType, SlaveID, MasterID):
        """
        Initialize a Motor object.

        :param MotorType: Type of the motor.
        :param SlaveID: CAN ID of the motor.
        :param MasterID: Master ID (recommended not to be 0).
        """
        self.Pd = 0.0  # Desired position (optional use)
        self.Vd = 0.0  # Desired velocity (optional use)
        self.goal_position = 0.0  # Target position for control
        self.goal_tau = 0.0       # Target torque for control

        self.state_q = 0.0        # Current position
        self.state_dq = 0.0       # Current velocity
        self.state_tau = 0.0      # Current torque
        self.state_tmos = 0       # Motor temperature (MOSFET)
        self.state_trotor = 0     # Rotor temperature

        self.SlaveID = SlaveID
        self.MasterID = MasterID
        self.MotorType = MotorType
        self.isEnable = False
        self.NowControlMode = Control_Type.MIT
        self.temp_param_dict = {}  # Temporary storage for parameters

    def recv_data(self, q: float, dq: float, tau: float, tmos: int, trotor: int):
        """
        Update motor state variables.

        :param q: Current position
        :param dq: Current velocity
        :param tau: Current torque
        :param tmos: Temperature of the motor MOSFET
        :param trotor: Temperature of the rotor
        """
        self.state_q = q
        self.state_dq = dq
        self.state_tau = tau
        self.state_tmos = tmos
        self.state_trotor = trotor

    def getPosition(self) -> float:
        """Return the current position of the motor."""
        return self.state_q

    def getVelocity(self) -> float:
        """Return the current velocity of the motor."""
        return self.state_dq

    def getTorque(self) -> float:
        """Return the current torque of the motor."""
        return self.state_tau

    def getParam(self, RID):
        """
        Retrieve a parameter previously read from the motor.

        :param RID: Parameter ID
        :return: Stored parameter value or None if not available
        """
        return self.temp_param_dict.get(RID)


class MotorControl:
    # Limit parameters for each motor type [max_position, max_velocity, max_torque]
    Limit_Param = [
        [12.5, 50, 5],   # DM3507 
        [12.5, 30, 10],  # DM4310
        [12.5, 50, 10],  # DM4310_48V
        [12.5, 8, 28],   # DM4340
        [12.5, 10, 28],  # DM4340_48V
        [12.5, 45, 20],  # DM6006
        [12.5, 45, 40],  # DM8006
        [12.5, 45, 54],  # DM8009
        [12.5, 25, 200], # DM10010L
        [12.5, 20, 200], # DM10010
        [12.5, 280, 1],  # DMH3510
        [12.5, 45, 10],  # DMH6215
        [12.5, 45, 10]   # DMG6220
    ]
    def __init__(self, channel: str, use_canfd: bool = True):
        """
        MotorControl constructor with CAN FD support
        :param channel: CAN interface channel name (e.g., 'can0')
        :param bitrate: Bitrate for classical CAN or arbitration phase of CAN FD
        :param use_canfd: Whether to use CAN FD mode
        """
        self.Motors_map = dict()
        self.data_save = bytes()
        self.use_canfd = use_canfd

        if self.use_canfd:
            self.canbus = can.interface.Bus(
                channel=channel,
                interface='socketcan',
                fd=True,
                bitrate=1000000,
                data_bitrate=5000000  # You may configure this separately if needed
            )
        else:
            self.canbus = can.interface.Bus(
                channel=channel,
                interface='socketcan',
                bitrate=1000000
            )


    def controlMIT(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        """
        Control the motor using MIT control mode.

        Args:
            DM_Motor (Motor): Target motor object
            kp (float): Position gain
            kd (float): Velocity gain
            q (float): Desired position
            dq (float): Desired velocity
            tau (float): Desired torque
        """
        if DM_Motor.SlaveID not in self.Motors_map:
            print("controlMIT ERROR : Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        MotorType = DM_Motor.MotorType
        Q_MAX = self.Limit_Param[MotorType][0]
        DQ_MAX = self.Limit_Param[MotorType][1]
        TAU_MAX = self.Limit_Param[MotorType][2]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00]*8, np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xff
        data_buf[1] = q_uint & 0xff
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)
        data_buf[4] = kp_uint & 0xff
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)
        data_buf[7] = tau_uint & 0xff
        self.__send_data(DM_Motor.SlaveID, data_buf)
        self.recv()

    def controlMIT2(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float):
        """
        Control the motor using MIT control mode.

        Args:
            DM_Motor (Motor): Target motor object
            kp (float): Position gain
            kd (float): Velocity gain
            q (float): Desired position
            dq (float): Desired velocity
            tau (float): Desired torque
        """
        if DM_Motor.SlaveID not in self.Motors_map:
            print("controlMIT ERROR : Motor ID not found")
            return
        kp_uint = float_to_uint(kp, 0, 500, 12)
        kd_uint = float_to_uint(kd, 0, 5, 12)
        MotorType = DM_Motor.MotorType
        Q_MAX = self.Limit_Param[MotorType][0]
        DQ_MAX = self.Limit_Param[MotorType][1]
        TAU_MAX = self.Limit_Param[MotorType][2]
        q_uint = float_to_uint(q, -Q_MAX, Q_MAX, 16)
        dq_uint = float_to_uint(dq, -DQ_MAX, DQ_MAX, 12)
        tau_uint = float_to_uint(tau, -TAU_MAX, TAU_MAX, 12)
        data_buf = np.array([0x00]*8, np.uint8)
        data_buf[0] = (q_uint >> 8) & 0xff
        data_buf[1] = q_uint & 0xff
        data_buf[2] = dq_uint >> 4
        data_buf[3] = ((dq_uint & 0xf) << 4) | ((kp_uint >> 8) & 0xf)
        data_buf[4] = kp_uint & 0xff
        data_buf[5] = kd_uint >> 4
        data_buf[6] = ((kd_uint & 0xf) << 4) | ((tau_uint >> 8) & 0xf)
        data_buf[7] = tau_uint & 0xff
        self.__send_data(DM_Motor.SlaveID, data_buf)
    

    def control_delay(self, DM_Motor, kp: float, kd: float, q: float, dq: float, tau: float, delay: float):
        """
        MIT control mode with delay.

        Args:
            DM_Motor (Motor): Target motor object
            kp (float): Position gain
            kd (float): Velocity gain
            q (float): Desired position
            dq (float): Desired velocity
            tau (float): Desired torque
            delay (float): Delay duration in seconds
        """
        self.controlMIT(DM_Motor, kp, kd, q, dq, tau)
        sleep(delay)

    def control_Pos_Vel(self, Motor, P_desired: float, V_desired: float):
        """
        Control the motor in position and velocity mode.

        Args:
            Motor (Motor): Target motor object
            P_desired (float): Desired position
            V_desired (float): Desired velocity
        """
        if Motor.SlaveID not in self.Motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        Motorid = 0x100 + Motor.SlaveID
        data_buf = np.array([0x00]*8, np.uint8)
        data_buf[0:4] = float_to_uint8s(P_desired)
        data_buf[4:8] = float_to_uint8s(V_desired)
        self.__send_data(Motorid, data_buf)
        self.recv()

    def control_Pos_Vel2(self, Motor, P_desired: float, V_desired: float):
        """
        Control the motor in position and velocity mode.

        Args:
            Motor (Motor): Target motor object
            P_desired (float): Desired position
            V_desired (float): Desired velocity
        """
        if Motor.SlaveID not in self.Motors_map:
            print("Control Pos_Vel Error : Motor ID not found")
            return
        Motorid = 0x100 + Motor.SlaveID
        data_buf = np.array([0x00]*8, np.uint8)
        data_buf[0:4] = float_to_uint8s(P_desired)
        data_buf[4:8] = float_to_uint8s(V_desired)
        self.__send_data(Motorid, data_buf)

    def control_Vel(self, Motor, Vel_desired):
        """
        Control the motor in velocity mode.

        Args:
            Motor (Motor): Target motor object
            Vel_desired (float): Desired velocity
        """
        if Motor.SlaveID not in self.Motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        Motorid = 0x200 + Motor.SlaveID
        data_buf = np.array([0x00]*8, np.uint8)
        data_buf[0:4] = float_to_uint8s(Vel_desired)
        self.__send_data(Motorid, data_buf)
        self.recv()

    def control_Vel2(self, Motor, Vel_desired):
        """
        Control the motor in velocity mode.

        Args:
            Motor (Motor): Target motor object
            Vel_desired (float): Desired velocity
        """
        if Motor.SlaveID not in self.Motors_map:
            print("control_VEL ERROR : Motor ID not found")
            return
        Motorid = 0x200 + Motor.SlaveID
        data_buf = np.array([0x00]*8, np.uint8)
        data_buf[0:4] = float_to_uint8s(Vel_desired)
        self.__send_data(Motorid, data_buf)

    def control_pos_force(self, Motor, Pos_des: float, Vel_des, i_des):
        """
        Control the motor in hybrid position-force mode (EMIT).

        Args:
            Motor (Motor): Target motor object
            Pos_des (float): Desired position in radians
            Vel_des (float): Desired velocity (scaled ×100)
            i_des (float): Desired normalized current (0–10000)
        """
        if Motor.SlaveID not in self.Motors_map:
            print("control_pos_vel ERROR : Motor ID not found")
            return
        Motorid = 0x300 + Motor.SlaveID
        data_buf = np.array([0x00]*8, np.uint8)
        data_buf[0:4] = float_to_uint8s(Pos_des)
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xff
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xff
        data_buf[7] = ides_uint >> 8
        self.__send_data(Motorid, data_buf)
        self.recv()


    def control_pos_force2(self, Motor, Pos_des: float, Vel_des, i_des):
        """
        Control the motor in hybrid position-force mode (EMIT).

        Args:
            Motor (Motor): Target motor object
            Pos_des (float): Desired position in radians
            Vel_des (float): Desired velocity (scaled ×100)
            i_des (float): Desired normalized current (0–10000)
        """
        if Motor.SlaveID not in self.Motors_map:
            print("control_pos_vel ERROR : Motor ID not found")
            return
        Motorid = 0x300 + Motor.SlaveID
        data_buf = np.array([0x00]*8, np.uint8)
        data_buf[0:4] = float_to_uint8s(Pos_des)
        Vel_uint = np.uint16(Vel_des)
        ides_uint = np.uint16(i_des)
        data_buf[4] = Vel_uint & 0xff
        data_buf[5] = Vel_uint >> 8
        data_buf[6] = ides_uint & 0xff
        data_buf[7] = ides_uint >> 8
        self.__send_data(Motorid, data_buf)

    def enable(self, Motor):
        """
        Enable the motor by sending the enable command (0xFC).

        Args:
            Motor (Motor): Target motor object
        """
        self.__control_cmd(Motor, np.uint8(0xFC))
        sleep(0.1)
        self.recv()

    def enable_old(self, Motor, ControlMode):
        """
        Enable the motor with legacy firmware using adjusted control ID.

        Args:
            Motor (Motor): Target motor object
            ControlMode (int): Control mode index (e.g., MIT = 1)
        """
        data_buf = np.array([0xff] * 7 + [0xfc], np.uint8)
        enable_id = ((int(ControlMode) - 1) << 2) + Motor.SlaveID
        self.__send_data(enable_id, data_buf)
        sleep(0.1)
        self.recv()

    def disable(self, Motor):
        """
        Disable the motor by sending the disable command (0xFD).

        Args:
            Motor (Motor): Target motor object
        """
        self.__control_cmd(Motor, np.uint8(0xFD))
        sleep(0.1)
        self.recv()

    def set_zero_position(self, Motor):
        """
        Set the current motor position as the zero reference (0xFE).

        Args:
            Motor (Motor): Target motor object
        """
        self.__control_cmd(Motor, np.uint8(0xFE))
        sleep(0.1)
        self.recv()

    def recv(self):
        """
        Receive a single CAN frame and process it if valid.
        """
        data_recv = self.canbus.recv(0.1)
        if data_recv is not None:
            CANID = data_recv.data[0]
            CMD = 0x11  # Placeholder: fixed command type
            self.__process_packet(data_recv.data, CANID, CMD)

    # def recv_set_param_data(self):
        # """
        # Receive parameter data over CAN and process it.
        # """
        # data_recv = self.canbus.recv(0.1)
        # if data_recv is not None:
            # data = data_recv.data
            # CANID = data_recv.arbitration_id
            # CMD = 0x11  # Parameter response command
            # self.__process_packet(data, CANID, CMD)

            # print(f"[CAN] ID: 0x{CANID:03X}, CMD: 0x{CMD:02X}")

    def recv_set_param_data(self):
        """
        Receive parameter data over SocketCAN and process it using process_set_param_packet.
        """
        data_recv = self.canbus.recv(0.1)
        if data_recv is None:
            print("[WARN] No CAN data received.")
            return

        data = data_recv.data  # type: List[int] or bytes of length 8
        CANID = data_recv.arbitration_id
        # Fixed command is OK (CMD is handled separatelyand also is not used with SocketCAN)
        CMD = 0x11

        # process_set_param_packet internally checks data[2] == 0x33 or 0x55
        if len(data) >= 8:
            self.__process_set_param_packet(data, CANID, CMD)
            # print(f"[CAN] ID: 0x{CANID:03X}, CMD: 0x{CMD:02X}, DATA: {[hex(b) for b in data]}")
        else:
            print("[ERROR] CAN data too short.")
        
    def __process_packet(self, data, CANID, CMD):
        # print(f"CANID: {CANID}, CMD: {CMD}, Data: {data}")
        if CMD == 0x11:
            if CANID != 0x00:
                if CANID in self.Motors_map:
                    q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                    dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                    tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                    t_mos = data[6]
                    t_rotor = data[7]
                    MotorType_recv = self.Motors_map[CANID].MotorType
                    Q_MAX = self.Limit_Param[MotorType_recv][0]
                    DQ_MAX = self.Limit_Param[MotorType_recv][1]
                    TAU_MAX = self.Limit_Param[MotorType_recv][2]
                    recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                    self.Motors_map[CANID].recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor)
            else:
                MasterID=data[0] & 0x0f
                if MasterID in self.Motors_map:
                    q_uint = np.uint16((np.uint16(data[1]) << 8) | data[2])
                    dq_uint = np.uint16((np.uint16(data[3]) << 4) | (data[4] >> 4))
                    tau_uint = np.uint16(((data[4] & 0xf) << 8) | data[5])
                    t_mos = data[6]
                    t_rotor = data[7]
                    MotorType_recv = self.Motors_map[MasterID].MotorType
                    Q_MAX = self.Limit_Param[MotorType_recv][0]
                    DQ_MAX = self.Limit_Param[MotorType_recv][1]
                    TAU_MAX = self.Limit_Param[MotorType_recv][2]
                    recv_q = uint_to_float(q_uint, -Q_MAX, Q_MAX, 16)
                    recv_dq = uint_to_float(dq_uint, -DQ_MAX, DQ_MAX, 12)
                    recv_tau = uint_to_float(tau_uint, -TAU_MAX, TAU_MAX, 12)
                    self.Motors_map[MasterID].recv_data(recv_q, recv_dq, recv_tau, t_mos, t_rotor)

    def __process_set_param_packet(self, data, CANID, CMD):
        """
        Process a received parameter-setting packet.
        """
        if CMD == 0x11 and (data[2] == 0x33 or data[2] == 0x55):
            masterid = CANID
            slaveId = ((data[1] << 8) | data[0])
            if CANID == 0x00:
                masterid = slaveId

            if masterid not in self.Motors_map:
                if slaveId not in self.Motors_map:
                    return
                else:
                    masterid = slaveId

            RID = data[3]
            if is_in_ranges(RID):
                num = uint8s_to_uint32(data[4], data[5], data[6], data[7])
            else:
                num = uint8s_to_float(data[4], data[5], data[6], data[7])
            self.Motors_map[masterid].temp_param_dict[RID] = num

    def addMotor(self, Motor):
        """
        Add a motor to the internal control mapping.
        """
        self.Motors_map[Motor.SlaveID] = Motor
        if Motor.MasterID != 0:
            self.Motors_map[Motor.MasterID] = Motor
        return True

    def __control_cmd(self, Motor, cmd: np.uint8):
        """
        Send a control command to the specified motor.
        """
        data_buf = np.array([0xff]*7 + [cmd], np.uint8)
        self.__send_data(Motor.SlaveID, data_buf)

    def __send_data(self, Motor_id, data):
        """
        Send CAN data to the motor.
        """
        msg = can.Message(
            arbitration_id=Motor_id,
            data=data,
            is_extended_id=False,
            is_fd=self.use_canfd,
            is_remote_frame=False
        )
        self.canbus.send(msg)

    def __read_RID_param(self, Motor, RID):
        """
        Internal: send a request to read a specific motor parameter.
        """
        can_id_l = Motor.SlaveID & 0xff
        can_id_h = (Motor.SlaveID >> 8) & 0xff
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x33, np.uint8(RID)] + [0x00]*4, np.uint8)
        self.__send_data(0x7FF, data_buf)

    def __write_motor_param(self, Motor, RID, data):
        """
        Internal: write a value to a motor parameter.
        """
        can_id_l = Motor.SlaveID & 0xff
        can_id_h = (Motor.SlaveID >> 8) & 0xff
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0x55, np.uint8(RID)] + [0x00]*4, np.uint8)
        if not is_in_ranges(RID):
            data_buf[4:8] = float_to_uint8s(data)
        else:
            data_buf[4:8] = data_to_uint8s(int(data))
        self.__send_data(0x7FF, data_buf)

    def switchControlMode(self, Motor, ControlMode):
        """
        Switch the control mode of the motor.
        """
        max_retries = 20
        retry_interval = 0.1
        RID = 10
        self.__write_motor_param(Motor, RID, np.uint8(ControlMode))
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.Motors_map:
                if RID in self.Motors_map[Motor.SlaveID].temp_param_dict:
                    if abs(self.Motors_map[Motor.SlaveID].temp_param_dict[RID] - ControlMode) < 0.1:
                        return True
                    else:
                        return False
        return False

    def save_motor_param(self, Motor):
        """
        Save all parameters of the motor to flash memory.
        """
        can_id_l = Motor.SlaveID & 0xff
        can_id_h = (Motor.SlaveID >> 8) & 0xff
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xAA] + [0x00]*5, np.uint8)
        # pdb.set_trace()
        sleep(3)
        self.disable(Motor)
        sleep(3)
        self.__send_data(0x7FF, data_buf)
        sleep(3)

    def change_limit_param(self, Motor_Type, PMAX, VMAX, TMAX):
        """
        Change position, velocity, and torque limits for a motor type.
        """
        self.Limit_Param[Motor_Type][0] = PMAX
        self.Limit_Param[Motor_Type][1] = VMAX
        self.Limit_Param[Motor_Type][2] = TMAX

    def refresh_motor_status(self, Motor):
        """
        Request the latest status from the motor.
        """
        can_id_l = Motor.SlaveID & 0xff
        can_id_h = (Motor.SlaveID >> 8) & 0xff
        data_buf = np.array([np.uint8(can_id_l), np.uint8(can_id_h), 0xCC] + [0x00]*5, np.uint8)
        self.__send_data(0x7FF, data_buf)
        self.recv()

    def change_motor_param(self, Motor, RID, data):
        """
        Change a specific parameter value in the motor.
        """
        max_retries = 20
        retry_interval = 0.05
        self.__write_motor_param(Motor, RID, data)
        for _ in range(max_retries):
            self.recv_set_param_data()
            if Motor.SlaveID in self.Motors_map and RID in self.Motors_map[Motor.SlaveID].temp_param_dict:
                if abs(self.Motors_map[Motor.SlaveID].temp_param_dict[RID] - data) < 0.1:
                    return True
                else:
                    return False
            sleep(retry_interval)
        return False

    def read_motor_param(self, Motor, RID):
        """
        # Read a specific parameter value from the motor.
        # """
        max_retries = 5
        retry_interval = 0.005
        self.__read_RID_param(Motor, RID)
        for _ in range(max_retries):
            sleep(retry_interval)
            self.recv_set_param_data()
            if Motor.SlaveID in self.Motors_map:
                if RID in self.Motors_map[Motor.SlaveID].temp_param_dict:
                    return self.Motors_map[Motor.SlaveID].temp_param_dict[RID]
        return None

    # def read_motor_param(self, Motor, RID):
        # max_retries = 5
        # retry_interval = 0.05
        # print(f"[REQ] Sending read request: RID=0x{RID:02X} to Motor ID={Motor.SlaveID}")
        # self.__read_RID_param(Motor, RID)

        # for i in range(max_retries):
            # sleep(retry_interval)
            # self.recv_set_param_data()
            # if Motor.SlaveID in self.Motors_map:
                # if RID in self.Motors_map[Motor.SlaveID].temp_param_dict:
                    # val = self.Motors_map[Motor.SlaveID].temp_param_dict[RID]
                    # print(f"[OK] Received RID=0x{RID:02X} value: {val}")
                    # return val
                # else:
                    # print(f"[WAIT] RID 0x{RID:02X} not yet in temp_param_dict")
            # else:
                # print(f"[ERR] Motor ID {Motor.SlaveID} not in Motors_map")
                # print(f"[FAIL] Read RID 0x{RID:02X} failed after {max_retries} retries.")
        # return None

    def __extract_packets(self, data):
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        remainder_pos = 0

        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
                remainder_pos = i
            else:
                i += 1
        self.data_save = data[remainder_pos:]
        return frames


def LIMIT_MIN_MAX(x, min, max):
    if x <= min:
        x = min
    elif x > max:
        x = max


def float_to_uint(x: float, x_min: float, x_max: float, bits):
    LIMIT_MIN_MAX(x, x_min, x_max)
    span = x_max - x_min
    data_norm = (x - x_min) / span
    return np.uint16(data_norm * ((1 << bits) - 1))


def uint_to_float(x: np.uint16, min: float, max: float, bits):
    span = max - min
    data_norm = float(x) / ((1 << bits) - 1)
    temp = data_norm * span + min
    return np.float32(temp)


def float_to_uint8s(value):
    # Pack the float into 4 bytes
    packed = pack('f', value)
    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)


def data_to_uint8s(value):
    # Check if the value is within the range of uint32
    if isinstance(value, int) and (0 <= value <= 0xFFFFFFFF):
        # Pack the uint32 into 4 bytes
        packed = pack('I', value)
    else:
        raise ValueError("Value must be an integer within the range of uint32")

    # Unpack the bytes into four uint8 values
    return unpack('4B', packed)


def is_in_ranges(number):
    """
    check if the number is in the range of uint32
    :param number:
    :return:
    """
    if (7 <= number <= 10) or (13 <= number <= 16) or (35 <= number <= 36):
        return True
    return False


def uint8s_to_uint32(byte1, byte2, byte3, byte4):
    # Pack the four uint8 values into a single uint32 value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a uint32 value
    return unpack('<I', packed)[0]


def uint8s_to_float(byte1, byte2, byte3, byte4):
    # Pack the four uint8 values into a single float value in little-endian order
    packed = pack('<4B', byte1, byte2, byte3, byte4)
    # Unpack the packed bytes into a float value
    return unpack('<f', packed)[0]


def print_hex(data):
    hex_values = [f'{byte:02X}' for byte in data]
    print(' '.join(hex_values))


def get_enum_by_index(index, enum_class):
    try:
        return enum_class(index)
    except ValueError:
        return None



class DM_Motor_Type(IntEnum):
    DM3507 = 0
    DM4310 = 1
    DM4310_48V = 2
    DM4340 = 3
    DM4340_48V = 4
    DM6006 = 5
    DM8006 = 6
    DM8009 = 7
    DM10010L = 8
    DM10010 = 9
    DMH3510 = 10
    DMH6215 = 11
    DMG6220 = 12

class DM_variable(IntEnum):
    UV_Value = 0
    KT_Value = 1
    OT_Value = 2
    OC_Value = 3
    ACC = 4
    DEC = 5
    MAX_SPD = 6
    MST_ID = 7
    ESC_ID = 8
    TIMEOUT = 9
    CTRL_MODE = 10
    Damp = 11
    Inertia = 12
    hw_ver = 13
    sw_ver = 14
    SN = 15
    NPP = 16
    Rs = 17
    LS = 18
    Flux = 19
    Gr = 20
    PMAX = 21
    VMAX = 22
    TMAX = 23
    I_BW = 24
    KP_ASR = 25
    KI_ASR = 26
    KP_APR = 27
    KI_APR = 28
    OV_Value = 29
    GREF = 30
    Deta = 31
    V_BW = 32
    IQ_c1 = 33
    VL_c1 = 34
    can_br = 35
    sub_ver = 36
    u_off = 50
    v_off = 51
    k1 = 52
    k2 = 53
    m_off = 54
    dir = 55
    p_m = 80
    xout = 81


class Control_Type(IntEnum):
    MIT = 1
    POS_VEL = 2
    VEL = 3
    Torque_Pos = 4

# class DamiaoPort:
    # def __init__(self, device, types, can_ids, master_ids, motor_with_torque, control_mode=Control_Type.MIT, use_canfd = True):
        # self.device = device
        # self.types = types
        # self.can_ids = can_ids
        # self.master_ids = master_ids
        # self.control = MotorControl(self.device, use_canfd)
        # self.motors = [Motor(type, can_id, master_id) for type, can_id, master_id in zip(types, can_ids, master_ids)]
        # self.stat_data = []
        # self.stat_time = []
        # self.logger = logging.getLogger(self.__class__.__name__)
        # self.logger.setLevel(logging.INFO)

        # if not self.logger.handlers:
            # handler = logging.StreamHandler()
            # formatter = logging.Formatter('[%(levelname)s] %(name)s: %(message)s')
            # handler.setFormatter(formatter)
            # self.logger.addHandler(handler)

        # self.init_success = True
        
        # # Check lengths of lists
        # if not (len(types) == len(can_ids) == len(master_ids) == len(motor_with_torque)):
            # self.logger.error("Lengths of types, can_ids, master_ids, and motor_with_torque must match!")
            # self.init_success = False
            # return        

        # for idx, motor in enumerate(self.motors, start=1):
            # self.control.addMotor(motor)
            # mst_id = self.control.read_motor_param(motor, DM_variable.MST_ID)
            # esc_id = self.control.read_motor_param(motor, DM_variable.ESC_ID)
            # can_br = self.control.read_motor_param(motor, DM_variable.can_br)

            # if can_br == 9:      # canfd baudrate 5Mbps
                # self.logger.info(f"NOW CANFD MDOE")
            # elif can_br == 4:    # can2.0 baudrate 1Mbps
                # self.logger.info(f"NOW CAN2.0 MDOE")

            # motor_type_name = DM_Motor_Type(motor.MotorType).name
            # if mst_id == motor.MasterID and esc_id == motor.SlaveID:
                # self.logger.info(f"Motor type {motor_type_name} (MasterID: {hex(motor.MasterID)}, SlaveID: {hex(motor.SlaveID)}) found")
                # # self.control.enable(motor)
            # else:
                # self.logger.warning(f"Motor {idx}: type {motor_type_name} (MasterID: {hex(motor.MasterID)}, SlaveID: {hex(motor.SlaveID)}) not found, read MST_ID: {mst_id}, ESC_ID: {esc_id}")
                # self.logger.warning("Hint: Check CAN or CANFD configuration, bitrate, and ID assignments.")
                # for added_motor in self.motors:
                    # pass
                    # # self.control.disable(added_motor)
                # self.init_success = False
                # break  


class DamiaoPort:
    def __init__(self, device, types, can_ids, master_ids, control_mode=Control_Type.MIT, use_canfd=True):
        self.device = device
        self.types = types
        self.can_ids = can_ids
        self.master_ids = master_ids
        self.control = MotorControl(self.device, use_canfd)
        self.motors = [Motor(type, can_id, master_id) for type, can_id, master_id in zip(types, can_ids, master_ids)]
        self.stat_data = []
        self.stat_time = []
        self.logger = logging.getLogger(self.__class__.__name__)
        self.logger.setLevel(logging.INFO)

        if not self.logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('[%(levelname)s] %(name)s: %(message)s')
            handler.setFormatter(formatter)
            self.logger.addHandler(handler)

        self.init_success = True

        try:
            if not (len(types) == len(can_ids) == len(master_ids)):
                self.logger.error("Lengths of types, can_ids, master_ids must match!")
                self.init_success = False
                return

            for idx, motor in enumerate(self.motors, start=1):
                self.control.addMotor(motor)
                try:
                    mst_id = self.control.read_motor_param(motor, DM_variable.MST_ID)
                    esc_id = self.control.read_motor_param(motor, DM_variable.ESC_ID)
                    can_br = self.control.read_motor_param(motor, DM_variable.can_br)
                    if mst_id is None or esc_id is None or can_br is None:
                        self.logger.error(f"Error: Failed to read motor parameters for {idx}th motor (MotorType: {DM_Motor_Type(motor.MotorType).name}, MasterID: {hex(motor.MasterID)}, SlaveID: {hex(motor.SlaveID)})")
                        self.logger.warning("1. Check CAN communication.")
                        self.logger.warning("2. Confirm that MasterID and SlaveID are correct and motor is powered on.")
                        self.logger.warning("3. Supported CAN bitrates: CAN2.0 at 1Mbps, CANFD at 5Mbps.")
                        self.logger.warning("4. Verify wiring and connector status.")
                        self.init_success = False
                        return

                except Exception as e:
                    self.logger.error(f"Error reading motor parameters: {e}")
                    self.init_success = False
                    return

                if can_br == 9:
                    self.logger.info("NOW CANFD MODE")
                elif can_br == 4:
                    self.logger.info("NOW CAN2.0 MODE")
                else:
                    self.logger.warning("Supported CAN bitrates are: CAN2.0 at 1Mbps and CANFD at 5Mbps.")

                motor_type_name = DM_Motor_Type(motor.MotorType).name
                if mst_id == motor.MasterID and esc_id == motor.SlaveID:
                    self.logger.info(f"Motor type {motor_type_name} (MasterID: {hex(motor.MasterID)}, SlaveID: {hex(motor.SlaveID)}) found")
                else:
                    master_status = "OK" if mst_id == motor.MasterID else f"Mismatch (expected {hex(motor.MasterID)}, got {hex(mst_id)})"
                    slave_status = "OK" if esc_id == motor.SlaveID else f"Mismatch (expected {hex(motor.SlaveID)}, got {hex(esc_id)})"
                    self.logger.warning(
                    f"{idx}th motor : type {motor_type_name} "
                    f"(MasterID: {hex(motor.MasterID)}, SlaveID: {hex(motor.SlaveID)}) not found. "
                    f"MasterID status: {master_status}, SlaveID status: {slave_status}"
                    )
                    self.logger.warning("Hint: Check CAN or CANFD configuration, bitrate, and ID assignments.")
                    for added_motor in self.motors:
                        pass
                    self.init_success = False
                    return

        except Exception as e:
            self.logger.error(f"Unexpected error in DamiaoPort init: {e}")
            self.init_success = False


    def get_present_status(self):
        self.stat_time.append(time())
        stat = [[
            Motor.goal_position,
            Motor.goal_tau,
            Motor.getPosition(),
            Motor.getVelocity(),
            Motor.getTorque(),
            Motor.state_tmos,
            Motor.state_trotor,
        ] for Motor in self.motors]
        self.stat_data.append(stat)

        return stat

    def save_status(self, filename):
        np.savez(filename, np.array(self.stat_time), np.array(self.stat_data))

    def enable(self):
        for Motor in self.motors:
            self.control.enable(Motor)

    def disable(self):
        for Motor in self.motors:
            self.control.disable(Motor)

    def shutdown(self):
        for Motor in self.motors:
            self.control.controlMIT(Motor, 0, 0, 0, 0, 0)
        # self.control.canbus.shutdown()

    def set_zero_position(self):
        for Motor in self.motors:
            self.control.disable(Motor)
        sleep(1)
        for Motor in self.motors:
            self.control.set_zero_position(Motor)
        sleep(1)
        for Motor in self.motors:
            self.control.enable(Motor)
        return 0

    def set_goal_torque_sync(self, goal_taus):
        for Motor, goal_tau in zip(self.motors, goal_taus):
            Motor.goal_position = 0
            Motor.goal_tau = goal_tau
            self.control.controlMIT(Motor, 0, 0, 0, 0, Motor.goal_tau)
            sleep(0.00003)

    def move_torque_sync(self, taus):
        for Motor,tau in zip(self.motors, taus):
            Motor.goal_position = 0
            Motor.goal_tau = tau
            self.control.controlMIT(Motor, 0, 0, 0, 0, Motor.goal_tau)
            sleep(0.00003)

    def keep_torque_sync(self):
        for Motor in self.motors:
            self.control.controlMIT(Motor, 0, 0, 0, 0, Motor.goal_tau)
            sleep(0.00003)

    async def set_goal_positions(self, goal_positions, kps):
        for Motor, goal_position, kp in zip(self.motors, goal_positions, kps):
            Motor.goal_position = goal_position
            Motor.goal_tau = 0
            self.control.controlMIT(Motor, kp, 1.2, goal_position, 0, 0)
            await asyncio.sleep(0.00003)

    def set_goal_positions_sync(self, goal_positions, kps, kds):
        for Motor, goal_position, kp, kd in zip(self.motors, goal_positions, kps, kds):
            Motor.goal_position = goal_position
            Motor.goal_tau = 0
            self.control.controlMIT(Motor, kp, kd, goal_position, 0, 0)
            sleep(0.00003)

    def set_goal_posvel(self, goal_positions):
        for Motor, goal_position in zip(self.motors, goal_positions):
            Motor.goal_position = goal_position
            Motor.goal_tau = 0
            self.control.control_pos_force(Motor, goal_position, 1, 1)

    def setPosVel(self, goal_positions, goal_velocities):
        for Motor, goal_position, goal_velocity in zip(
            self.motors, goal_positions, goal_velocities):
            Motor.goal_position = goal_position
            Motor.goal_velocity = goal_velocity
            self.control.control_Pos_Vel(Motor, goal_position, goal_velocity)

    def setPosVelSync(self, goal_positions, goal_velocities):
        for Motor, goal_position, goal_velocity in zip(
            self.motors, goal_positions, goal_velocities):
            Motor.goal_position = goal_position
            Motor.goal_velocity = goal_velocity
            self.control.control_Pos_Vel2(Motor, goal_position, goal_velocity)
        sleep(0.00003)
        for _ in self.motors:
            self.control.recv()

    def setPosForce(self, goal_positions, goal_velocities, goal_taus): 
        for Motor, goal_position, goal_velocity, goal_tau in zip(
            self.motors, goal_positions, goal_velocities, goal_taus):
                self.control.control_pos_force(Motor, goal_position, goal_position, goal_tau)

    def setPosForceSync(self, goal_positions, goal_velocities, goal_taus): 
        for Motor, goal_position, goal_velocity, goal_tau in zip(
            self.motors, goal_positions, goal_velocities, goal_taus):
                self.control.control_pos_force2(Motor, goal_position, goal_position, goal_tau)
        sleep(0.00003)
        for _ in self.motors:
            self.control.recv()

    def setVel(self, goal_velocities):
        for Motor, goal_velocity in zip(self.motors, goal_velocities):
            Motor.goal_velocities = goal_velocities
            self.control.control_Vel(Motor, goal_velocity)

    def setVelSync(self, goal_velocities): 
        for Motor, goal_velocity in zip(self.motors, goal_velocities):
            Motor.goal_velocity = goal_velocity
            self.control.control_Vel2(Motor, goal_velocity)
        sleep(0.00003)
        for _ in self.motors:
            self.control.recv()

    def setMIT(self, goal_positions, goal_velocities, kps, kds,goal_taus):
        for Motor, goal_position, goal_velocity, kp, kd, tau in zip(
            self.motors, goal_positions, goal_velocities, kps, kds, goal_taus):
                self.control.controlMIT(Motor, kp, kd, goal_position, goal_position, tau)

    def setMITSync(self, goal_positions, goal_velocities, kps, kds,goal_taus):
        for Motor, goal_position, goal_velocity, kp, kd, tau in zip(
            self.motors, goal_positions, goal_velocities, kps, kds, goal_taus):
            Motor.goal_position = goal_position
            Motor.goal_velocity = goal_velocity
            Motor.goal_tau = tau
            self.control.controlMIT2(Motor, kp, kd, goal_position, goal_velocity, tau)
        sleep(0.00003)
        for _ in self.motors:
            self.control.recv()
