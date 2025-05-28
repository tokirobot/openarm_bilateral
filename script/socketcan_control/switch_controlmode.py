#!/usr/bin/env python3
#
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

from DM_SocketCANFD import *
from time import sleep
import numpy as np
from numpy import pi
import time

CAN_DEVICE_NAME = "can0"
NUMJOINT = 1

SAVE_MOTOR_PARAMETER_PERMANENTLY = False

def choose_control_mode():
    print("Select control mode:")
    print(" 1: MIT")
    print(" 2: POS_VEL")
    print(" 3: VEL")
    print(" 4: Torque_Pos")

    while True:
        try:
            mode = int(input("Enter mode number: ").strip())
            if mode in [1, 2, 3, 4]:
                return Control_Type(mode)
            else:
                print("Invalid input. Please enter 1, 2, 3, or 4.")
        except ValueError:
            print("Invalid input. Please enter a number.")

def main():
    openarm = DamiaoPort(
        CAN_DEVICE_NAME,
        [
            DM_Motor_Type.DM4340
        ],
        [0x02],
        [0x12],
        [True],
        bitrate=1000000,
        data_bitrate=5000000,
        use_canfd=True
    )

    control_mode = choose_control_mode()

    for i, motor in enumerate(openarm.motors, start=1):
        # openarm.control.read_motor_param(motor, DM_variable.CTRL_MODE)
        # openarm.control.change_motor_param(motor, DM_variable.CTRL_MODE, control_mode)
        if openarm.control.switchControlMode(motor, control_mode):
            print(f"[Motor {i}] Control mode switch to {control_mode.name} succeeded!")
            if SAVE_MOTOR_PARAMETER_PERMANENTLY:
                openarm.control.save_motor_param(motor)
                print("saved motor control mode ")
                print( "CAUTION: Motor parameters can be modified up to 10,000 times. Be sure not to use this command too often!")
        else:
            print(f"[Motor {i}] Control mode switch failed.")

    openarm.disable()
    print("done!!!!!")

if __name__ == "__main__":
    main()
