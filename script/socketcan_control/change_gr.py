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
import click

CAN_DEVICE_NAME = "can0"
SAVE_MOTOR_PARAMETER_PERMANENTLY =True

if __name__ == "__main__":
    print(
        "WARNING: setting the baudrate above 1Mbps requires specialized hardware supporting CAN-FD."
    )
    print(f"flashing {CAN_DEVICE_NAME}")


    def main():

        # openarm = DamiaoPort(
            # CAN_DEVICE_NAME,
            # [
                # DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                # DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                # DM_Motor_Type.DM4310, DM_Motor_Type.DM4310,
                # DM_Motor_Type.DM4310, DM_Motor_Type.DM3507
            # ],
            # [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            # [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18],
            # use_canfd =True
        # )

        openarm = DamiaoPort(
            CAN_DEVICE_NAME,
            [
                DM_Motor_Type.DM4310
            ],
            [0x05],
            [0x15],
            use_canfd =True
        )
        
        openarm.enable()
        addr = DM_variable.Gr
        gr_desire = 2
        print("before Gr (Gear ratio):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.Gr))

        for i, motor in enumerate(openarm.motors, start=1):
            openarm.control.change_motor_param(motor, addr, gr_desire)
            if SAVE_MOTOR_PARAMETER_PERMANENTLY:
                print(f"saving motor parameter permanentaly for motor {i}")
                openarm.control.save_motor_param(motor)
            print("Gr (Gear ratio):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.Gr))
        openarm.disable()
        print("done")
    main()
