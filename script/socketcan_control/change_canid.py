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
SAVE_MOTOR_PARAMETER_PERMANENTLY = False


@click.command()
@click.option('--slaveid', type=str, required=False, help="Motor CAN ID (SlaveID), e.g., 0x01 or 1")
@click.option('--masterid', type=str, required=False, help="Master CAN ID (MasterID), e.g., 0x12 or 18")
if __name__ == "__main__":
        if not slaveid or not masterid:
            print("That is wrong usage!!!!")
            return

        openarm = DamiaoPort(
            CAN_DEVICE_NAME,
            [
                DM_Motor_Type.DM3507
            ],
            [slaveid],
            [masterid],
            [True],
            bitrate = 1000000,
            data_bitrate = 5000000,
            use_canfd =True
        )

        mst_addr = DM_variable.MST_ID
        esc_addr = DM_variable.ESC_ID

        changed_mst_id = 0x08
        changed_esc_id = 0x18

        for i, motor in enumerate(openarm.motors, start=1):
            openarm.control.read_motor_param(motor,mst_addr)
            openarm.control.read_motor_param(motor,esc_addr)
            openarm.control.change_motor_param(motor, mst_addr, changed_mst_id)
            openarm.control.change_motor_param(motor, esc_addr, changed_esc_id)
            if SAVE_MOTOR_PARAMETER_PERMANENTLY:
                print(f"saving motor parameter permanentaly for motor {i}")
                openarm.control.save_motor_param(motor)
            openarm.control.read_motor_param(motor,mst_addr)
            openarm.control.read_motor_param(motor, esc_addr)
        openarm.disable()
        print("done")
    main()
