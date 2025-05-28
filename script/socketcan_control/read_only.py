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

Kp = [0, 0, 0, 0, 0, 0, 0, 0]
Kd = [0, 0, 0, 0, 0, 0, 0, 0]
Pose = [0, 0, 0, 0, 0, 0, 0, 0]

zeros = np.zeros(8)

if __name__ == "__main__":

    def main():
        openarm = DamiaoPort(
            CAN_DEVICE_NAME,
            [
                DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                DM_Motor_Type.DM4310, DM_Motor_Type.DM4310,
                DM_Motor_Type.DM4310, DM_Motor_Type.DM3507
            ],
            [0x01, 0x02,  0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
            [0x11, 0x12,  0x13, 0x14, 0x15, 0x16, 0x17, 0x18],
            [True, True, True, True, True, True, True, True],
            bitrate = 1000000,
            data_bitrate = 5000000,
            use_canfd = True
        )

        # check initialise has completed
        if not openarm.init_success:
            print("Error: Initialization failed due to mOtor ID mismatch.")
            print("All motors disabled and CAN bus shutdown.")
            import sys
            sys.exit(1)
        else:
            print("Initialization successful")

        try:
            while True:
                openarm.set_goal_positions_sync(Pose, Kp, Kd)
                # openarm.controlMIT(zeros, zeros, zeros, zeros, zeros)
                status = openarm.get_present_status()

                for i, motor_status in enumerate(status):
                    print(f"[Motor {i}] goal_pos: {motor_status[0]:.3f}, "
                          f"goal_tau: {motor_status[1]:.3f}, "
                          f"pos: {motor_status[2]:.3f}, "
                          f"vel: {motor_status[3]:.3f}, "
                          f"tau: {motor_status[4]:.3f}, "
                          f"tmos: {motor_status[5]}, "
                          f"trotor: {motor_status[6]}")
                sleep(0.001)
                print("="*90)
        except KeyboardInterrupt:
            print("Interrupted by user. Disabling motors...")
            openarm.disable()
            print("Shutdown complete.")

    main()

