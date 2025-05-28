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

from datetime import datetime
from DM_SocketCANFD import *
import numpy as np
from numpy import pi
import time

openarm_DEVICENAME0 = "can0"
TICK = 0.01
POSE0 = [0, 0, 0, 0, 0, 0, 0, 0]
K0 = [0, 0, 0, 0, 0, 0, 0, 0]
KP1 = [35, 35, 25, 33, 6, 5, 4, 0]
KD = [1.2, 3.8, 1.0, 0.5, 0.2, 0.2, 0.2, 0]

def record(openarm):
    pose = POSE0
    openarm.set_goal_positions_sync(pose, KP1, KD)
    time.sleep(3)
    print("ready")
    t0 = datetime.now().timestamp()
    t_schedule = t0
    i = 0
    while True:
        # openarm.move_towards_sync(pose, K0, K0)
        openarm.setMIT(pose, [0]*8, K0, K0, [0]*8)
        openarm.get_present_status()
        t_schedule += TICK
        td = t_schedule - datetime.now().timestamp()
        if td > 0:
            time.sleep(td)
        else:
            print("timeout", td)
        i += 1
        if i % 6000 == 0:
            print(t_schedule - t0)

if __name__ == "__main__":
    import click
    @click.command()
    @click.option('--set_zero_position', is_flag=True, default=False)
    def main(set_zero_position):
        save = "record00.npz"
        openarm = DamiaoPort(openarm_DEVICENAME0,
                              [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4310, DM_Motor_Type.DM4310,
                               DM_Motor_Type.DM4310, DM_Motor_Type.DM3507],
                              [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
                              [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18],
                              [True, True, True, True, True, True, True, True],
                              bitrate = 1000000,
                              data_bitrate = 5000000,
                              use_canfd = True)

        # check initialise has completed
        if not openarm.init_success:
            print("Error: Initialization failed due to mOtor ID mismatch.")
            print("All motors disabled and CAN bus shutdown.")
            import sys
            sys.exit(1)
        else:
            print("Initialization successful")

        if set_zero_position:
            openarm.set_zero_position()
            return openarm.disable()
        try:
            record(openarm)
        except KeyboardInterrupt:
            print("key pressed")
        openarm.set_goal_positions_sync(POSE0, KP1, KD)
        if save:
            openarm.save_status(save)
        time.sleep(3)
        openarm.disable()
        print("done")
    main()
