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
KP = [35, 35, 25, 33, 6, 5, 4, 0]
KD = [1.2, 3.8, 1.0, 0.5, 0.2, 0.2, 0.2, 0]

def current_pose(openarm):
    if len(openarm.stat_data) == 0:
        return False
    current_stat = np.array(openarm.stat_data[-1])
    return current_stat[:,2]

def replay(openarm, logfile, delay=8):
    time_, data = np.load(logfile).values()
    print(data.shape)
    pose = POSE0
    poses = data[:,:,2]
    openarm.set_goal_positions_sync(pose, KP, KD)
    time.sleep(3)
    t0 = datetime.now().timestamp()
    t_schedule = t0
    for n in range(2):
        for r in range(5):
            #steps = 2 ** r
            steps = r + 1
            for i in range(len(poses)-1):
                p0 = poses[i]
                p1 = poses[i+1]
                for t in np.linspace(0, 1, steps):
                    pose = t * p0 + (1-t) * p1
                    openarm.setMIT(pose, [0]*8, KP, KD, [0]*8)
                    openarm.get_present_status()
                    t_schedule += TICK
                    td = t_schedule - datetime.now().timestamp()
                    if td > 0:
                        time.sleep(td)
                    else:
                        print("timeout", td)
            print("replay finished {} {}".format(n, r))

if __name__ == "__main__":
    import click
    @click.command()
    @click.option('--set_zero_position', is_flag=True, default=False)
    @click.option('--logfile', default="record00.npz")
    def main(set_zero_position, logfile):
        openarm = DamiaoPort(openarm_DEVICENAME0,
                              [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                               DM_Motor_Type.DM4310, DM_Motor_Type.DM4310,
                               DM_Motor_Type.DM4310, DM_Motor_Type.DM3507],
                              [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08],
                              [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18],
                              use_canfd = True
                              )

        #check openarm init_success flag is true or not
        if openarm.init_success == True:
            print("openarm init success")
        else:
            print("openarm init failed")
            sys.exit(1)
        
        openarm.enable()

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
            replay(openarm, logfile)
        except KeyboardInterrupt:
            print("key pressed")
        openarm.set_goal_positions_sync(POSE0, KP, KD)
        save = datetime.now().strftime("%Y%m%d%H%M%S")
        openarm.save_status(save)
        time.sleep(3)
        openarm.disable()
        print("done")
    main()
