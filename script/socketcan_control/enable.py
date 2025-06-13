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

CAN_DEVICE_NAME = "can1"

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
            use_canfd = True
        )

        #check openarm init_success flag is true or not
        if openarm.init_success == True:
            print("openarm init success")
        else:
            print("openarm init failed")
            sys.exit(1)

        openarm.enable()
        print("openarm enabled !!!!!")

        sleep(3)

        openarm.disable()
        print("openarm disabled!")

    main()


