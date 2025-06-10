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
Kp = [0]
Kd = [0]
# Pos = [np.pi*2]
Pos = [0]
Vel = [0]
Tau = [1.0]

zeros = np.zeros(NUMJOINT)

if __name__ == "__main__":

    def main():
        openarm = DamiaoPort(
            CAN_DEVICE_NAME,
            [
                DM_Motor_Type.DM4340
            ],
            [0x08],
            [0x18],
            use_canfd = True
        )

        openarm.enable()

        try:
            while True:
                start = time.perf_counter()

                # openarm.set_goal_positions_sync(Pose, Kp, Kd)
                # openarm.setMIT(Kp, Kd, Pos, Vel, Tau)
                # openarm.set_goal_posvel(Pos)
                openarm.setMITSync(Kp, Kd, Pos, Vel, Tau)
                # openarm.setVel(Vel)
                # openarm.setVelSync(Vel)
                # openarm.setPosVel(Pos,Vel)
                # openarm.setPosVelSync(Pos,Vel)
                # openarm.setPosForce(Pos,Vel,Tau)
                # openarm.setPosForceSync(Pos, Vel, Tau)

                status = openarm.get_present_status()

                for i, motor_status in enumerate(status):
                    print(f"[Motor {i}] goal_pos: {motor_status[0]:.3f}, "
                          f"goal_tau: {motor_status[1]:.3f}, "
                          f"pos: {motor_status[2]:.3f}, "
                          f"vel: {motor_status[3]:.3f}, "
                          f"tau: {motor_status[4]:.3f}, "
                          f"tmos: {motor_status[5]}, "
                          f"trotor: {motor_status[6]}")
                
                end = time.perf_counter()
                elapsed = end - start
                freq = 1.0 / elapsed if elapsed > 0 else 0.0

                print(f"Loop time: {elapsed*1000:.3f} ms, Frequency: {freq:.1f} Hz")
                print("="*100)

            

        except KeyboardInterrupt:
            print("Interrupted by user. Disabling motors...")
            openarm.disable()
            print("Shutdown complete.")

    main()

