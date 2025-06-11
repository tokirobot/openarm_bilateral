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
import numpy as np
from urdf_parser_py.urdf import URDF
from ament_index_python.packages import get_package_share_directory
import os
import sys

openarm_DEVICENAME = "can0"
POSE0 = [0, 0, 0, 0, 0, 0, 0, 0]
K0 = [0, 0, 0, 0, 0, 0, 0, 0]

JOINT_LIMITS = {
    motor_id: {"lower": -np.pi, "upper": np.pi}
    for motor_id in [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]
}

def load_joint_limits_from_urdf():
    description_path = get_package_share_directory("openarm_bimanual_description")
    urdf_path = os.path.join(description_path, "urdf", "openarm_bimanual.urdf")
    joint_limits = {}

    try:
        robot = URDF.from_xml_file(urdf_path)
    except Exception as e:
        print(f"Warning: Could not load URDF file. Reason: {e}")
        print("Hint: Make sure 'openarm_bimanual_description' is built and sourced.")
        return {}, {}

    right_limits = {}
    left_limits = {}

    for joint in robot.joints:
        if joint.limit:
            if joint.name.startswith("right_"):
                right_limits[joint.name] = {
                    "lower": joint.limit.lower,
                    "upper": joint.limit.upper
                }
            elif joint.name.startswith("left_"):
                left_limits[joint.name] = {
                    "lower": joint.limit.lower,
                    "upper": joint.limit.upper
                }

    return right_limits, left_limits

if __name__ == "__main__":
    @click.command()
    @click.option('--set_zero_position', is_flag=True, default=False, help="Load joint limits from file.")
    @click.option('--load_limits', is_flag=True, default=False, help="Load joint limits from file.")
    def main(set_zero_position, load_limits):

        if not set_zero_position and not load_limits:
            print("Hint: Please specify at least one of the options: --set_zero_position or --load_limits.")
            print("Example:")
            print("  python calibrate.py --set_zero_position")
            print("  or")
            print("  python calibrate.py --load_limits")
            return

        print(f"using {openarm_DEVICENAME}")
        openarm = DamiaoPort(openarm_DEVICENAME,
                              [DM_Motor_Type.DM4340],
                              [ 0x08],
                              [ 0x18],
                              use_canfd = True
                              )
       
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
        
        #check openarm init_success flag is true or not
        if openarm.init_success == True:
            print("openarm init success")
        else:
            print("openarm init failed")
            sys.exit(1)

        openarm.enable()

        if set_zero_position:
            openarm.disable()
            openarm.set_zero_position()
            print("Zero position has been set and torque disabled. Exiting.")
            return openarm.disable()

        if load_limits:
            right_limits, left_limits = load_joint_limits_from_urdf()

            if right_limits:
                print("Right arm joint limits:")
                for joint_id, limit in right_limits.items():
                    print(f"  {joint_id}: Lower: {limit['lower']}, Upper: {limit['upper']}")

            if left_limits:
                print("Left arm joint limits:")
                for joint_id, limit in left_limits.items():
                    print(f"  {joint_id}: Lower: {limit['lower']}, Upper: {limit['upper']}")


        openarm.shutdown()

    main()

