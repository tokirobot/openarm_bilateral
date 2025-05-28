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
from datetime import datetime
import click
import json
import time
from urdf_parser_py.urdf import URDF
import os
from ament_index_python.packages import get_package_share_directory

openarm_DEVICENAME0 = "can0"

TICK = 0.01
POSE0 = [0, 0, 0, 0, 0, 0, 0]
KP1 = [35, 35, 25, 33, 6, 5, 4]
KD = [1.2, 3.8, 1.0, 0.5, 0.2, 0.2, 0.2]


def get_current_positions(openarm):
    """
    Retrieve the current joint positions of all motors.
    """
    status = openarm.get_present_status()  # Get motor statuses
    current_positions = np.array([motor_status[2] for motor_status in status])  # Extract positions
    return current_positions

def load_joint_limits_from_urdf():
    description_path = get_package_share_directory("openarm_bimanual_description")
    urdf_path = os.path.join(description_path, "urdf", "openarm_bimanual.urdf")
    joint_limits = {}

    try:
        robot = URDF.from_xml_file(urdf_path)
    except Exception as e:
        print(f"Warning: Could not load URDF file. Reason: {e}")
        print("Hint: Make sure 'openarm_bimanual_description' is built and sourced.")
        return {
            1: {"lower": -2.348, "upper": 2.377},
            2: {"lower": -0.139, "upper": 2.996},
            3: {"lower": -2.066, "upper": 2.113},
            4: {"lower": -0.453, "upper": 2.637},
            5: {"lower": -2.175, "upper": 0.105},
            6: {"lower": -1.473, "upper": 0.625},
            7: {"lower": -2.0, "upper": 2.0},
        }

    rev_joints = [joint for joint in robot.joints if "rev" in joint.name]
    rev_joints.sort(key=lambda j: j.name)

    for idx, joint in enumerate(rev_joints, start=1):
        if joint.limit:
            joint_limits[idx] = {
                "lower": joint.limit.lower,
                "upper": joint.limit.upper,
            }

    return joint_limits



def validate_goal_positions(goal_positions, joint_limits):
    """
    Validate that all goal positions are within the joint limits.
    Ensures the goal position lies between the lower and upper limits,
    regardless of whether the limits are positive or negative.
    """
    for motor_id, goal_position in enumerate(goal_positions, start=1):  # Iterate over array
        if motor_id not in joint_limits:
            print(f"Warning: No joint limits found for Motor {motor_id}. Skipping validation.")
            continue

        lower = joint_limits[motor_id]["lower"]
        upper = joint_limits[motor_id]["upper"]

        # Determine the actual range (min, max) to validate against
        limit_min, limit_max = min(lower, upper), max(lower, upper)

        if not (limit_min <= goal_position <= limit_max):
            print(f"Error: Goal position for Motor {motor_id} ({goal_position:.4f}) "
                  f"is out of bounds (Valid Range: {limit_min:.4f} to {limit_max:.4f}).")
            return False

    return True


def hold(openarm, goal_positions):
    print("Holding position. Press Ctrl+C to return to zero position.")
    while True:
        openarm.setMITSync(goal_positions, [0] * 7, KP1, KD, [0] * 7)
        openarm.get_present_status()
        time.sleep(TICK)  # Maintain the position at regular intervals

def move(openarm, start, end):
    # openarm.set_goal_positions_sync(start, KP1, KD)
    time.sleep(3)

    p0 = np.array(start)
    p1 = np.array(end)

    t0 = datetime.now().timestamp()
    t_schedule = t0
    for t in np.linspace(0, 1, 1000):
        pose = (1-t) * p0 + t * p1
        openarm.setMITSync(pose, [0] * 7, KP1, KD, [0] * 7)
        openarm.get_present_status()
        t_schedule += TICK
        td = t_schedule - datetime.now().timestamp()
        if td > 0:
            time.sleep(td)
        else:
            print("timeout", td)
    
def move_to_zero(openarm, joint_limits):

    p0 = np.array(get_current_positions(openarm))
    p1 = np.array(POSE0)

    if not validate_goal_positions(p0, joint_limits):
        print("Current robot positions out of joint limits")
        return
    else:
        t0 = datetime.now().timestamp()
        t_schedule = t0
        for t in np.linspace(0, 1, 1000):
            pose = (1-t) * p0 + t * p1
            openarm.setMITSync(pose, [0] * 7, KP1, KD, [0] * 7)
            openarm.get_present_status()
            t_schedule += TICK
            td = t_schedule - datetime.now().timestamp()
            if td > 0:
                time.sleep(td)
            else:
                print("timeout", td)

if __name__ == "__main__":

    @click.command()
    @click.option('--goal_positions', type=float, nargs=7, required=True, help="Specify 7 joint positions, space separated. E.g. --goal_positions 0.5 0.0 0.0 0.0 0.0 0.0 0.0")

    def main (goal_positions):
        # Initialize motor controller
        openarm = DamiaoPort(openarm_DEVICENAME0,
                                [DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                                DM_Motor_Type.DM4340, DM_Motor_Type.DM4340,
                                DM_Motor_Type.DM4310, DM_Motor_Type.DM4310,
                                DM_Motor_Type.DM4310],
                                [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
                                [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17],
                                [True, True, True, True, True, True, True],
                                bitrate = 1000000,
                                data_bitrate = 5000000,
                                use_canfd = True
                                )

        # Load joint limits
        joint_limits = load_joint_limits_from_urdf()
        goal_positions = list(goal_positions)
        print(f"Goal positions: {goal_positions}")

        if not validate_goal_positions(goal_positions, joint_limits):
            print("One or more goal positions are invalid. Exiting.")
            openarm.disable()
            return

        try:
            move(openarm, POSE0, goal_positions)
        except KeyboardInterrupt:
            print("Key Pressed")
            move_to_zero(openarm, joint_limits)
        
        try:
            hold(openarm, goal_positions)
        except KeyboardInterrupt:
            print("Returning to Zero Position")
            move_to_zero(openarm, joint_limits)
            
        print("Synchronous movement complete.")
        # Disable motors after movement
        print("Disabling motors and performing CANbus shutdown")
        openarm.disable()

    main()
