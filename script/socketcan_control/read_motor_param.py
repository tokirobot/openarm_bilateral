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

import time
import numpy as np
from DM_SocketCANFD import *
from enum import IntEnum
import click
import sys
CAN_DEVICE_NAME = "can0"
NUMJOINT = 1
zeros = np.zeros(NUMJOINT)

def parse_hex_or_dec(value):
    return int(value, 0)

@click.command()
@click.option('--slaveid', type=str, required=False, help="Motor CAN ID (SlaveID), e.g., 0x01 or 1")
@click.option('--masterid', type=str, required=False, help="Master CAN ID (MasterID), e.g., 0x12 or 18")
def main(slaveid, masterid):
    if not slaveid or not masterid:
        print("That is wrong usage!!!!")
        print("Usage Example: python3 read_motor_param.py --slaveid 0x01 --masterid 0x11")
        return

    slaveid_int = parse_hex_or_dec(slaveid)
    masterid_int = parse_hex_or_dec(masterid)

    print("Hint: Before using DamiaoPort, please specify the motor's masterid and slaveid with --masterid and --slaveid.")
    print(f"  For example: python3 script.py --slaveid {slaveid} --masterid {masterid}")
    print(f"Parsed IDs: SlaveID={slaveid_int} (0x{slaveid_int:02X}), MasterID={masterid_int} (0x{masterid_int:02X})")

    openarm = DamiaoPort(
        CAN_DEVICE_NAME,
        [DM_Motor_Type.DM3507],
        [slaveid_int],
        [masterid_int],
        use_canfd=True
    )
    
    #check openarm init_success flag is true or not
    if openarm.init_success == True:
        print("openarm init success")
    else:
        print("openarm init failed")
        sys.exit(1)
   
    openarm.enable()

    print("---- Start Reading All Motor Parameters (original order) ----")

    # 0–10
    print("UV_Value (Undervoltage protection threshold):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.UV_Value))
    print("KT_Value (Torque constant [Nm/A]):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.KT_Value))
    print("OT_Value (Overtemperature threshold [°C]):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.OT_Value))
    print("OC_Value (Overcurrent threshold [A]):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.OC_Value))
    print("ACC (Acceleration limit):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.ACC))
    print("DEC (Deceleration limit):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.DEC))
    print("MAX_SPD (Maximum speed):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.MAX_SPD))
    mst_id = openarm.control.read_motor_param(openarm.motors[0], DM_variable.MST_ID)
    esc_id = openarm.control.read_motor_param(openarm.motors[0], DM_variable.ESC_ID)

    print(f"MST_ID (Master CAN ID): {mst_id} (0x{int(mst_id):02X})")
    print(f"ESC_ID (Motor/ESC ID): {esc_id} (0x{int(esc_id):02X})")
    print("TIMEOUT (Communication timeout):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.TIMEOUT))
    print("CTRL_MODE (Control mode: 1=MIT, 2=POS_VEL, 3=VEL, 4=Torque_Pos):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.CTRL_MODE))

    # 11–20
    print("Damp (Damping coefficient):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.Damp))
    print("Inertia (Rotor inertia):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.Inertia))
    print("hw_ver (Hardware version):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.hw_ver))
    print("sw_ver (Software version):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.sw_ver))
    print("SN (Serial number):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.SN))
    print("NPP (Number of pole pairs):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.NPP))
    print("Rs (Stator resistance):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.Rs))
    print("LS (Stator inductance):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.LS))
    print("Flux (Magnetic flux linkage):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.Flux))
    print("Gr (Gear ratio):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.Gr))

    # 21–36
    print("PMAX (Maximum power or torque):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.PMAX))
    print("VMAX (Maximum velocity):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.VMAX))
    print("TMAX (Maximum torque):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.TMAX))
    print("I_BW (Current loop bandwidth [Hz]):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.I_BW))
    print("KP_ASR (Speed loop P gain):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.KP_ASR))
    print("KI_ASR (Speed loop I gain):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.KI_ASR))
    print("KP_APR (Position loop P gain):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.KP_APR))
    print("KI_APR (Position loop I gain):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.KI_APR))
    print("OV_Value (Overvoltage protection threshold):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.OV_Value))
    print("GREF (Gain reference or scaling):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.GREF))
    print("Deta (Delta coefficient - custom use):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.Deta))
    print("V_BW (Velocity loop bandwidth):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.V_BW))
    print("IQ_c1 (Current limit or scaling factor):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.IQ_c1))
    print("VL_c1 (Voltage clamp or limit):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.VL_c1))
    print("CAN_BAUDRATE (CAN bit rate setting):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.can_br))
    print("sub_ver (Firmware sub-version):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.sub_ver))

    # 50–55
    print("u_off (U phase offset):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.u_off))
    print("v_off (V phase offset):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.v_off))
    print("k1 (Calibration scale factor 1):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.k1))
    print("k2 (Calibration scale factor 2):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.k2))
    print("m_off (Mechanical zero offset):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.m_off))
    print("dir (Rotation direction: 1=fwd, -1=rev):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.dir))

    # 80–81
    print("p_m (Position measurement):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.p_m))
    print("xout (Output encoder value):", openarm.control.read_motor_param(openarm.motors[0], DM_variable.xout))

    openarm.disable()
    print("---- Parameter Reading Complete ----")

if __name__ == "__main__":
    main()


