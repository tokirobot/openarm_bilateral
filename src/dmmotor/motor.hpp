// Copyright 2025 Reazon Holdings, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>
#include <map>
#include <vector>
#include <iostream>
#include <array>
#include <algorithm>
#include <cstring>
#include <stdio.h>

enum class DM_Motor_Type : uint8_t {
    DM3507 = 0,
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    COUNT
};

enum class DM_variable : uint8_t {
    UV_Value = 0,
    KT_Value,
    OT_Value,
    OC_Value,
    ACC,
    DEC,
    MAX_SPD,
    MST_ID,
    ESC_ID,
    TIMEOUT,
    CTRL_MODE,
    Damp,
    Inertia,
    hw_ver,
    sw_ver,
    SN,
    NPP,
    Rs,
    LS,
    Flux,
    Gr,
    PMAX,
    VMAX,
    TMAX,
    I_BW,
    KP_ASR,
    KI_ASR,
    KP_APR,
    KI_APR,
    OV_Value,
    GREF,
    Deta,
    V_BW,
    IQ_c1,
    VL_c1,
    can_br,
    sub_ver,
    u_off = 50,
    v_off,
    k1,
    k2,
    m_off,
    dir,
    p_m = 80,
    xout,
    COUNT
};

enum class Control_Type : uint8_t {
    MIT = 1,
    POS_VEL,
    VEL,
    Torque_Pos,
    COUNT
};

class Motor {
public:
    Motor() = default;
    Motor(DM_Motor_Type motorType, uint16_t slaveID, uint16_t masterID);

    void recv_data(double q, double dq, double tau, int tmos, int trotor);
    double getPosition() const;
    double getVelocity() const;
    double getTorque() const;
    int getParam(int RID) const;
    void setTempParam(int RID, int val);
    uint16_t SlaveID;
    uint16_t MasterID;
    bool isEnable;
    Control_Type NowControlMode;
    DM_Motor_Type MotorType;

    int getStateTmos() const;
    int getStateTrotor() const;
    double getGoalPosition() const;
    double getGoalVelocity() const;
    double getGoalTau() const;

    void setGoalPosition(double pos);
    void setGoalVelocity(double vel);
    void setGoalTau(double tau);
    void setStateTmos(int tmos);
    void setStateTrotor(int trotor);

    std::map<int, double> temp_param_dict;
private:
    double Pd, Vd;
    double goal_position,goal_velocity, goal_tau;
    double state_q, state_dq, state_tau;
    int state_tmos, state_trotor;
};

double LIMIT_MIN_MAX(double x, double min, double max);

uint16_t double_to_uint(double x, double x_min, double x_max, int bits);

double uint_to_double(uint16_t x, double min, double max, int bits);

std::array<uint8_t, 8> double_to_uint8s(double value);

std::array<uint8_t, 4> float_to_uint8s(float value);

float uint8s_to_float(const std::array<uint8_t, 4>& bytes);

std::array<uint8_t, 8> data_to_uint8s(uint32_t value);

uint32_t uint8s_to_uint32(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

double uint8s_to_double(uint8_t byte1, uint8_t byte2, uint8_t byte3, uint8_t byte4);

bool is_in_ranges(int number);

void print_hex(const std::vector<uint8_t>& data);

template <typename T>
T get_enum_by_index(int index);

