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

#include <unistd.h>
#include <time.h>
#include <iostream>
/*
 * Basic config for Aloha Stationary Kits
 */
constexpr double PI = 3.14159265358979323846;

// 8piecies including gripper
// Joints and motors don't always have a one-to-one correspondence
#define NJOINTS 8
#define NMOTORS 8

#define ROLE_LEADER 1
#define ROLE_FOLLOWER 2

#define CAN0 "can0"
#define CAN1 "can1"

#define CAN2 "can2"
#define CAN3 "can3"

#define GRIP_SCALE 1.0
#define LOGGING true
#define AUTOBALANCER_ON false
#define TANHFRIC true

//#define CAN_MODE_CLASSIC 0
//#define CAN_MODE_FD      1

/*
 * Macros for Bilateral control
 */
#define FREQUENCY 1500.0
#define ALPHA 1.0
#define BETA  0.5
#define CUTOFF_FREQUENCY 90.0

#define ELBOWLIMIT PI/5.0
#define GN_SCALE 1.0

static const double SATURATION_DOB[NJOINTS] = {
11.5, 11.5, 10.5, 11.5, 7.0, 7.0, 6.0, 2.5 
};

static const double INITIAL_POSITION[NMOTORS] = {
  0, 0, 0, ELBOWLIMIT, 0, 0, 0, 0
};

// safety limit position
static const double position_limit_max_L[] = { (2.0/3.0)*PI, PI, PI/2.0, PI, PI/2.0, PI/2.0, PI/3.0, PI };
static const double position_limit_min_L[] = { -(2.0/3.0)*PI, -PI/17.0, -PI/2.0, ELBOWLIMIT, -PI/2.0, -PI/2.0, -PI/3.0, -PI };
static const double position_limit_max_F[] = { (2.0/3.0)*PI, PI, PI/2.0, PI, PI/2.0, PI/2.0, PI/3.0, PI };
static const double position_limit_min_F[] = { -(2.0/3.0)*PI, -PI/17.0, -PI/2.0, ELBOWLIMIT, -PI/2.0, -PI/2.0, -PI/3.0, -PI };
// sefaty limit velocity              
static const double velocity_limit_L[] = {8.0,8.0,8.0,8.0,8.0,8.0,8.0,8.0}; 
static const double velocity_limit_F[] = {8.0,8.0,8.0,8.0,8.0,8.0,8.0,8.0}; 
// sefaty limit effort
static const double effort_limit_L[] = {20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0}; 
static const double effort_limit_F[] = {20.0,20.0,20.0,20.0,20.0,20.0,20.0,20.0};   

// opening function
inline void printOpenArmBanner() {
    std::cout << R"(

                                     ██████╗ ██████╗ ███████╗███╗   ██╗ █████╗ ██████╗ ███╗   ███╗                                               
                                    ██╔═══██╗██╔══██╗██╔════╝████╗  ██║██╔══██╗██╔══██╗████╗ ████║                                               
                                    ██║   ██║██████╔╝█████╗  ██╔██╗ ██║███████║██████╔╝██╔████╔██║                                               
                                    ██║   ██║██╔═══╝ ██╔══╝  ██║╚██╗██║██╔══██║██╔══██╗██║╚██╔╝██║                                               
                                    ╚██████╔╝██║     ███████╗██║ ╚████║██║  ██║██║  ██║██║ ╚═╝ ██║                                               
                                     ╚═════╝ ╚═╝     ╚══════╝╚═╝  ╚═══╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝                                               
                                                                                                                                                 
██████╗ ██╗██╗      █████╗ ████████╗███████╗██████╗  █████╗ ██╗          ██████╗ ██████╗ ███╗   ██╗████████╗██████╗  ██████╗ ██╗     ██╗██╗██╗██╗
██╔══██╗██║██║     ██╔══██╗╚══██╔══╝██╔════╝██╔══██╗██╔══██╗██║         ██╔════╝██╔═══██╗████╗  ██║╚══██╔══╝██╔══██╗██╔═══██╗██║     ██║██║██║██║
██████╔╝██║██║     ███████║   ██║   █████╗  ██████╔╝███████║██║         ██║     ██║   ██║██╔██╗ ██║   ██║   ██████╔╝██║   ██║██║     ██║██║██║██║
██╔══██╗██║██║     ██╔══██║   ██║   ██╔══╝  ██╔══██╗██╔══██║██║         ██║     ██║   ██║██║╚██╗██║   ██║   ██╔══██╗██║   ██║██║     ╚═╝╚═╝╚═╝╚═╝
██████╔╝██║███████╗██║  ██║   ██║   ███████╗██║  ██║██║  ██║███████╗    ╚██████╗╚██████╔╝██║ ╚████║   ██║   ██║  ██║╚██████╔╝███████╗██╗██╗██╗██╗
╚═════╝ ╚═╝╚══════╝╚═╝  ╚═╝   ╚═╝   ╚══════╝╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝     ╚═════╝ ╚═════╝ ╚═╝  ╚═══╝   ╚═╝   ╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝╚═╝╚═╝╚═╝

    )" << std::endl;
}
