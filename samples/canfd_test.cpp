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

// ============================================================================
// Single Motor CANFD Control Test
//
// This program tests controlling a single motor via CAN-FD using MIT control mode.
// It continuously sends a constant torque command to the motor at a fixed control rate.
//
// Features:
// - Uses MIT mode to send a torque command of 1.4 Nm.
// - Reads and prints motor position, velocity, and torque in real-time.
//
// This is useful for verifying basic communication and control behavior
// over CAN-FD with a single motor before expanding to multi-motor systems.
// ============================================================================

#include <atomic>
#include <string>
#include <chrono>
#include <csignal>  
#include <vector>
#include <iostream>
#include <thread>
#include <fstream>
#include <sstream>
#include "../src/controller/dynamics.hpp"
#include "../src/dmmotor/damiao_port.hpp"

#define FOLLOWER_DEVICENAME0 "can0"
#define TICK 0.002
#define DOF 1 
#define PI 3.141592665

std::atomic<bool> running{true};

void signalHandler(int) {
    std::cout << "\n[Signal] SIGINT caught. Shutting down gracefully...\n";
    running = false;  
}

int main(int argc, char **argv){
    std::signal(SIGINT, signalHandler);

    DamiaoPort follower(
        FOLLOWER_DEVICENAME0, 
        {DM_Motor_Type::DM4340},
        {0x02},
        {0x12},
        {true},
        Control_Type::MIT,
        CAN_MODE_FD);

    //follower.setZeroPosition();
    //follower.moveTorqueSync2({0});

    std::vector<double> joint_positions(DOF, 0.0);
    std::vector<double> joint_velocities(DOF, 0.0);
    std::vector<double> joint_torques(DOF, 0.0);
    std::vector<double> command(DOF, 0.0);
    std::cout << "CANFD TEST START!!!" << std::endl;
    while (running) {
        auto loop_start_time = std::chrono::steady_clock::now();

        for (size_t i = 0; i < DOF; ++i) {
            joint_positions[i] = follower.motors_[i]->getPosition();
            joint_velocities[i] = follower.motors_[i]->getVelocity();
            joint_torques[i] = follower.motors_[i]->getTorque();

            //std::cout << "joint_positions[" << i << "] = " << joint_positions[i] << std::endl;
            //std::cout << "joint_velocities[" << i << "] = " << joint_velocities[i] << std::endl;
            //std::cout << "joint_torques[" << i << "] = " << joint_torques[i] << std::endl;
        }

        std::vector<double> kp_temp = {10.0};
        std::vector<double> kd_temp = {0.1};
        std::vector<double> pos_temp = {3.14};
        std::vector<double> vel_temp = {10000};
        command = {10000};

        // setMIT mode
        //follower.setMIT(pos_temp, vel_temp, kp_temp, kd_temp, command);
        
        // setMITSync mode
        //follower.setMITSync(pos_temp, vel_temp, kp_temp, kd_temp, command);

        // setPosVel mode
        // follower.setPosVel(pos_temp, vel_temp);

        // setPosVelSync mode
         //follower.setPosVelSync(pos_temp, vel_temp);

        // setPosForce mode
         //follower.setPosForce(pos_temp, vel_temp, command);

        // setPosForceSync mode
         //follower.setPosForceSync(pos_temp, vel_temp, command);

        // setVel mode
        //follower.setVel(vel_temp);

        // setVelSync mode
       //follower.setVelSync(vel_temp);

        auto loop_end_time = std::chrono::steady_clock::now();
        auto t_elapsed = std::chrono::duration<double>(loop_end_time - loop_start_time).count();
        std::cout << "elapsed time [s] : " << t_elapsed << std::endl;
        std::cout << "control Frequency [Hz]" << 1.0/t_elapsed << std::endl;
        auto sleep_time = std::chrono::duration<double>(TICK) - (loop_end_time - loop_start_time);
        if (sleep_time.count() > 0.0) {
            std::this_thread::sleep_for(sleep_time);
        } else {
            std::cout << "Loop execution time exceeded the tick duration!" << std::endl;
        }
    }

    std::cout << "finished" << std::endl;
    follower.disable();

    return 0;
}

