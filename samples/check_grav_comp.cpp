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
// Gravity Compensation Control for 7-DOF Arm using Model-Based Dynamics
//
// This program performs continuous gravity compensation using torques
// computed from a URDF-based rigid body dynamics model. Each joint torque
// is calculated based on the estimated gravitational load, and the result is
// sent to the motors using torque control.
// ============================================================================


#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <string>
#include <chrono>
#include <csignal>  
#include <vector>
#include <iostream>
#include <thread>
#include <cstdlib>
#include <chrono>
#include "../src/controller/dynamics.hpp"
#include "../src/dmmotor/damiao_port.hpp"

#define openarm_DEVICENAME0 "can3"
#define TICK 0.02
#define DOF 7
#define NJOINTS 8

std::atomic<bool> running{true};

void signalHandler(int) {
        running = false;  
}

int main() {
        signal(SIGINT, signalHandler);

        //std::string description_path = ament_index_cpp::get_package_share_directory(
                        //"openarm_v1_check_description"
                        //);
        std::string description_path = ament_index_cpp::get_package_share_directory(
                        "openarm_v1_bimanual_description"
                        );
        auto urdf_path = description_path + "/urdf/openarm_v1_bimanual.urdf";
        std::string chain_root_link = "pedestal_v1_link";
        std::string left_leaf_link = "left_oparm_link8_1";
        std::string right_leaf_link = "right_oparm_link8_1";

        auto dyn = Dynamics(urdf_path, chain_root_link, left_leaf_link);
        dyn.Init();
        std::cout << description_path << std::endl;
        DamiaoPort openarm(
                        openarm_DEVICENAME0, 
                        {DM_Motor_Type::DM8009, DM_Motor_Type::DM8009, 
                         DM_Motor_Type::DM4340, DM_Motor_Type::DM4340,
                         DM_Motor_Type::DM4310, DM_Motor_Type::DM4310,
                         DM_Motor_Type::DM4310, DM_Motor_Type::DM4310},
                        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
                        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18},
                        Control_Type::MIT,
                        CAN_MODE_FD);

        if(openarm.check_init_success() == true){
                std::cout << "openarm init success!" << std::endl;
        }else{
                std::cout << "openarm init failed!" << std::endl;
                std::exit(1);
        }

        openarm.enable();


        openarm.moveTorqueSync2({0, 0, 0, 0, 0, 0, 0, 0});
        const double kp_grav = 1.0;
        // const double kp_grav = 0.5;
        std::vector<double> joint_positions(NJOINTS, 0.0);
        std::vector<double> joint_velocities(NJOINTS, 0.0);

        std::vector<double> grav_torques(DOF, 0.0);
        std::vector<double> command(DOF, 0.0);
        while (running) {  
                //std::cout << "In the loop" << std::endl;
                auto loop_start_time = std::chrono::steady_clock::now();

                for(size_t i = 0; i < NJOINTS; ++i){
                        joint_positions[i] = openarm.motors_[i]->getPosition();
                        joint_velocities[i] = openarm.motors_[i]->getVelocity();
                        //std::cout << "joint_positions[" << i << "] = " << joint_positions[i] << std::endl;
                }
                dyn.GetGravity(joint_positions.data(), grav_torques.data());

                for(size_t i = 0; i < DOF; ++i){
                        command[i] = kp_grav * grav_torques[i];
                        // std::cout << "command[" << i << "] = " << command[i] << std::endl;
                }

                // std::cout << "joint_positions[" << 4 << "] = " << joint_positions[4]<< std::endl;
                // std::cout << "joint_velocities[" << 4 << "] = " << joint_velocities[4]<< std::endl;

                command[0] *= (1.0/0.85);
                command[1] *= (1.0/0.85);

                openarm.moveTorqueSync2(command);
                auto loop_end_time = std::chrono::steady_clock::now();
                std::chrono::duration<double> t_elapsed = loop_end_time - loop_start_time;
                //std::cout << "elapsed time [s] = " << t_elapsed.count() << std::endl;

                auto sleep_time = std::chrono::duration<double>(TICK) - t_elapsed;
                if (sleep_time.count() > 0.0) {
                        std::this_thread::sleep_for(sleep_time);
                } else {
                        //std::cout << "Loop execution time exceeded the tick duration!" << std::endl;
                }
        }
        openarm.disable();
}
