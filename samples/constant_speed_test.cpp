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
// Constant Velocity Friction Characterization Test
//
// This program conducts a constant velocity test to analyze motor friction 
// characteristics. It incrementally increases the reference joint velocity, 
// uses a PI controller to track the reference, and applies torque commands 
// accordingly.
//
// During each velocity segment:
// - The system waits for convergence between reference and actual velocity.
// - Once stabilized, it records the average commanded torque and actual velocity.
// - Data is saved to a CSV file for post-analysis of friction behavior.
//
// The process repeats over a predefined range of velocities in both directions 
// to capture asymmetric friction effects such as Coulomb and viscous friction.
//
// This test is essential for identifying parameters of friction models (e.g., 
// atan, tanh, sigmoid) used in motor control and simulation.
// ============================================================================ 

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <atomic>
#include <string>
#include <chrono>
#include <csignal>  
#include <vector>
#include <iostream>
#include <thread>
#include <fstream>
#include <sstream>
#include <numeric>
#include "../src/controller/dynamics.hpp"
#include "../src/dmmotor/damiao_port.hpp"

#define openarm_DEVICENAME0 "can0"
#define TICK 0.0009
#define DOF 7
#define PI 3.14159265
#define VELSTEP 0.1
#define VELSTART 0.1
#define VELEND 1.2
#define N 7

// Velocity controller gains and joint limits
const double Kp_vel[NJOINTS] = {18.0, 17.5, 16.5, 17.0, 0.75, 0.75, 0.9, 0.4};
const double Ki_vel[NJOINTS] = {4.0, 6.0, 3.0, 4.0, 3.0, 3.0, 3.0, 2.0};
const double posmin[NJOINTS] = {-PI/6, PI/2 - PI/3, -PI/3, 0   , -PI/3, -PI/3, -PI/4, -PI/6};
const double posmax[NJOINTS] = { PI/6, PI/2 + PI/3, PI/3 , PI/2,  PI/3,  PI/3,  PI/4, 0.0};
const std::vector<double> poskp = {380.0, 380.0, 380.0, 380.0, 50.0, 50.0, 50.0, 5};
const std::vector<double> poskd = {  4.0,   4.0,   4.0,   4.0,  1.4,  1.4,  1.5, 0.5};

std::atomic<bool> running{true};

void signalHandler(int) {
    running = false;  // Stop the control loop when SIGINT is received
}

int main(int argc, char **argv) {
    std::signal(SIGINT, signalHandler);  // Setup signal handling for Ctrl+C

    // Load URDF and initialize dynamics
    std::string description_path = ament_index_cpp::get_package_share_directory("openarm_bimanual_description");
    auto urdf_path = description_path + "/urdf/openarm_bimanual.urdf";
    auto dyn = Dynamics(urdf_path, "pedestal_link", "right_link8");
    dyn.Init();

    // Initialize motors
    DamiaoPort openarm(
        openarm_DEVICENAME0, 
        {DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340, DM_Motor_Type::DM4340,
         DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM4310, DM_Motor_Type::DM3507},
        {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08},
        {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18},
        Control_Type::MIT,
        CAN_MODE_FD
    );


    if(openarm.check_init_success() == true){
        std::cout << "openarm init success!" << std::endl;
    }else{
        std::cout << "openarm init failed!" << std::endl;
        std::exit(1);
    }
        
    // enable torque
    openarm.enable();

    openarm.setZeroPosition();
    openarm.moveTorqueSync2(std::vector<double>(NJOINTS, 0.0));

    // Initialize vectors
    std::vector<double> joint_positions(NJOINTS, 0.0);
    std::vector<double> joint_velocities(NJOINTS, 0.0);
    std::vector<double> joint_torques(NJOINTS, 0.0);
    std::vector<double> grav_torques(NJOINTS, 0.0);
    std::vector<double> command(NJOINTS, 0.0);
    std::vector<double> velcommand(NJOINTS, 0.0);
    std::vector<double> kp_temp(NJOINTS, 0.0), kd_temp(NJOINTS, 0.0);
    std::vector<double> pos_temp(NJOINTS, 0.0), vel_temp(NJOINTS, 0.0);
    std::vector<double> goalpos(NJOINTS, 0.0);

    // Create velocity map for reference velocity ramping
    std::vector<double> vel_map;
    for (double v = VELSTART; v <= VELEND + 1e-9; v += VELSTEP) vel_map.push_back(v);
    const int round_trip = vel_map.size();

    // Control variables
    int round_trip_counter = 0;
    double rotate_dir = 1.0;
    bool reached_min = false, reached_max = false;
    double vel_command_integral = 0.0;
    double ref_vel_filtered = 0.0;
    const double alpha = 0.008, kp_grav = 1.5;

    // Logging setup
    std::ofstream csv_file("src/openarm_bilateral/data/velocity_torque.csv");
    if (!csv_file.is_open()) {
        std::cerr << "Failed to open file for writing!" << std::endl;
        return 1;
    }
    csv_file << "ref_velocity,velcommand,actual_velocity" << std::endl;

    double prev_ref_vel_raw = -999.0;
    bool sampling = false;
    bool already_logged_pos = false;
    bool already_logged_neg = false;
    std::vector<double> velcmd_buffer, velact_buffer;

    // Posture initialization
    std::vector<double> initangle(DOF, 0.0);
    std::vector<double> offangle(DOF, 0.0);

    // Offset initialization per motion case
    switch (N) {
        case 0: offangle = {0.0, PI/2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; break;
        case 1: offangle = {PI/2.0, PI/2.0, 0.0, (2.0/3.0)*PI, 0.0, 0.0, 0.0, 0.0}; break;
        case 2:
        case 4: offangle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; break;
        case 3: offangle = {0.0, PI/2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; break;
        case 5: offangle = {0.0, 0.0, 0.0, PI/2.0, -PI/2.0, 0.0, 0.0, 0.0}; break;
        case 6: offangle = {0.0, 0.0, 0.0, PI/2.0, 0.0, 0.0, 0.0, 0.0}; break;
        case 7: offangle = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; break;
        default: break;
    }

    // Move to home position gradually
    std::cout << "go to home position!!!" << std::endl;
    const int nstep = 300;
    for (int step = 0; step < nstep; ++step) {
        double ratio = static_cast<double>(step + 1) / nstep;
        for (int i = 0; i < DOF; ++i)
            goalpos[i] = (1.0 - ratio) * initangle[i] + ratio * offangle[i];
        openarm.setGoalPositionsSync(goalpos, poskp, poskd);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "loop start !!!" << std::endl;

while (running) {
        auto loop_start_time = std::chrono::steady_clock::now();

        for (size_t i = 0; i < NJOINTS; ++i) {
            joint_positions[i] = openarm.motors_[i]->getPosition();
            joint_velocities[i] = openarm.motors_[i]->getVelocity();
            joint_torques[i] = openarm.motors_[i]->getTorque();
        }

        dyn.GetGravity(joint_positions.data(), grav_torques.data());
        for (size_t i = 0; i < DOF; ++i)
            command[i] = kp_grav * grav_torques[i];
        
        command[NJOINTS - 1] = 0.0;

        // low pass filtering 
        double ref_vel_raw = rotate_dir * vel_map[round_trip_counter];
        ref_vel_filtered = (1 - alpha) * ref_vel_filtered + alpha * ref_vel_raw;
        
        // torque based PI controller 
        velcommand[N] = Kp_vel[N] * (ref_vel_filtered - joint_velocities[N]);
        vel_command_integral += Ki_vel[N] * (ref_vel_filtered - joint_velocities[N]) * TICK;
        vel_command_integral = std::clamp(vel_command_integral, -4.0, 4.0);
        velcommand[N] += vel_command_integral;
        command[N] += velcommand[N];
        
        // select gain
        for (int i = 0; i < NJOINTS; ++i) {
            kp_temp[i] = (i == N) ? 0 : poskp[i];
            kd_temp[i] = (i == N) ? 0 : poskd[i];
        }
        
        // drive motors
        openarm.setMITSync(offangle, vel_temp, kp_temp, kd_temp, command);

        // invert rotate direction
        if (joint_positions[N] > posmax[N]) {
            rotate_dir = -1.0;
            vel_command_integral = 0.0;
            reached_max = true;
        } else if (joint_positions[N] < posmin[N]) {
            if (reached_max && !reached_min && rotate_dir == -1.0) {
                reached_min = true;
                rotate_dir = 1.0;
                vel_command_integral = 0.0;
                round_trip_counter++;
                std::cout << "round trip counter : " << round_trip_counter << std::endl;
                if (round_trip_counter < round_trip)
                    std::cout << "next ref velocity : " << vel_map[round_trip_counter] << std::endl;
                reached_max = reached_min = false;
            }
        }
        
        std::cout<< "joint_positions[8]"  <<joint_positions[NJOINTS -1] << std::endl;

        if (round_trip_counter >= round_trip) {
            std::cout << "finish constant speed test!!!" << std::endl;
            break;
        }

        if (ref_vel_raw != prev_ref_vel_raw) {
            prev_ref_vel_raw = ref_vel_raw;
            sampling = false;
            velcmd_buffer.clear();
            velact_buffer.clear();
            if (ref_vel_raw > 0) already_logged_pos = false;
            else already_logged_neg = false;
        }

        bool &already_logged = (ref_vel_raw > 0) ? already_logged_pos : already_logged_neg;
        if (!already_logged && std::abs(ref_vel_filtered - ref_vel_raw) < 1e-3) {
            sampling = true;
        }

        if (sampling) {
            velcmd_buffer.push_back(velcommand[N]);
            velact_buffer.push_back(joint_velocities[N]);

            if (velcmd_buffer.size() >= 100) {
                double velcmd_avg = std::accumulate(velcmd_buffer.begin(), velcmd_buffer.end(), 0.0) / velcmd_buffer.size();
                double velact_avg = std::accumulate(velact_buffer.begin(), velact_buffer.end(), 0.0) / velact_buffer.size();

                csv_file << ref_vel_raw << "," << velcmd_avg << "," << velact_avg << std::endl;
                sampling = false;
                already_logged = true;
            }
        }

        auto t_elapsed = std::chrono::steady_clock::now() - loop_start_time;
        auto sleep_time = std::chrono::duration<double, std::micro>(TICK * 1e6) - t_elapsed;
        if (sleep_time > std::chrono::microseconds(0))
            std::this_thread::sleep_for(sleep_time);
        else
           // std::cout << "Loop execution time exceeded the tick duration!" << std::endl;
            ;
    }

    vel_command_integral = 0;
    std::cout << "back to home position" << std::endl;
    for (int step = 0; step < nstep; ++step) {
            double ratio = static_cast<double>(step + 1) / nstep;
            for (int i = 0; i < DOF; ++i) {
                    goalpos[i] = (1.0 - ratio) * joint_positions[i] + ratio * initangle[i];
            }
            openarm.setGoalPositionsSync(goalpos, poskp, poskd);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "finished" << std::endl;

    std::cout << "you can use srcipt/torque_velocity.py !!!!!" << std::endl;
    csv_file.close();
    openarm.disable();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    return 0;
}

