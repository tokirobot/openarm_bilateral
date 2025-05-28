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

#include <sensor_msgs/msg/joint_state.hpp>
#include <utility>
#include <fstream>
#include <deque>
#include <numeric> 

#include "controller/global.hpp"
#include "dmmotor/damiao_port.hpp"
#include "controller/diff.hpp"
#include "controller/dynamics.hpp"
/*
 * Implement Bilateral control on top of the low-level
 * manipulator class (Check out arm.h).
 */

class Control
{
        DamiaoPort *arm_;
        double Ts_;
        int role_;
        Differentiator *differentiator_;

        std::string arm_type_;

        // Debug variables
        int ncycle;
        int t0;

        Dynamics *dynamics_f_;
        Dynamics *dynamics_l_;

        float oblique_coordinates_force;
        float oblique_coordinates_position;

        // DOB-estimated disturbance torque [Nm]
        double disturbance_lowpassin_[NJOINTS] = {0.0};
        double disturbance_lowpassout_[NJOINTS] = {0.0};
        double disturbance_[NJOINTS] = {0.0};

        double reactionforce_lowpassin_[NJOINTS] = {0.0};
        double reactionforce_lowpassout_[NJOINTS] = {0.0};

        // for easy logging  
        std::vector<std::pair<double, double>> velocity_log_;  // (differ_velocity, motor_velocity)
        std::string log_file_path_ = "../data/velocity_comparison.csv";
        static constexpr int VEL_WINDOW_SIZE = 10;
        static constexpr double VIB_THRESHOLD = 0.7; // [rad/s]
        std::deque<double> velocity_buffer_[NJOINTS];

        public:
        sensor_msgs::msg::JointState *response_;
        sensor_msgs::msg::JointState *reference_;

        double Dn_[NJOINTS] = {0.0};
        double Gn_[NJOINTS] = {0.0};
        double Jn_[NJOINTS] = {0.0};
        double gn_[NJOINTS] = {0.0};
        double Kp_[NJOINTS] = {0.0};
        double Kd_[NJOINTS] = {0.0};
        double Kf_[NJOINTS] = {0.0};
        double Fc_[NJOINTS] = {0.0};
        double k_[NJOINTS] = {0.0};
        double Fv_[NJOINTS] = {0.0};
        double Fo_[NJOINTS] = {0.0};

        Control(DamiaoPort *arm, double Ts, int role);
        Control(DamiaoPort *arm, double Ts, int role, std::string arm_type);
        ~Control();

        bool Setup(void);
        void Setstate(int state);
        void Shutdown(void);

        void Configure(const double *Dn, const double *Jn, const double *gn,
                        const double *Kn, const double *Kd, const double *Kf, const double *Fc, const double *k, const double *Fv, const double *Fo);

        // Return a copy of response (or reference)
        void GetResponse(sensor_msgs::msg::JointState *response);
        void SetReference(sensor_msgs::msg::JointState *reference);

        bool AdjustPosition(void);

        // Compute torque based on bilateral
        bool DoControl();
        bool DoControl_u();

        // NOTE! Control() class operates on "joints", while the underlying
        // classes operates on "actuators". The following functions map
        // joints to actuators.
        void ComputeJointPosition(const double *motor_position, double *joint_position);
        void ComputeJointVelocity(const double *motor_velocity, double *joint_velocity);
        void ComputeMotorTorque(const double *joint_torque, double *motor_torque);
        void ComputeFriction(const double *velocity, double *friction);
        void ComputeGravity(const double *position, double *gravity);
        bool DetectVibration(const double* velocity, bool *what_axis);
        void save_velocity_log_to_csv();
        void Debug(void);
};
