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


#include <vector>
#include <memory>  
#include <cmath>
#include "motor_control.hpp"
#include "motor.hpp"
#include "canfd/canbus.hpp"

class DamiaoPort {
        public:
                DamiaoPort(const std::string& device, 
                                const std::vector<DM_Motor_Type>& types, 
                                const std::vector<uint16_t>& can_ids, 
                                const std::vector<uint16_t>& master_ids, 
                                Control_Type control_mode = Control_Type::MIT,
                                int can_mode = CAN_MODE_FD);
                
                bool check_init_success();

                std::vector<std::vector<double>> getPresentStatus();

                std::vector<double> getPositions();

                std::vector<double> getVelocities();

                std::vector<double> getTorques();

                void enable();

                void disable();

                void setZeroPosition();
                
                void moveTowardsSync(const std::vector<double>& goal_positions, 
                                const std::vector<double>& kps, 
                                const std::vector<double>& kds);
                void setGoalTorqueSync(const std::vector<double>& goal_taus);

                void moveTorqueSync(const std::vector<double>& taus);

                //Optimized function for batch sending and receiving of torque commands to all motors
                void moveTorqueSync2(const std::vector<double>& taus);

                void keepTorqueSync();

                void setGoalPositionsSync(const std::vector<double>& goal_positions, 
                                const std::vector<double>& kps, 
                                const std::vector<double>& kds);

                void setPosVel(const std::vector<double>& goal_positions,
                                const std::vector<double>& goal_velocities);

                void setPosVelSync(const std::vector<double>& goal_positions,
                                const std::vector<double>& goal_velocities);

                void setVel(const std::vector<double>& goal_velocities);

                void setVelSync(const std::vector<double>& goal_velocities);

                void setPosForce(const std::vector<double>& goal_positions,
                                const std::vector<double>& goal_velocities,
                                const std::vector<double>& goal_tau);

                void setPosForceSync(const std::vector<double>& goal_positions,
                                const std::vector<double>& goal_velocities,
                                const std::vector<double>& goal_tau);

                void setMIT(
                                const std::vector<double>& goal_positions,
                                const std::vector<double>& goal_velocities,
                                const std::vector<double>& kps, 
                                const std::vector<double>& kds,
                                const std::vector<double>& goal_tau);

                //Optimized function for batch sending and receiving of MIT commands to all motors
                void setMITSync(
                                const std::vector<double>& goal_positions,
                                const std::vector<double>& goal_velocities,
                                const std::vector<double>& kps, 
                                const std::vector<double>& kds,
                                const std::vector<double>& goal_tau);

                const std::vector<std::unique_ptr<Motor>>& getMotors() const { return motors_; }

                std::unique_ptr<CANBus> canbus_; 
                //MotorControl control_;
                std::unique_ptr<MotorControl> control_;
                std::vector<std::unique_ptr<Motor>> motors_; 
                std::vector<bool> motor_with_torque_; 
        private:
               bool init_success_ = true;

};


