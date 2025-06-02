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

#include <thread>
#include <chrono>
#include "damiao_port.hpp"


//DamiaoPort::DamiaoPort(const std::string& device,
                       //const std::vector<DM_Motor_Type>& types,
                       //const std::vector<uint16_t>& can_ids,
                       //const std::vector<uint16_t>& master_ids,
                       //const std::vector<bool>& motor_with_torque,
                       //Control_Type control_mode,
                       //int can_mode)
    //: motor_with_torque_(motor_with_torque) {
    //try {
        //canbus_ = std::make_unique<CANBus>(device, can_mode);
        //control_= std::make_unique<MotorControl>(*canbus_);
    //} catch (const std::exception& e) {
        //std::cerr << "[DamiaoPort] CANBus initialization failed: " << e.what() << std::endl;
        //return;
    //}

    //if (!(types.size() == can_ids.size() && types.size() == master_ids.size() && types.size() == motor_with_torque.size())) {
            //std::cerr << "[DamiaoPort] Error: Vector sizes do not match.\n"
                    //<< "  types: " << types.size() 
                    //<< ", can_ids: " << can_ids.size()
                    //<< ", master_ids: " << master_ids.size()
                    //<< ", motor_with_torque: " << motor_with_torque.size() << std::endl;
            //init_success_ = false;
            //return;
    //}


    //for (size_t i = 0; i < types.size(); ++i) {
        //motors_.emplace_back(std::make_unique<Motor>(types[i], can_ids[i], master_ids[i]));
    //}
   
  //bool any_mismatch = false;

    //for (size_t i = 0; i < motors_.size(); ++i) {
        //auto& motor = motors_[i];
        //control_->addMotor(*motor);

        //double mst_id_val = control_->readMotorParam(*motor, static_cast<int>(DM_variable::MST_ID));
        //double esc_id_val = control_->readMotorParam(*motor, static_cast<int>(DM_variable::ESC_ID));

        //if (mst_id_val < 0 || esc_id_val < 0) {
            //std::cerr << "Error: Failed to read MST_ID/ESC_ID for Motor (SlaveID: 0x"
                      //<< std::hex << motor->SlaveID << std::dec << ")\n";
            //control_->disable(*motor);
            //any_mismatch = true;
            //continue;
        //}

        //uint16_t read_mst_id = static_cast<uint16_t>(mst_id_val);
        //uint16_t read_esc_id = static_cast<uint16_t>(esc_id_val);

        //if (read_mst_id == motor->MasterID && read_esc_id == motor->SlaveID) {
            //std::cout << "Motor " << (i + 1) << " (MasterID: 0x" << std::hex << motor->MasterID
                      //<< ", SlaveID: 0x" << motor->SlaveID << ") found.\n" << std::dec;
            //control_->enable(*motor);
        //} else {
            //std::cerr << "Motor " << (i + 1) << " ID mismatch. "
                      //<< "Expected MST_ID: 0x" << motor->MasterID << ", got: 0x" << read_mst_id
                      //<< "; Expected ESC_ID: 0x" << motor->SlaveID << ", got: 0x" << read_esc_id << ". Disabling motor.\n";
            //control_->disable(*motor);
            //any_mismatch = true;
        //}
    //}
    //init_success_ = !any_mismatch;


    ////for (size_t i = 0; i < types.size(); ++i) {
        ////motors_.push_back(std::make_unique<Motor>(types[i], can_ids[i], master_ids[i]));
    ////}

    ////for (auto& motor : motors_) {
        ////control_->addMotor(*motor);
        ////control_->enable(*motor);
    ////}
//}


DamiaoPort::DamiaoPort(const std::string& device,
                       const std::vector<DM_Motor_Type>& types,
                       const std::vector<uint16_t>& can_ids,
                       const std::vector<uint16_t>& master_ids,
                       Control_Type control_mode,
                       int can_mode)
    : init_success_(true)
{
    try {
        canbus_ = std::make_unique<CANBus>(device, can_mode);
        control_ = std::make_unique<MotorControl>(*canbus_);
    } catch (const std::exception& e) {
        std::cerr << "[DamiaoPort] CANBus initialization failed: " << e.what() << std::endl;
        init_success_ = false;
        return;
    }

    // Vector size check
    if (!(types.size() == can_ids.size() && types.size() == master_ids.size())) {
        std::cerr << "[DamiaoPort] Error: Vector sizes do not match.\n"
                  << "  types: " << types.size()
                  << ", can_ids: " << can_ids.size()
                  << ", master_ids: " << master_ids.size() << std::endl;
        init_success_ = false;
        return;
    }

    // Motor objects creation
    for (size_t i = 0; i < types.size(); ++i) {
        motors_.emplace_back(std::make_unique<Motor>(types[i], can_ids[i], master_ids[i]));
    }

    // Motor validation
    for (size_t idx = 0; idx < motors_.size(); ++idx) {
        auto& motor = motors_[idx];
        control_->addMotor(*motor);

        double mst_id_val = control_->readMotorParam(*motor, static_cast<int>(DM_variable::MST_ID));
        double esc_id_val = control_->readMotorParam(*motor, static_cast<int>(DM_variable::ESC_ID));
        double can_br_val = control_->readMotorParam(*motor, static_cast<int>(DM_variable::can_br));

        if (std::isnan(mst_id_val) || std::isnan(esc_id_val) || std::isnan(can_br_val)) {
            std::cerr << "[DamiaoPort] Error: Failed to read motor parameters for the " << (idx + 1) << "th motor" << std::endl;
            std::cerr << "1. Check CAN communication.\n";
            std::cerr << "2. Confirm MasterID and SlaveID are correct and motor is powered.\n";
            std::cerr << "3. Supported CAN bitrates: CAN2.0 at 1Mbps, CANFD at 5Mbps.\n";
            std::cerr << "4. Verify wiring and connector status.\n";
            init_success_ = false;
            return;
        }

        uint16_t mst_id = static_cast<uint16_t>(mst_id_val);
        uint16_t esc_id = static_cast<uint16_t>(esc_id_val);
        int can_br = static_cast<int>(can_br_val);

        auto DM_Motor_Type_ToString = [](DM_Motor_Type type) -> std::string {
        switch (type) {
        case DM_Motor_Type::DM3507:     return "DM3507";
        case DM_Motor_Type::DM4310:     return "DM4310";
        case DM_Motor_Type::DM4310_48V: return "DM4310_48V";
        case DM_Motor_Type::DM4340:     return "DM4340";
        case DM_Motor_Type::DM4340_48V: return "DM4340_48V";
        case DM_Motor_Type::DM6006:     return "DM6006";
        case DM_Motor_Type::DM8006:     return "DM8006";
        case DM_Motor_Type::DM8009:     return "DM8009";
        case DM_Motor_Type::DM10010L:   return "DM10010L";
        case DM_Motor_Type::DM10010:    return "DM10010";
        case DM_Motor_Type::DMH3510:    return "DMH3510";
        case DM_Motor_Type::DMH6215:    return "DMH6215";
        case DM_Motor_Type::DMG6220:    return "DMG6220";
        case DM_Motor_Type::COUNT:      return "COUNT";
        default:                        return "UNKNOWN_TYPE";
                        }
        };

        std::string motor_type_name = DM_Motor_Type_ToString(motor->MotorType);

        if (can_br == 9) {
            std::cout << "[INFO] NOW CANFD MODE\n";
        } else if (can_br == 4) {
            std::cout << "[INFO] NOW CAN2.0 MODE\n";
        } else {
            std::cerr << "[WARNING] Supported CAN bitrates are: CAN2.0 at 1Mbps and CANFD at 5Mbps.\n";
        }

        if (mst_id == motor->MasterID && esc_id == motor->SlaveID) {
            std::cout << "[INFO] Motor type " << motor_type_name << " (MasterID: 0x"
                      << std::hex << motor->MasterID << ", SlaveID: 0x" << motor->SlaveID << ") found\n" << std::dec;
        } else {
            std::cerr << "[WARNING] " << (idx + 1) << "th motor : type " << motor_type_name
                      << " (MasterID: 0x" << std::hex << motor->MasterID << ", SlaveID: 0x" << motor->SlaveID << ") not found. "
                      << "MasterID status: " << (mst_id == motor->MasterID ? "OK" : "Mismatch")
                      << ", SlaveID status: " << (esc_id == motor->SlaveID ? "OK" : "Mismatch") << std::dec << std::endl;
            std::cerr << "Hint: Check CAN or CANFD configuration, bitrate, and ID assignments.\n";
            init_success_ = false;
            return;
        }
    }
}



bool DamiaoPort::check_init_success(){
        return init_success_;
}

std::vector<std::vector<double>> DamiaoPort::getPresentStatus() {
        std::vector<std::vector<double>> stat;
        for (auto& motor : motors_) {
                stat.push_back({
                                motor->getGoalPosition(),
                                motor->getGoalTau(),
                                motor->getPosition(),
                                motor->getVelocity(),
                                motor->getTorque(),
                                static_cast<double>(motor->getStateTmos()),
                                static_cast<double>(motor->getStateTrotor())
                                });
        }
        return stat;
}

std::vector<double> DamiaoPort::getPositions() {
        std::vector<double> stat;
        for (auto& motor : motors_) {
                stat.push_back(motor->getPosition());
        }
        return stat;
}

std::vector<double> DamiaoPort::getVelocities() {
        std::vector<double> stat;
        for (auto& motor : motors_) {
                stat.push_back(motor->getVelocity());
        }
        return stat;
}

std::vector<double> DamiaoPort::getTorques() {
        std::vector<double> stat;
        for (auto& motor : motors_) {
                stat.push_back(motor->getTorque());
        }
        return stat;
}

void DamiaoPort::enable() {
        for (auto& motor : motors_) {
                control_->enable(*motor);
        }
}


void DamiaoPort::disable() {
        for (auto& motor : motors_) {
                control_->controlMIT(*motor, 0, 0, 0, 0, 0);
                control_->disable(*motor);
        }
}

void DamiaoPort::setZeroPosition() {
        sleep(0.1);
        for (auto& motor : motors_) {
                control_->set_zero_position(*motor);
        }

        sleep(0.1);
}


void DamiaoPort::moveTowardsSync(const std::vector<double>& goal_positions, 
                const std::vector<double>& kps, 
                const std::vector<double>& kds) {
        for (size_t i = 0; i < motors_.size(); ++i) {
                double delta = goal_positions[i] - motors_[i]->getPosition();
                double v = motors_[i]->getVelocity();
                double tau = kps[i] * delta - kds[i] * v;

                motors_[i]->setGoalPosition(goal_positions[i]);
                control_->controlMIT(*motors_[i], 0, 0, 0, 0, tau);
        }
}

void DamiaoPort::setGoalTorqueSync(const std::vector<double>& goal_taus) {
        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalTau(goal_taus[i]);
                control_->controlMIT(*motors_[i], 0, 0, 0, 0, motors_[i]->getGoalTau());
                std::this_thread::sleep_for(std::chrono::microseconds(30));
        }
}

void DamiaoPort::moveTorqueSync(const std::vector<double>& taus) {

        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalTau(taus[i]);
                control_->controlMIT(*motors_[i], 0, 0, 0, 0, motors_[i]->getGoalTau());
                std::this_thread::sleep_for(std::chrono::microseconds(30));
        }
}

void DamiaoPort::moveTorqueSync2(const std::vector<double>& taus) {

        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalTau(taus[i]);
                control_->controlMIT2(*motors_[i], 0, 0, 0, 0, motors_[i]->getGoalTau());

        }
        std::this_thread::sleep_for(std::chrono::microseconds(30));
        for (size_t i = 0; i < motors_.size(); ++i) {
                control_->recv();
        }

}

void DamiaoPort::keepTorqueSync() {
        for (auto& motor : motors_) {
                control_->controlMIT(*motor, 0, 0, 0, 0, motor->getGoalTau());
                std::this_thread::sleep_for(std::chrono::microseconds(30));
        }
}

void DamiaoPort::setGoalPositionsSync(const std::vector<double>& goal_positions,
                const std::vector<double>& kps, 
                const std::vector<double>& kds) {
        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalPosition(goal_positions[i]);
                control_->controlMIT(*motors_[i], kps[i], kds[i], goal_positions[i], 0, 0);
                std::this_thread::sleep_for(std::chrono::microseconds(30));
        }
}


void DamiaoPort::setPosVel(const std::vector<double>& goal_positions,
                const std::vector<double>& goal_velocities){
        std::cout << "PosVel " <<std::endl;
        for (size_t i = 0; i < motors_.size(); ++i) {

                motors_[i]->setGoalPosition(goal_positions[i]);
                motors_[i]->setGoalVelocity(goal_velocities[i]);
                control_->controlPosVel(*motors_[i], goal_positions[i], goal_velocities[i]);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(30));

}

void DamiaoPort::setPosVelSync(const std::vector<double>& goal_positions,
                const std::vector<double>& goal_velocities){

        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalPosition(goal_positions[i]);
                motors_[i]->setGoalVelocity(goal_velocities[i]);

                control_->controlPosVel2(*motors_[i], goal_positions[i], goal_velocities[i]);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(30));

        for (size_t i = 0; i < motors_.size(); ++i) {
                control_->recv();
        }
}

void DamiaoPort::setVel(const std::vector<double>& goal_velocities){

        for (size_t i = 0; i < motors_.size(); ++i) {

                motors_[i]->setGoalVelocity(goal_velocities[i]);
                control_->controlVel(*motors_[i],goal_velocities[i]);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));

}

void DamiaoPort::setVelSync(const std::vector<double>& goal_velocities){

        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalVelocity(goal_velocities[i]);
                control_->controlVel2(*motors_[i],goal_velocities[i]);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));

        for (size_t i = 0; i < motors_.size(); ++i) {
                control_->recv();
        }
}

void DamiaoPort::setPosForce(const std::vector<double>& goal_positions,
                const std::vector<double>& goal_velocities,
                const std::vector<double>& goal_taus){

        for (size_t i = 0; i < motors_.size(); ++i) {

                motors_[i]->setGoalPosition(goal_positions[i]);
                motors_[i]->setGoalVelocity(goal_velocities[i]);
                motors_[i]->setGoalTau(goal_taus[i]);
                control_->controlPosForce(*motors_[i], goal_positions[i], goal_velocities[i], goal_taus[i]);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));

}

void DamiaoPort::setPosForceSync(const std::vector<double>& goal_positions,
                const std::vector<double>& goal_velocities,
                const std::vector<double>& goal_taus){

        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalPosition(goal_positions[i]);
                motors_[i]->setGoalVelocity(goal_velocities[i]);
                motors_[i]->setGoalTau(goal_taus[i]);
                control_->controlPosForce2(*motors_[i], goal_positions[i], goal_velocities[i], goal_taus[i]);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(30));

        for (size_t i = 0; i < motors_.size(); ++i) {
                control_->recv();
        }

}

void DamiaoPort::setMIT(
                const std::vector<double>& goal_positions,
                const std::vector<double>& goal_velocities,
                const std::vector<double>& kps, 
                const std::vector<double>& kds,
                const std::vector<double>& goal_taus) {

        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalPosition(goal_positions[i]);
                motors_[i]->setGoalVelocity(goal_velocities[i]);
                motors_[i]->setGoalTau(goal_taus[i]);

                control_->controlMIT(*motors_[i], kps[i], kds[i], goal_positions[i], goal_velocities[i], goal_taus[i]);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(10));
}

void DamiaoPort::setMITSync(
                const std::vector<double>& goal_positions,
                const std::vector<double>& goal_velocities,
                const std::vector<double>& kps, 
                const std::vector<double>& kds,
                const std::vector<double>& goal_taus) {
        for (size_t i = 0; i < motors_.size(); ++i) {
                motors_[i]->setGoalPosition(goal_positions[i]);
                motors_[i]->setGoalVelocity(goal_velocities[i]);
                motors_[i]->setGoalTau(goal_taus[i]);

                control_->controlMIT2(*motors_[i], kps[i], kds[i], goal_positions[i], goal_velocities[i], goal_taus[i]);
        }

        std::this_thread::sleep_for(std::chrono::microseconds(20));

        for (size_t i = 0; i < motors_.size(); ++i) {
                control_->recv();
        }

}
