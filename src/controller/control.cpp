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

#include <cmath>
#include <unistd.h>
#include <string.h>
#include <thread>
#include <cmath>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <algorithm>
#include <iomanip>
#include "control.hpp"

Control::Control(DamiaoPort *arm, double Ts, int role):
        arm_(arm), Ts_(Ts), role_(role)
{
        differentiator_ = new Differentiator(Ts);
        response_ = new sensor_msgs::msg::JointState();
        reference_ = new sensor_msgs::msg::JointState();

}

Control::Control(DamiaoPort *arm, double Ts, int role, std::string arm_type):
        arm_(arm), Ts_(Ts), role_(role)
{
        differentiator_ = new Differentiator(Ts);
        response_ = new sensor_msgs::msg::JointState();
        reference_ = new sensor_msgs::msg::JointState();
        arm_type_ = arm_type;
}

Control::~Control() {
        std::cout << "Control destructed " << std::endl;
}

bool Control::Setup(void)
{
        double motor_position[NMOTORS] = {0.0};

        response_->position.resize(NJOINTS, 0.0);
        response_->velocity.resize(NJOINTS, 0.0);
        response_->effort.resize(NJOINTS, 0.0);

        reference_->position.resize(NJOINTS, 0.0);
        reference_->velocity.resize(NJOINTS, 0.0);
        reference_->effort.resize(NJOINTS, 0.0);


        printf("Control created for arm_type: %s\n", arm_type_.c_str());
        //std::string description_path = ament_index_cpp::get_package_share_directory(
                        //"openarm_bimanual_description"
                        //);
        //auto urdf_path = description_path + "/urdf/openarm_bimanual.urdf";
        //std::string chain_root_link = "pedestal_link";
        //std::string left_leaf_link = "left_link8";
        //std::string right_leaf_link = "right_link8";

        std::string description_path = ament_index_cpp::get_package_share_directory(
                        "openarm_v1_bimanual_description"
                        );

        auto urdf_path = description_path + "/urdf/openarm_v1_bimanual.urdf";
        std::string chain_root_link = "pedestal_v1_link";
        std::string left_leaf_link = "left_oparm_link8_1";
        std::string right_leaf_link = "right_oparm_link8_1";

        if(arm_type_ == "left_arm"){
                dynamics_l_ = new Dynamics(urdf_path, chain_root_link, left_leaf_link);
                dynamics_f_ = new Dynamics(urdf_path, chain_root_link, left_leaf_link);
        }
        else if(arm_type_ == "right_arm"){
                dynamics_l_ = new Dynamics(urdf_path, chain_root_link, right_leaf_link);
                dynamics_f_ = new Dynamics(urdf_path, chain_root_link, right_leaf_link);
        }

        if (!dynamics_l_->Init()) {
                return false;
        }
        if (!dynamics_f_->Init()) {
                return false;
        }

        ComputeJointPosition(motor_position, response_->position.data());

        std::cout << "!control->Setup()  finished "<< std::endl;

        return true;
}

void Control::Shutdown(void){
        std::cout << "control shutdown !!!" << std::endl;

        //if (!velocity_log_.empty()) {
                //save_velocity_log_to_csv();
        //}

        arm_->disable();
}

void Control::Configure(const double *Dn, const double *Jn, const double *gn,
                const double *Kp, const double *Kd, const double *Kf, const double *Fc, const double *k, const double *Fv, const double *Fo)
{
        memcpy(Dn_, Dn, sizeof(double) * NJOINTS);
        memcpy(Jn_, Jn, sizeof(double) * NJOINTS);
        memcpy(gn_, gn, sizeof(double) * NJOINTS);
        memcpy(Kp_, Kp, sizeof(double) * NJOINTS);
        memcpy(Kd_, Kd, sizeof(double) * NJOINTS);
        memcpy(Kf_, Kf, sizeof(double) * NJOINTS);
        memcpy(Fc_, Fc, sizeof(double) * NJOINTS);
        memcpy(k_,   k, sizeof(double) * NJOINTS);
        memcpy(Fv_, Fv, sizeof(double) * NJOINTS);
        memcpy(Fo_, Fo, sizeof(double) * NJOINTS);
}

// Multithreading and KDL-enabled bilateral control
bool Control::DoControl()
{
        double motor_position[NMOTORS] = {0.0};
        double motor_velocity[NMOTORS] = {0.0};
        double motor_torque[NMOTORS] = {0.0};
        double joint_torque[NJOINTS] = {0.0};
        double input_torque[NJOINTS] = {0.0};
        double friction[NJOINTS] = {0.0};
        double gravity[NJOINTS] = {0.0};
        double colioli[NJOINTS] = {0.0};
        double inertia_diag[NJOINTS] = {0.0};

        //leader and followet inertia
        double inertia_diag_l[NJOINTS] = {0.0};
        double inertia_diag_f[NJOINTS] = {0.0};

        auto start_time = std::chrono::steady_clock::now();

        std::vector<double> kp_temp = {290.523, 290.0, 250.0, 250.0, 24.0, 29.0, 29.0, 1.0};
        std::vector<double> kd_temp = {3.9, 3.0, 4.0, 4.0, 0.2, 0.17, 0.2, 0.05};  

        if(role_ == ROLE_FOLLOWER){
                kp_temp[NJOINTS - 1] *= (1.0f/GRIP_SCALE);
                kd_temp[NJOINTS - 1] *= (1.0f/GRIP_SCALE);
        }

        std::vector<double> positions = arm_->getPositions();
        std::vector<double> velocities = arm_->getVelocities();
        
        if(role_ == ROLE_FOLLOWER){
        // std::cout << "follower positions[4]: " << positions[4] << std::endl;
        }
        else if(role_ == ROLE_LEADER){
                // std::cout << "leader positions[4]" << positions[4] << std::endl;
                // positions[4] *= 1.0;
        }

        for (size_t i = 0; i < positions.size(); ++i) {
                motor_position[i] = positions[i];
        }
        
        ComputeJointPosition(motor_position, response_->position.data());

        
        //low-pass filtering only gripper 
        double temp_a = 0.05;
        if(role_ == ROLE_FOLLOWER){
                temp_a = 0.02;
        }
        else {
                temp_a = 0.02;
        }

        static double filtered_velocity_7 = 0.0;
        filtered_velocity_7 = temp_a * velocities[7] + (1.0 - temp_a) * filtered_velocity_7;         
        // std::cout << "filtered_velocity_7 : " << filtered_velocity_7 << "  velocity raw : "<< velocities[7] << std::endl;
        velocities[7] = filtered_velocity_7;


        if(role_ == ROLE_LEADER){
                reference_->position[NJOINTS - 1] *= 1.0f/(GRIP_SCALE);
                reference_->velocity[NJOINTS - 1] *= 1.0f/(GRIP_SCALE);
        }
        else if(role_ == ROLE_FOLLOWER){
                reference_->position[NJOINTS - 1] *= (GRIP_SCALE);
                reference_->velocity[NJOINTS - 1] *= (GRIP_SCALE);
        }

        //        if(role_ == ROLE_LEADER){
        //                std::cout << "positions: [";
        //                for (int i = 0; i < NJOINTS - 1; ++i) {
        //                        std::cout << std::fixed << std::setprecision(4) << response_->position[i];
        //                        if (i != NJOINTS - 2) std::cout << ", ";
        //                }
        //                std::cout << "]" << std::endl;
        //        }


        // Compute inertia based on oblique coordinate system
        //float kg = 1.0;
        std::vector<double> kg = {0.625, 0.625, 1.0, 1.0, 1.0, 1.0, 1.0 ,1.0};  
        if (role_ == ROLE_LEADER) {
                dynamics_l_->GetGravity(response_->position.data(), gravity);
                // dynamics_l_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
                dynamics_l_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag);

                dynamics_l_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag_l);
                dynamics_f_->GetMassMatrixDiagonal(reference_->position.data(), inertia_diag_f);
                inertia_diag_l[NJOINTS - 1] = 0.001;
                inertia_diag_f[NJOINTS - 1] = 0.001;
        } else {
                dynamics_f_->GetGravity(response_->position.data(), gravity);
                // dynamics_f_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
                dynamics_f_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag);

                dynamics_l_->GetMassMatrixDiagonal(reference_->position.data(), inertia_diag_l);
                dynamics_f_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag_f);
                inertia_diag_l[NJOINTS - 1] = 0.001;
                inertia_diag_f[NJOINTS - 1] = 0.001;
        }

        for(int i = 0; i < NJOINTS; ++i){
                inertia_diag_l[i] *= kg[i];
                inertia_diag_f[i] *= kg[i];
        }

        // differentiator_->Differentiate(motor_position, motor_velocity);
        differentiator_->Differentiate_w_obs(motor_position, motor_velocity, inertia_diag_l, input_torque);
        if( role_ == ROLE_FOLLOWER){

                //std::cout << "vel[i] from differ: " << motor_velocity[3] << std::endl;
                // std::cout << "vel[i] from motor : " << velocities[3] << std::endl;
              //  velocity_log_.emplace_back(motor_velocity[3], velocities[3]);

        }

        // temp actually observer based vel estimate often become unstable
        for (size_t i = 0; i < positions.size(); ++i) {
                motor_velocity[i] = velocities[i];
        }

        // Get joint states based on actuator states
        ComputeJointVelocity(motor_velocity, response_->velocity.data());

        // Friction (compute joint friction)
        ComputeFriction(response_->velocity.data(), friction);

        if (role_ == ROLE_LEADER) {
                dynamics_l_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
        } else {
                dynamics_f_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
        }

        // ncycle++;
        for (int i = 0; i < NJOINTS; i++) {
                if (role_ == ROLE_LEADER){
                        if(i >= 0 && i < 7){
                                Jn_[i] = inertia_diag_l[i];
                        }
                }

                if (role_ == ROLE_FOLLOWER){
                        if(i >= 0 && i < 7){
                                Jn_[i] = inertia_diag_f[i];
                        }
                }


                if (role_ == ROLE_LEADER) {
                        oblique_coordinates_force = (ALPHA * inertia_diag_l[i]) / (ALPHA * inertia_diag_l[i] + BETA * inertia_diag_f[i]);
                        oblique_coordinates_position = (BETA * inertia_diag_l[i] * inertia_diag_f[i]) / (ALPHA * inertia_diag_l[i] + BETA * inertia_diag_f[i]);
                } else {
                        oblique_coordinates_force = (inertia_diag_f[i]) / (ALPHA  * inertia_diag_l[i] + BETA * inertia_diag_f[i]);
                        oblique_coordinates_position = (inertia_diag_l[i] * inertia_diag_f[i]) / (ALPHA * inertia_diag_l[i] + BETA * inertia_diag_f[i]);
                }

                if(role_ == ROLE_LEADER){
                if (i ==4){

                        std::cout << "=============================================" << std::endl;
                        std::cout << " L  kp oblique : " << oblique_coordinates_position*Kp_[i] << " kd oblique : " <<  oblique_coordinates_position*Kd_[i] << std::endl;
                        std::cout << " L   kf oblique : " << oblique_coordinates_force*Kf_[i] << std::endl ;

                        }
                }
                if(role_ == ROLE_FOLLOWER){
                        if (i == 4){
        
                        std::cout << "=============================================" << std::endl;
                        std::cout << " F  kp oblique : " << oblique_coordinates_position*Kp_[i] << " kd oblique : " <<  oblique_coordinates_position*Kd_[i] << std::endl;
                        std::cout << " F   kf oblique : " << oblique_coordinates_force*Kf_[i] << std::endl ;
        
                        }
                }

                // if(role_ == ROLE_FOLLOWER){
                //         if(i == 7){
                //                 std::cout << " reference_->position[i] : " << reference_->position[i] << std::endl;
                //                 }
                // }

                if (i >= 0 && i < 5) {
                        kp_temp[i] = oblique_coordinates_position * Kp_[i];
                        kd_temp[i] = oblique_coordinates_position * Kd_[i];
                    }


                // double tau_p_oblique = oblique_coordinates_position * Kp_[i] * (reference_->position[i] - response_->position[i]);
                // double tau_v_oblique = oblique_coordinates_position * Kd_[i] * (reference_->velocity[i] - response_->velocity[i]);
                // double tau_f_oblique =  - oblique_coordinates_force * Kf_[i] * (reference_->effort[i] + response_->effort[i]);

                double tau_p_oblique = kp_temp[i] * (reference_->position[i] - response_->position[i]);
                double tau_v_oblique = kd_temp[i] * (reference_->velocity[i] - response_->velocity[i]);
                double tau_f_oblique = - oblique_coordinates_force * Kf_[i] * (reference_->effort[i] + response_->effort[i]);

                joint_torque[i] = tau_p_oblique + tau_v_oblique + gravity[i] * kg[i] + tau_f_oblique + disturbance_[i];
                input_torque[i] = joint_torque[i];
                
                if (i == 7){
                        joint_torque[i] = tau_p_oblique + tau_v_oblique + tau_f_oblique + disturbance_[i]*0.0 ;
                }

                // DOB 1 Mass jointspace
                double a = gn_[i] * Ts_;
                disturbance_lowpassin_[i] = (joint_torque[i] - gravity[i]*kg[i]) + gn_[i] * Jn_[i] * response_->velocity[i];
                disturbance_lowpassout_[i] += a * (disturbance_lowpassin_[i] - disturbance_lowpassout_[i]);
                disturbance_[i] = disturbance_lowpassout_[i] - response_->velocity[i] * Jn_[i] * gn_[i];


                // DOB saturation to prevent vibrating
                for (int i = 0; i < NJOINTS; ++i) {
                        disturbance_[i] = std::clamp(disturbance_[i], -SATURATION_DOB[i], SATURATION_DOB[i]);
                }     


                // RFOB 1 Mass jointspace
                double a_ = GN_SCALE * gn_[i] * Ts_;
                reactionforce_lowpassin_[i] = (joint_torque[i] - gravity[i]*kg[i]) + GN_SCALE*gn_[i] * Jn_[i] * response_->velocity[i] - friction[i] -  colioli[i];
                reactionforce_lowpassout_[i] += a_ * (reactionforce_lowpassin_[i] - reactionforce_lowpassout_[i]);
                response_->effort[i] = reactionforce_lowpassout_[i] - response_->velocity[i] * Jn_[i] *GN_SCALE * gn_[i];

                // For DOB and RFOB caluculation
                if(i < NJOINTS - 1){
                        joint_torque[i] += (-tau_p_oblique - tau_v_oblique);
                }
        }


        bool vibration_flags[NJOINTS];
        if (DetectVibration(response_->velocity.data(), vibration_flags)) {
                std::cout << "[INFO] Vibrations detected on joints: ";
                for (int i = 0; i < NJOINTS; ++i) {
                        if (vibration_flags[i]) {
                                std::cout << i << " ";
                                // joint_torque[i] += -k_damp[i] * response_->velocity[i];
                        }
                }
                std::cout << std::endl;
        }

        // Auto Endeffector Balancer 
        if(AUTOBALANCER_ON){
                if(role_ == ROLE_LEADER){

                        double balance_torque[NJOINTS];
                        double balance_angle[NJOINTS];
                        Eigen::MatrixXd Null;
                        Eigen::MatrixXd Null_T;
                        Eigen::Matrix3d R_ee, R_pre;
                        Eigen::Vector3d p_ee, p_pre;

                        dynamics_l_->GetEECordinate(response_->position.data(), R_ee, p_ee);
                        dynamics_l_->GetPreEECordinate(response_->position.data(), R_pre, p_pre);
                        dynamics_l_->GetNullSpaceTauSpace(response_->position.data(), Null_T);
                        const double temp[NJOINTS - 1] = {0, 0.25, -0.78, 0.93, 0.30, 0.81, 0.53};

                        Eigen::VectorXd delta_q0(NJOINTS - 1);
                        for (int i = 0; i < NJOINTS - 1; ++i) {
                                delta_q0(i) = temp[i] - response_->position[i];
                        }

                        double k_balance = 0.0;
                        Eigen::VectorXd tau_null = k_balance * (Null_T * delta_q0);

                        for (int i = 0; i < NJOINTS-1; ++i) {
                                balance_torque[i] = tau_null(i);
                        }

                        //std::cout << "[Null-space Torque Result]" << std::endl;
                        //for (int i = 0; i < NJOINTS-1; ++i) {
                        //        std::cout << "Joint " << i << ": " << balance_torque[i] << std::endl;
                        //}                        
                        for (int i = 0; i < NJOINTS-1; ++i){
                                joint_torque[i] += balance_torque[i];

                        }

                }                        

        }

        ComputeMotorTorque(joint_torque, motor_torque);

        std::vector<double> damiao_torque;

        for (size_t i = 0; i <NJOINTS; ++i) {
                //if(i == NJOINTS - 1)
                        //motor_torque[i] *= -1.0;
                damiao_torque.push_back(motor_torque[i]);
        }
        //This is because EE is current control 
        //kp_temp[NJOINTS-1] = 0.0;
        //kd_temp[NJOINTS-1] = 0.0;
        
        arm_->setMITSync(reference_->position, reference_->velocity, kp_temp, kd_temp, damiao_torque);


        auto end_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;

         //std::cout << "[bilateral control] elapsed time [s] = " << elapsed.count();
         //std::cout << " (" << 1.0 / elapsed.count() << "Hz)" << std::endl;

        return true;
        }


        bool Control::DoControl_u(){

                double motor_position[NMOTORS] = {0.0};
                double motor_velocity[NMOTORS] = {0.0};
                double motor_torque[NMOTORS] = {0.0};
                double joint_torque[NJOINTS] = {0.0};
                double friction[NJOINTS] = {0.0};
                double gravity[NJOINTS] = {0.0};
                double colioli[NJOINTS] = {0.0};
                double inertia_diag[NJOINTS] = {0.0};

                std::vector<double> positions = arm_->getPositions();
                std::vector<double> velocities = arm_->getVelocities();

                for (size_t i = 0; i < positions.size(); ++i) {
                        motor_position[i] = positions[i];
                        motor_velocity[i] = velocities[i];
                }

                ComputeJointPosition(motor_position, response_->position.data());
                ComputeJointVelocity(motor_velocity, response_->velocity.data());
                std::vector<double> kg = {0.625, 0.625, 1.0, 1.0, 1.0, 1.0, 1.0 ,1.0};  

                if(role_ == ROLE_LEADER){

                        ComputeFriction(response_->velocity.data(), friction);

                        dynamics_l_->GetGravity(response_->position.data(), gravity);
                        dynamics_l_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
                        dynamics_l_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag);

                        for (int i = 0; i < NJOINTS; i++) {
                                if(i == NJOINTS - 1){
                                        friction[i] = 0.0;
                                        colioli[i] = 0.0;
                                        gravity[i] = 0.0;
                                }
                                joint_torque[i] = gravity[i]*kg[i] + friction[i]*0.5 + colioli[i]*0.1;
                        }

                        ComputeMotorTorque(joint_torque, motor_torque);
                        // return arm_->SetTorque(motor_torque);
                        std::vector<double> damiao_torque;

                        for (size_t i = 0; i < positions.size(); ++i) {
                                damiao_torque.push_back(motor_torque[i]);
                        }

                        std::vector<double> kp_temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                        std::vector<double> kd_temp = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0};

                        arm_->setMITSync(reference_->position, reference_->velocity, kp_temp, kd_temp, damiao_torque);

                        return true;

                }

                else if(role_ == ROLE_FOLLOWER){

                        dynamics_f_->GetGravity(response_->position.data(), gravity);
                        dynamics_f_->GetColiori(response_->position.data(), response_->velocity.data(), colioli);
                        dynamics_f_->GetMassMatrixDiagonal(response_->position.data(), inertia_diag);

                        std::vector<double> damiao_torque;

                        for (size_t i = 0; i < positions.size(); ++i) {
                                damiao_torque.push_back(0);

                        }

                        std::vector<double> kp_temp = {300, 250.0, 250.0, 250.0, 30.0, 30.0, 30.0, 2.0};
                        std::vector<double> kd_temp = {4.0, 4.0, 4.0, 4.0, 0.7, 0.7, 0.7, 0.20};

                        reference_->position[NJOINTS - 1] *= GRIP_SCALE;
                        reference_->velocity[NJOINTS - 1] *= GRIP_SCALE;
                        
                        arm_->setMITSync(reference_->position, reference_->velocity, kp_temp, kd_temp, damiao_torque);

                        return true;
                }

                return true;

        }


        // Mapping from joint angles to motor angles
        // (The number of joints and motors may not necessarily match)
        void Control::ComputeJointPosition(const double *motor_position, double *joint_position)
        {

                joint_position[0] = motor_position[0];
                joint_position[1] = motor_position[1];
                joint_position[2] = motor_position[2];
                joint_position[3] = motor_position[3];
                joint_position[4] = motor_position[4];
                joint_position[5] = motor_position[5];
                joint_position[6] = motor_position[6];
                joint_position[7] = motor_position[7];

        }

        void Control::ComputeJointVelocity(const double *motor_velocity, double *joint_velocity)
        {

                joint_velocity[0] = motor_velocity[0];
                joint_velocity[1] = motor_velocity[1];
                joint_velocity[2] = motor_velocity[2];
                joint_velocity[3] = motor_velocity[3];
                joint_velocity[4] = motor_velocity[4];
                joint_velocity[5] = motor_velocity[5];
                joint_velocity[6] = motor_velocity[6];
                joint_velocity[7] = motor_velocity[7];

        }

        void Control::ComputeMotorTorque(const double *joint_torque, double *motor_torque)
        {
                motor_torque[0] = joint_torque[0];
                motor_torque[1] = joint_torque[1];
                motor_torque[2] = joint_torque[2];
                motor_torque[3] = joint_torque[3];
                motor_torque[4] = joint_torque[4];
                motor_torque[5] = joint_torque[5];
                motor_torque[6] = joint_torque[6];
                motor_torque[7] = joint_torque[7];

        }

void Control::ComputeFriction(const double *velocity, double *friction)
{
        if (TANHFRIC) {
                // tanh friction model
                double amp_tmp = 1.00;
                // To make the region near the static friction smoother
                double coef_tmp = 0.1;
                for (int i = 0; i < NJOINTS; i++) {
                        const double v = velocity[i];
                        const double Fc = Fc_[i];
                        const double k = k_[i];
                        const double Fv = Fv_[i];
                        const double Fo = Fo_[i];
                        friction[i] = amp_tmp * Fc * std::tanh(coef_tmp * k * v) + Fv * v + Fo;
                }
        } else {
                // linear friction model
                for (int i = 0; i < NJOINTS; i++) {
                        friction[i] = velocity[i] * Dn_[i];
                }
        }
}

        void Control::GetResponse(sensor_msgs::msg::JointState *response)
        {
                response->position.resize(NJOINTS, 0.0);
                response->velocity.resize(NJOINTS, 0.0);
                response->effort.resize(NJOINTS, 0.0);

                for (int i = 0; i < NJOINTS; i++) {
                        response->position[i] = response_->position[i];
                        response->velocity[i] = response_->velocity[i];
                        response->effort[i] = response_->effort[i];
                }
        }

        void Control::SetReference(sensor_msgs::msg::JointState *reference)
        {
                reference->position.resize(NJOINTS, 0.0);
                reference->velocity.resize(NJOINTS, 0.0);
                reference->effort.resize(NJOINTS, 0.0);

                for (int i = 0; i < NJOINTS; i++) {
                        reference_->position[i] = reference->position[i];
                        reference_->velocity[i] = reference->velocity[i];
                        reference_->effort[i] = reference->effort[i];
                }
                reference_->position[NJOINTS - 1] = -reference->position[NJOINTS - 1];
                reference_->velocity[NJOINTS - 1] = -reference->velocity[NJOINTS - 1];
                reference_->effort[NJOINTS - 1] = -reference->effort[NJOINTS - 1];

        }

        bool Control::AdjustPosition(void)
        {
                //double motor_position[NMOTORS] = {0.0};
                int nstep = 220;
                double a;

                std::vector<double> goalpos(NMOTORS, 0.0);

                std::vector<double> positions_now = arm_->getPositions();
                std::vector<double> velocities_now = arm_->getVelocities();

                ComputeJointPosition(positions_now.data(), response_->position.data());
                ComputeJointVelocity(velocities_now.data(), response_->velocity.data());

                std::vector<double> kp_temp = {120, 260.0, 110.0, 260.0, 10.0, 60.0, 10.0, 0.0};
                std::vector<double> kd_temp = {2.3, 1.5, 1.5, 1.5, 0.3, 0.2, 0.3, 0.0};

                for (int step = 0; step < nstep; step++) {
                        a = static_cast<double>(step + 1) / nstep;

                        for (int i = 0; i < NMOTORS; i++) {
                                goalpos[i] = INITIAL_POSITION[i] * a + positions_now[i] * (1.0 - a);
                        }

                        arm_->setGoalPositionsSync(goalpos, kp_temp, kd_temp);

                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                }

                ComputeJointPosition(goalpos.data(), response_->position.data());

                return true;
        }

        bool Control::DetectVibration(const double* velocity, bool* what_axis)
        {
                bool vibration_detected = false;

                for (int i = 0; i < NJOINTS; ++i) {
                        what_axis[i] = false;

                        velocity_buffer_[i].push_back(velocity[i]);
                        if (velocity_buffer_[i].size() > VEL_WINDOW_SIZE)
                                velocity_buffer_[i].pop_front();

                        if (velocity_buffer_[i].size() < VEL_WINDOW_SIZE)
                                continue;

                        double mean = std::accumulate(
                                        velocity_buffer_[i].begin(), velocity_buffer_[i].end(), 0.0
                                        ) / velocity_buffer_[i].size();

                        double var = 0.0;
                        for (double v : velocity_buffer_[i]) {
                                var += (v - mean) * (v - mean);
                        }

                        double stddev = std::sqrt(var / velocity_buffer_[i].size());

                        if (stddev > VIB_THRESHOLD) {
                                what_axis[i] = true;
                                vibration_detected = true;
                                std::cout << "[VIBRATION] Joint " << i << " stddev: " << stddev << std::endl;
                        }
                }

                return vibration_detected;
        }

        // TODO: update this CSV export to integrate with training-data collection
        void Control::save_velocity_log_to_csv()
        {
                std::ofstream file(log_file_path_);
                if (!file.is_open()) {
                        std::cerr << "Error: Cannot open file " << log_file_path_ << std::endl;
                        return;
                }

                file << "differ_velocity,motor_velocity\n";

                for (const auto& entry : velocity_log_) {
                        file << entry.first << "," << entry.second << "\n";
                }

                file.close();
        }

        void Control::Debug(void)
        {
                for (int i = 0; i < NJOINTS; i++) {
                        printf("%i %+3.1lf %+3.1lf %+3.1lf|", i, response_->position[i], response_->velocity[i], response_->effort[i]);
                }
                for (int i = 0; i < NJOINTS; i++) {
                        printf("%i %+3.1lf %+3.1lf %+3.1lf|", i, reference_->position[i], reference_->velocity[i], reference_->effort[i]);
                }
                printf("\n");
        }
