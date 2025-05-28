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
#include <string.h>
#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <urdf/model.h>
#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include "controller/global.hpp"
/*
 * Compute gravity and inertia compensation using Orocos
 * Kinematics and Dynamics Library (KDL).
 */
class Dynamics
{
        private:
                urdf::Model urdf_model;

                std::string urdf_path;
                std::string start_link;
                std::string end_link;

                KDL::JntSpaceInertiaMatrix inertia_matrix;
                KDL::JntArray q;
                KDL::JntArray q_d;
                KDL::JntArray coriolis_forces;
                KDL::JntArray gravity_forces;

                KDL::JntArray biasangle;

                KDL::Tree kdl_tree;
                KDL::Chain kdl_chain;
                std::unique_ptr<KDL::ChainDynParam> solver;

        public:
                Dynamics(std::string urdf_path, std::string start_link, std::string end_link);
                ~Dynamics();
                bool Init();
                bool Init(std::vector<double> grav_vector, std::vector<double> bias);
                void GetGravity(const double *motor_position, double *gravity);
                void GetColiori(const double *motor_position, const double *motor_velocity, double *colioli);
                void GetMassMatrixDiagonal(const double *motor_position, double *inertia_diag);

                void GetJacobian(const double *motor_position, Eigen::MatrixXd &jacobian);
 
                void GetNullSpace(const double *motor_positon, Eigen::MatrixXd &nullspace);

                void GetNullSpaceTauSpace(const double* motor_position, Eigen::MatrixXd& nullspace_T);

                void GetEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);
                

                void GetPreEECordinate(const double *motor_position, Eigen::Matrix3d &R, Eigen::Vector3d &p);
};



