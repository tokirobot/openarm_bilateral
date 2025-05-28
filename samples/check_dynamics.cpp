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
// This program verifies the dynamics computation using a URDF-based
// dynamics model. It loads the robot's URDF, constructs a kinematic chain,
// and calculates the gravitational torques at each joint.
//
// Procedure:
// - Loads the robot description from a specified URDF file.
// - Initializes the Dynamics object for the left arm chain.
// - Evaluates the gravity vector at the default (zero) joint configuration.
// - Modifies a specific joint angle (q[0]) and evaluates the gravity again.
//
// Output:
// The computed gravity torques are printed to the console for both joint 
// configurations to validate correct behavior of the Dynamics::GetGravity()
// function.
//
// This tool is useful for testing gravity compensation in controller design.
// ============================================================================


#include <ament_index_cpp/get_package_share_directory.hpp>
#include <string>
#include <vector>
#include <iostream>

#include "../src/controller/dynamics.hpp"

int main(){
  std::string description_path = ament_index_cpp::get_package_share_directory(
      "openarm_bimanual_description"
  );
  auto urdf_path = description_path + "/urdf/openarm_bimanual.urdf";
  std::string chain_root_link = "pedestal_link";
  std::string left_leaf_link = "left_link8";
  std::string right_leaf_link = "right_link8";
  auto dyn = Dynamics(urdf_path, chain_root_link, left_leaf_link);
  dyn.Init();

  auto q = std::vector<double>(7, 0.0);
  auto g = std::vector<double>(7, -1.0); // -1.0 for dummy

  std::cout << "With default 0 configuration:" << std::endl;
  dyn.GetGravity(q.data(), g.data());
  for(size_t i = 0; i < 7; ++i){
    std::cout << "gravity [" << i << "] = " << g[i] << std::endl;
  }

  std::cout << "If only q[0] = 1.570795" << std::endl;
  q[0] = 1.570795;
  dyn.GetGravity(q.data(), g.data());
  for(size_t i = 0; i < 7; ++i){
    std::cout << "gravity [" << i << "] = " << g[i] << std::endl;
  }
}
