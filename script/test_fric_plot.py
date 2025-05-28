# Copyright 2025 Reazon Holdings, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import matplotlib.pyplot as plt

# Parameter definitions
Dn_L = [0.7, 0.7, 0.7, 0.7, 0.12, 0.11, 0.15, 0.030]
Fc_  = [0.572, 0.426, 0.450, 0.146, 0.070, 0.093, 0.212, 0.0]eigo
k_   = [193.814, 22.227, 29.065, 130.038, 151.771, 242.287, 7.888, 0.0]
Fv_  = [0.522, 0.440, 0.604, 0.813, 0.029, 0.072, 0.084, 0.0]
Fo_  = [0.012, 0.025, 0.008, -0.058, 0.005, 0.009, -0.059, 0.0]

# Select the axis to compare (e.g., axis 0)
index = 6 

# Linear friction model
def linear_friction_model(v, Fv, Fo):
    return Fv * v + Fo

# Hyperbolic tangent–based friction model
def tanh_friction_model(v, Fc, k, Fv, Fo):
    return 0.85 * Fc * np.tanh(0.1*k * v) + Fv * v + Fo


# Generate velocity range
v_data = np.linspace(-2, 2, 1000)

# Compute friction values
y_linear = linear_friction_model(v_data, Dn_L[index], 0.0)
y_tanh = tanh_friction_model(v_data, Fc_[index], k_[index], Fv_[index], Fo_[index])

# Plot
plt.figure(figsize=(8, 4))
plt.plot(v_data, y_linear, label="Linear Model", linestyle="--")
plt.plot(v_data, y_tanh, label="Tanh Model")
plt.title(f"Friction Model Comparison (Joint {index})")
plt.xlabel("Velocity")
plt.ylabel("Torque")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

