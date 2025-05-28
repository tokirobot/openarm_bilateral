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

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("data/velocity_comparison.csv") # Adjust this path as needed

Ts = 0.002  

time = np.arange(len(df)) * Ts

plt.figure(figsize=(12, 6))

plt.plot(time, df['differ_velocity'], label='Differentiated Velocity', linewidth=2)
plt.plot(time, df['motor_velocity'], label='Motor Sensor Velocity', linestyle='--', linewidth=2)

plt.xlabel('Time [s]')
plt.ylabel('Velocity [rad/s]')
plt.title('Velocity Comparison')
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
