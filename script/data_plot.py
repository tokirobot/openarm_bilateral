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

# Specify axis indices (e.g., axes 0, 1, 2)
index = [1,2,3]

# Read CSV files
leader_data = pd.read_csv("data/leader_log_20250512_004923.csv")
follower_data = pd.read_csv("data/follower_log_20250512_004923.csv")

# Display number of data (rows)
print(f"Leader data length:   {len(leader_data)}")
print(f"Follower data length: {len(follower_data)}")

time = leader_data['time']

# Generate column names corresponding to the specified indices
pos_cols = [f'pos{i}' for i in index]
vel_cols = [f'vel{i}' for i in index]
eff_cols = [f'eff{i}' for i in index]

# Plot
fig, axes = plt.subplots(3, 1, figsize=(15, 12), sharex=True)

# Position
for col in pos_cols:
    axes[0].plot(time, leader_data[col], label=f"leader_{col}")
    axes[0].plot(time, follower_data[col], '--', label=f"follower_{col}")
axes[0].set_ylabel('Position')
axes[0].legend(fontsize='small', ncol=4)
axes[0].grid(True)

# Velocity
for col in vel_cols:
    axes[1].plot(time, leader_data[col], label=f"leader_{col}")
    axes[1].plot(time, follower_data[col], '--', label=f"follower_{col}")
axes[1].set_ylabel('Velocity')
axes[1].legend(fontsize='small', ncol=4)
axes[1].grid(True)

# Effort
for col in eff_cols:
    axes[2].plot(time, leader_data[col], label=f"leader_{col}")
    axes[2].plot(time, follower_data[col], '--', label=f"follower_{col}")
axes[2].set_xlabel('Time [s]')
axes[2].set_ylabel('Effort')
axes[2].legend(fontsize='small', ncol=4)
axes[2].grid(True)

plt.suptitle('Leader vs Follower: Position, Velocity, and Effort', fontsize=16)
plt.show()

