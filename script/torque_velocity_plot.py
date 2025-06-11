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
from scipy.optimize import curve_fit

# --- Model Definitions ---

# Linear model
def linear_fit(x, a, b):
    return a * x + b

# Gaussian smoothed friction model
#def smooth_friction_model(v, Fs, Fc, vs, Fv):
#    sign_v = np.sign(v)
#    return Fc * sign_v + (Fs - Fc) * np.exp(-(v / vs)**2) * sign_v + Fv * v

# Arctangent-based friction model
def atan_friction_model(v, Fc, k, Fv, Fo):
    return Fc * np.arctan(k * v) + Fv * v + Fo

# Hyperbolic tangent–based friction model
def tanh_friction_model(v, Fc, k, Fv, Fo):
    return Fc * np.tanh(k * v) + Fv * v + Fo

# Sigmoid–based friction model
def sigmoid_friction_model(v, Fc, k, Fv, Fo):
    return Fc * (2 / (1 + np.exp(-k * v)) - 1) + Fv * v + Fo  # symmetric sigmoid（-1 ~ +1）

# Data loading
df = pd.read_csv("../data/velocity_torque.csv")
x_data = df["actual_velocity"].values
y_data = df["velcommand"].values

# Linear fit for positive/negative velocities
df_plus = df[df["actual_velocity"] >= 0]
df_minus = df[df["actual_velocity"] < 0]
a_plus, b_plus = np.polyfit(df_plus["actual_velocity"], df_plus["velcommand"], 1)
a_minus, b_minus = np.polyfit(df_minus["actual_velocity"], df_minus["velcommand"], 1)

# Curve fitting for various models
#params_smooth, _ = curve_fit(smooth_friction_model, x_data, y_data, p0=[0.2, 0.1, 0.05, 0.05])
params_atan, _ = curve_fit(atan_friction_model, x_data, y_data, p0=[0.1, 1.0, 0.01, 0.0])
params_tanh, _ = curve_fit(tanh_friction_model, x_data, y_data, p0=[0.1, 1.0, 0.01, 0.0])
params_sigmoid, _ = curve_fit(sigmoid_friction_model, x_data, y_data, p0=[0.1, 5.0, 0.01, 0.0])

# Prepare plotting
x_fit = np.linspace(x_data.min(), x_data.max(), 1000)

plt.figure(figsize=(10, 6))
plt.scatter(x_data, y_data, s=20, alpha=0.5, label="Measured Data", color='blue')

# Linear fits split by sign
x_fit_plus = x_fit[x_fit >= 0]
x_fit_minus = x_fit[x_fit < 0]

# Linear fit (positive velocities)
plt.plot(x_fit_plus, linear_fit(x_fit_plus, a_plus, b_plus),
         'r--', label=f"Linear Fit (+): y={a_plus:.2f}x+{b_plus:.2f}")

# Linear fit (negative velocities)
plt.plot(x_fit_minus, linear_fit(x_fit_minus, a_minus, b_minus),
         'g--', label=f"Linear Fit (-): y={a_minus:.2f}x+{b_minus:.2f}")


# Gaussian
#y_smooth = smooth_friction_model(x_fit, *params_smooth)
#plt.plot(x_fit, y_smooth, 'orange', label="Smooth Friction (Gaussian)")

# atan
y_atan = atan_friction_model(x_fit, *params_atan)
plt.plot(x_fit, y_atan, 'purple', label="atan Friction")

# tanh
y_tanh = tanh_friction_model(x_fit, *params_tanh)
plt.plot(x_fit, y_tanh, 'brown', linestyle=":", label="tanh Friction")

# sigmoid
y_sigmoid = sigmoid_friction_model(x_fit, *params_sigmoid)
plt.plot(x_fit, y_sigmoid, 'magenta', linestyle="--", label="Sigmoid Friction")


# Print Fitted Parameters
print(f"[Linear +]   y = {a_plus:.3f} x + {b_plus:.3f}")
print(f"[Linear -]   y = {a_minus:.3f} x + {b_minus:.3f}")
#print(f"[Gaussian]   τ(v) = {params_smooth[1]:.3f} * sign(v) + ({params_smooth[0]:.3f} - {params_smooth[1]:.3f}) * exp(-(v/{params_smooth[2]:.3f})²) * sign(v) + {params_smooth[3]:.3f} * v")
print(f"[atan]       τ(v) = {params_atan[0]:.3f} * atan({params_atan[1]:.3f} * v) + {params_atan[2]:.3f} * v + {params_atan[3]:.3f}")
print(f"[tanh]       τ(v) = {params_tanh[0]:.3f} * tanh({params_tanh[1]:.3f} * v) + {params_tanh[2]:.3f} * v + {params_tanh[3]:.3f}")
print(f"[sigmoid]    τ(v) = {params_sigmoid[0]:.3f} * (2/(1+exp(-{params_sigmoid[1]:.3f} * v)) - 1) + {params_sigmoid[2]:.3f} * v + {params_sigmoid[3]:.3f}")

# Plot decorations
plt.xlabel("Actual Velocity [rad/s]")
plt.ylabel("Command Torque [Nm]")
plt.title("Friction Models Comparison")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

