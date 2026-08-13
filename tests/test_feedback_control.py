from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

import numpy as np

from feedback_control import (
    end_effector_configuration,
    feedback_control,
    mobile_manipulator_jacobian,
    calculate_controls
)


DT = 0.01


# -------------------------------------------------
# Actual robot configuration from official test
# -------------------------------------------------

chassis = np.array([
    0.0,
    0.0,
    0.0
])

arm_angles = np.array([
    0.0,
    0.0,
    0.2,
    -1.6,
    0.0
])

X = end_effector_configuration(
    chassis,
    arm_angles
)


# -------------------------------------------------
# Desired configurations from official test
# -------------------------------------------------

Xd = np.array([
    [0.0, 0.0,  1.0, 0.5],
    [0.0, 1.0,  0.0, 0.0],
    [-1.0, 0.0, 0.0, 0.5],
    [0.0, 0.0,  0.0, 1.0]
])


Xd_next = np.array([
    [0.0, 0.0,  1.0, 0.6],
    [0.0, 1.0,  0.0, 0.0],
    [-1.0, 0.0, 0.0, 0.3],
    [0.0, 0.0,  0.0, 1.0]
])


# Feedforward only.
Kp = np.zeros((6, 6))
Ki = np.zeros((6, 6))

integral_error = np.zeros(6)


V, Xerr, integral_error_new, Vd = feedback_control(
    X,
    Xd,
    Xd_next,
    Kp,
    Ki,
    integral_error,
    DT
)


print("Vd:")
print(Vd)

print("\nXerr:")
print(Xerr)

print("\nIntegral increment:")
print(integral_error_new)

print("\nCommanded V:")
print(V)

Je = mobile_manipulator_jacobian(
    arm_angles
)

controls = calculate_controls(
    Je,
    V
)

print("\nJe shape:")
print(Je.shape)

print("\nJe:")
print(Je)

print("\nControls [wheels, arm joints]:")
print(controls)