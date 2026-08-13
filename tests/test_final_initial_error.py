from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

import numpy as np
import modern_robotics as mr

from feedback_control import end_effector_configuration


TSE_REFERENCE_INITIAL = np.array([
    [0, 0,  1, 0.0],
    [0, 1,  0, 0.0],
    [-1, 0, 0, 0.5],
    [0, 0,  0, 1.0]
])


configuration = np.array([
    np.pi / 4, 0.0, 0.0,
    0.0, 0.0, 0.2, -1.6, 0.0,
    0.0, 0.0, 0.0, 0.0
])


X = end_effector_configuration(
    configuration[0:3],
    configuration[3:8]
)


# Position error
position_error = np.linalg.norm(
    X[:3, 3]
    - TSE_REFERENCE_INITIAL[:3, 3]
)


# Orientation error
Rerr = (
    X[:3, :3].T
    @ TSE_REFERENCE_INITIAL[:3, :3]
)

orientation_error_rad = np.linalg.norm(
    mr.so3ToVec(
        mr.MatrixLog3(Rerr)
    )
)

orientation_error_deg = np.degrees(
    orientation_error_rad
)


print("Actual initial X:")
print(X)

print(
    "\nPosition error:",
    position_error,
    "m"
)

print(
    "Orientation error:",
    orientation_error_deg,
    "degrees"
)

print(
    "\nPosition requirement satisfied:",
    position_error >= 0.2
)

print(
    "Orientation requirement satisfied:",
    orientation_error_deg >= 30
)