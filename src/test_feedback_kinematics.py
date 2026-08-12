import numpy as np

from feedback_control import (
    BLIST,
    end_effector_configuration
)


chassis = np.array([
    0.0,   # phi
    0.0,   # x
    0.0    # y
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


print("BLIST shape:")
print(BLIST.shape)

print("\nCalculated X:")
print(X)