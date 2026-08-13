from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

import numpy as np

from feedback_control import test_joint_limits


tests = [
    # Valid configuration
    np.array([0.0, 0.0, 0.2, -1.6, 0.0]),

    # J1 above limit
    np.array([3.0, 0.0, 0.2, -1.6, 0.0]),

    # J2 above limit
    np.array([0.0, 1.6, 0.2, -1.6, 0.0]),

    # J3 above limit
    np.array([0.0, 0.0, 2.6, -1.6, 0.0]),

    # J4 below limit
    np.array([0.0, 0.0, 0.2, -1.9, 0.0]),

    # J5 above limit
    np.array([0.0, 0.0, 0.2, -1.6, 3.0]),
]


for arm in tests:
    print("\nArm:")
    print(arm)

    print("Violations:")
    print(test_joint_limits(arm))