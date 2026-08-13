from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

import numpy as np


r = 0.0475
l = 0.235
w = 0.15
dt = 0.01


F = (r / 4.0) * np.array([
    [-1.0 / (l + w),  1.0 / (l + w),
      1.0 / (l + w), -1.0 / (l + w)],

    [1.0, 1.0, 1.0, 1.0],

    [-1.0, 1.0, -1.0, 1.0]
])


forward = np.array([10, 10, 10, 10])
sideways = np.array([-10, 10, -10, 10])
rotation = np.array([-10, 10, 10, -10])


print("F matrix:")
print(F)

print("\nForward body increment:")
print(F @ (forward * dt))

print("\nSideways body increment:")
print(F @ (sideways * dt))

print("\nRotation body increment:")
print(F @ (rotation * dt))