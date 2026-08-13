from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

import numpy as np
import modern_robotics as mr

T=np.eye(4)
print("Test Transformation")
print(T)
print("\nInverse")
print(mr.TransInv(T))
print("\nSteup working properly")