from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

import numpy as np
data=np.zeros((5,13))
np.savetxt("/mnt/d/ROS_Workspaces/modern_robotics_capstone/results/test.csv", data, delimiter=",", fmt="%.6f")
print("CSV sucessfully created.")