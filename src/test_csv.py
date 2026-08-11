import numpy as np
data=np.zeros((5,13))
np.savetxt("/mnt/d/ROS_Workspaces/modern_robotics_capstone/results/test.csv", data, delimiter=",", fmt="%.6f")
print("CSV sucessfully created.")