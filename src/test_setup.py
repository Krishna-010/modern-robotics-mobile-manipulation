import numpy as np
import modern_robotics as mr

T=np.eye(4)
print("Test Transformation")
print(T)
print("\nInverse")
print(mr.TransInv(T))
print("\nSteup working properly")