import numpy as np
from next_state import next_state


dt = 0.01
max_speed = 100.0

configuration = np.zeros(12)


# -------------------------------------
# Test 1: Forward
# -------------------------------------

forward_controls = np.array([
    10, 10, 10, 10,
    0, 0, 0, 0, 0
])

forward_result = next_state(
    configuration,
    forward_controls,
    dt,
    max_speed
)

print("Forward:")
print(forward_result)


# -------------------------------------
# Test 2: Sideways
# -------------------------------------

sideways_controls = np.array([
    -10, 10, -10, 10,
    0, 0, 0, 0, 0
])

sideways_result = next_state(
    configuration,
    sideways_controls,
    dt,
    max_speed
)

print("\nSideways:")
print(sideways_result)


# -------------------------------------
# Test 3: Rotation
# -------------------------------------

rotation_controls = np.array([
    -10, 10, 10, -10,
    0, 0, 0, 0, 0
])

rotation_result = next_state(
    configuration,
    rotation_controls,
    dt,
    max_speed
)

print("\nRotation:")
print(rotation_result)

# -------------------------------------
# Test 4: Forward while facing 90 degrees
# -------------------------------------

rotated_configuration = np.zeros(12)
rotated_configuration[0] = np.pi / 2

rotated_forward_result = next_state(
    rotated_configuration,
    forward_controls,
    dt,
    max_speed
)

print("\nForward while facing 90 degrees:")
print(rotated_forward_result)