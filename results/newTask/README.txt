Controller: Feedforward + P
Kp = 2I
Ki = 0

New cube task:
Initial: (x, y, phi) = (1.0, 0.5, 0)
Goal:    (x, y, phi) = (-0.5, -1.0, pi/2)

This case also uses predictive joint-limit avoidance and null-space posture control. Joint limits were determined using CoppeliaSim Scene 3. The enhanced controller completed the pick, transport, place, and retreat task while keeping the arm within the selected joint limits.
