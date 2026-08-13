Controller: Feedforward + PI
Kp = 2I
Ki = 4I

Default cube task:
Initial: (x, y, phi) = (1, 0, 0)
Goal:    (x, y, phi) = (0, -1, -pi/2)

The controller intentionally exhibits overshoot and damped oscillation during the initial transient while still converging sufficiently before the grasp.
