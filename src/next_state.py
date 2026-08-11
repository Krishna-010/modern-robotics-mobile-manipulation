import numpy as np


WHEEL_RADIUS = 0.0475
HALF_LENGTH = 0.235
HALF_WIDTH = 0.15


def next_state(configuration, controls, dt, max_speed):

    configuration = np.array(configuration, dtype=float)
    controls = np.array(controls, dtype=float)

    # Apply velocity limits.
    controls = np.clip(
        controls,
        -max_speed,
        max_speed
    )

    # Split configuration.
    chassis = configuration[0:3]
    arm_angles = configuration[3:8]
    wheel_angles = configuration[8:12]

    # Split controls.
    wheel_speeds = controls[0:4]
    joint_speeds = controls[4:9]

    # Integrate arm joints.
    new_arm_angles = (
        arm_angles
        + joint_speeds * dt
    )

    # Integrate wheel angles.
    wheel_increments = wheel_speeds * dt

    new_wheel_angles = (
        wheel_angles
        + wheel_increments
    )

    # youBot mecanum-wheel kinematics.
    l = HALF_LENGTH
    w = HALF_WIDTH
    r = WHEEL_RADIUS

    F = (r / 4.0) * np.array([
        [
            -1.0 / (l + w),
             1.0 / (l + w),
             1.0 / (l + w),
            -1.0 / (l + w)
        ],

        [1.0, 1.0, 1.0, 1.0],

        [-1.0, 1.0, -1.0, 1.0]
    ])

    # Chassis body-twist increment.
    body_twist = F @ wheel_increments

    delta_phi = body_twist[0]
    vbx = body_twist[1]
    vby = body_twist[2]

    # Integrate body motion.
    if abs(delta_phi) < 1e-9:

        delta_q_body = np.array([
            0.0,
            vbx,
            vby
        ])

    else:

        delta_q_body = np.array([
            delta_phi,

            (
                vbx * np.sin(delta_phi)
                + vby * (
                    np.cos(delta_phi) - 1.0
                )
            ) / delta_phi,

            (
                vby * np.sin(delta_phi)
                + vbx * (
                    1.0 - np.cos(delta_phi)
                )
            ) / delta_phi
        ])

    # Convert body-frame displacement
    # into space-frame displacement.
    phi = chassis[0]

    body_to_space = np.array([
        [1.0, 0.0, 0.0],

        [
            0.0,
            np.cos(phi),
            -np.sin(phi)
        ],

        [
            0.0,
            np.sin(phi),
            np.cos(phi)
        ]
    ])

    delta_q_space = (
        body_to_space @ delta_q_body
    )

    # Update chassis.
    new_chassis = (
        chassis + delta_q_space
    )

    # Assemble the new 12-vector.
    new_configuration = np.concatenate([
        new_chassis,
        new_arm_angles,
        new_wheel_angles
    ])

    return new_configuration