import numpy as np
import modern_robotics as mr

WHEEL_RADIUS = 0.0475
HALF_LENGTH = 0.235
HALF_WIDTH = 0.15

F = (WHEEL_RADIUS / 4.0) * np.array([
    [
        -1.0 / (HALF_LENGTH + HALF_WIDTH),
         1.0 / (HALF_LENGTH + HALF_WIDTH),
         1.0 / (HALF_LENGTH + HALF_WIDTH),
        -1.0 / (HALF_LENGTH + HALF_WIDTH)
    ],
    [1.0, 1.0, 1.0, 1.0],
    [-1.0, 1.0, -1.0, 1.0]
])

# Fixed transform from chassis frame {b}
# to arm base frame {0}.
TB0 = np.array([
    [1.0, 0.0, 0.0, 0.1662],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.0026],
    [0.0, 0.0, 0.0, 1.0]
])


# End-effector home configuration relative
# to the arm base frame {0}.
M0E = np.array([
    [1.0, 0.0, 0.0, 0.033],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.6546],
    [0.0, 0.0, 0.0, 1.0]
])


# Body screw axes for the five arm joints.
# Each COLUMN is one 6-vector Bi = [omega; v].
BLIST = np.array([
    [0.0,  0.0,      0.0,      0.0,     0.0],
    [0.0, -1.0,     -1.0,     -1.0,     0.0],
    [1.0,  0.0,      0.0,      0.0,     1.0],
    [0.0, -0.5076,  -0.3526,  -0.2176,  0.0],
    [0.033, 0.0,      0.0,      0.0,     0.0],
    [0.0,   0.0,      0.0,      0.0,     0.0]
])

PREFERRED_ARM = np.array([
    0.0,
    0.0,
    0.2,
    -1.6,
    0.0
])
# Arm joint limits measured from the interactive
# KUKA youBot model in CoppeliaSim Scene 3.
JOINT_MIN = np.array([
    -2.932,   # J1
    -1.117,   # J2
    -2.500,   # J3
    -1.780,   # J4
    -2.890    # J5
])

JOINT_MAX = np.array([
     2.932,   # J1
     1.553,   # J2
     2.500,   # J3
     1.780,   # J4
     2.890    # J5
])

def chassis_transform(phi, x, y):
    """
    Return Tsb, the chassis frame {b}
    relative to the space frame {s}.
    """

    return np.array([
        [
            np.cos(phi),
            -np.sin(phi),
            0.0,
            x
        ],
        [
            np.sin(phi),
            np.cos(phi),
            0.0,
            y
        ],
        [
            0.0,
            0.0,
            1.0,
            0.0963
        ],
        [
            0.0,
            0.0,
            0.0,
            1.0
        ]
    ])

def end_effector_configuration(
    chassis,
    arm_angles
):
    """
    Calculate Tse for the current youBot configuration.
    """

    phi, x, y = chassis

    Tsb = chassis_transform(phi, x, y)

    T0e = mr.FKinBody(
        M0E,
        BLIST,
        arm_angles
    )

    Tse = Tsb @ TB0 @ T0e

    return Tse

def feedback_control(
    X,
    Xd,
    Xd_next,
    Kp,
    Ki,
    integral_error,
    dt
):
    """
    Compute the commanded end-effector twist.

    Parameters
    ----------
    X : 4x4 array
        Current actual end-effector configuration.

    Xd : 4x4 array
        Current desired end-effector configuration.

    Xd_next : 4x4 array
        Desired end-effector configuration one timestep later.

    Kp : 6x6 array
        Proportional gain matrix.

    Ki : 6x6 array
        Integral gain matrix.

    integral_error : length-6 array
        Running integral of Xerr.

    dt : float
        Controller timestep.

    Returns
    -------
    V : length-6 array
        Commanded end-effector twist.

    Xerr : length-6 array
        Current end-effector error.

    new_integral_error : length-6 array
        Updated integral error.

    Vd : length-6 array
        Feedforward reference twist.
    """

    # Current configuration error.
    Xerr = mr.se3ToVec(
        mr.MatrixLog6(
            mr.TransInv(X) @ Xd
        )
    )

    # Update numerical integral.
    new_integral_error = (
        integral_error + Xerr * dt
    )

    # Desired twist taking Xd to Xd_next in dt seconds.
    Vd = (
        mr.se3ToVec(
            mr.MatrixLog6(
                mr.TransInv(Xd) @ Xd_next
            )
        )
        / dt
    )

    # Express desired feedforward twist in
    # the current actual end-effector frame.
    feedforward = (
        mr.Adjoint(
            mr.TransInv(X) @ Xd
        )
        @ Vd
    )

    # Full feedforward + PI control law.
    V = (
        feedforward
        + Kp @ Xerr
        + Ki @ new_integral_error
    )

    return V, Xerr, new_integral_error, Vd
def mobile_manipulator_jacobian(arm_angles):
    """
    Compute the full 6x9 Jacobian of the youBot.

    Column ordering:
        wheel1, wheel2, wheel3, wheel4,
        joint1, joint2, joint3, joint4, joint5
    """

    # Arm forward kinematics relative to frame {0}.
    T0e = mr.FKinBody(
        M0E,
        BLIST,
        arm_angles
    )

    # Transform from chassis frame {b}
    # to end-effector frame {e}.
    Tbe = TB0 @ T0e

    # Mecanum wheel kinematics.
    F = (WHEEL_RADIUS / 4.0) * np.array([
        [
            -1.0 / (HALF_LENGTH + HALF_WIDTH),
             1.0 / (HALF_LENGTH + HALF_WIDTH),
             1.0 / (HALF_LENGTH + HALF_WIDTH),
            -1.0 / (HALF_LENGTH + HALF_WIDTH)
        ],
        [1.0, 1.0, 1.0, 1.0],
        [-1.0, 1.0, -1.0, 1.0]
    ])

    # Embed planar chassis twist into a 6D twist.
    F6 = np.zeros((6, 4))
    F6[2:5, :] = F

    # Base Jacobian expressed in the end-effector frame.
    Jbase = (
        mr.Adjoint(
            mr.TransInv(Tbe)
        )
        @ F6
    )

    # Arm body Jacobian.
    Jarm = mr.JacobianBody(
        BLIST,
        arm_angles
    )

    # Full 6x9 mobile-manipulator Jacobian.
    Je = np.hstack([
        Jbase,
        Jarm
    ])

    return Je

def calculate_controls(Je, V):
    """
    Convert desired end-effector twist into
    wheel and arm joint velocities.
    """

    return np.linalg.pinv(Je) @ V

def test_joint_limits(arm_angles):
    """
    Check the arm configuration against the joint limits
    measured from CoppeliaSim Scene 3.

    Parameters
    ----------
    arm_angles : array-like, shape (5,)
        Current or predicted youBot arm joint angles.

    Returns
    -------
    violations : ndarray of bool, shape (5,)
        True for each joint that lies outside its
        allowed range.
    """

    arm_angles = np.asarray(arm_angles)

    return (
        (arm_angles < JOINT_MIN)
        | (arm_angles > JOINT_MAX)
    )

def weighted_mobile_pseudoinverse(
    Je,
    wheel_scale=10.0,
    rcond=1e-3
):
    """
    Weighted generalized inverse for the mobile manipulator.

    wheel_scale > 1 makes wheel motion comparatively
    cheaper than arm motion, encouraging use of the base.
    """

    scales = np.array([
        wheel_scale,
        wheel_scale,
        wheel_scale,
        wheel_scale,
        1.0,
        1.0,
        1.0,
        1.0,
        1.0
    ])

    S = np.diag(scales)

    return (
        S
        @ np.linalg.pinv(
            Je @ S,
            rcond=rcond
        )
    )

def calculate_controls_with_joint_limits(
    Je,
    V,
    arm_angles,
    dt,
    centering_gain=2.0,
    pinv_rcond=1e-3
):
    """
    Mobile-manipulator control with:
      1. end-effector tracking
      2. null-space posture control
      3. predictive hard joint-limit avoidance
      4. singular-value tolerance
    """

    Je_limited = Je.copy()

    disabled_joints = np.zeros(
        5,
        dtype=bool
    )

    for _ in range(6):

        Je_pinv = np.linalg.pinv(
            Je_limited,
            rcond=pinv_rcond
        )

        # Primary task
        primary_controls = Je_pinv @ V

        # Secondary posture task
        preferred_controls = np.zeros(9)

        preferred_controls[4:9] = (
            centering_gain
            * (PREFERRED_ARM - arm_angles)
        )

        preferred_controls[4:9][
            disabled_joints
        ] = 0.0

        # Null-space projector
        null_space = (
            np.eye(9)
            - Je_pinv @ Je_limited
        )

        controls = (
            primary_controls
            + null_space @ preferred_controls
        )

        # Predict next arm pose
        predicted_arm = (
            arm_angles
            + controls[4:9] * dt
        )

        violated = test_joint_limits(
            predicted_arm
        )

        new_violations = (
            violated
            & ~disabled_joints
        )

        if not np.any(new_violations):
            return controls

        # Disable any newly offending joints
        for joint_index in np.where(
            new_violations
        )[0]:

            Je_limited[
                :,
                4 + joint_index
            ] = 0.0

            disabled_joints[
                joint_index
            ] = True

    return controls