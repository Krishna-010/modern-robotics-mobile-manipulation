from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

from pathlib import Path

import numpy as np

from next_state import next_state

from trajectory_generator import (
    trajectory_generator,
    TSC_INITIAL,
    TSC_FINAL,
    TCE_GRASP,
    TCE_STANDOFF,
)

from feedback_control import (
    end_effector_configuration,
    feedback_control,
    mobile_manipulator_jacobian,
    calculate_controls,
)


DT = 0.01
MAX_SPEED = 100.0

OUTPUT_DIR = Path("results/milestone3")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def main():

    # -------------------------------------------------
    # Initial youBot configuration
    #
    # [phi, x, y,
    #  J1, J2, J3, J4, J5,
    #  W1, W2, W3, W4]
    # -------------------------------------------------

    configuration = np.array([
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.2, -1.6, 0.0,
        0.0, 0.0, 0.0, 0.0
    ])


    # -------------------------------------------------
    # Make the first reference pose equal to the
    # actual initial end-effector pose.
    #
    # This is important for feedforward-only control.
    # -------------------------------------------------

    Tse_initial = end_effector_configuration(
        configuration[0:3],
        configuration[3:8]
    )


    # -------------------------------------------------
    # Generate Milestone 2 reference trajectory.
    # -------------------------------------------------

    reference_trajectory = trajectory_generator(
        Tse_initial,
        TSC_INITIAL,
        TSC_FINAL,
        TCE_GRASP,
        TCE_STANDOFF,
        dt=DT
    )

    print(
        "Reference configurations:",
        len(reference_trajectory)
    )


    # -------------------------------------------------
    # Feedforward only:
    # Kp = Ki = 0
    # -------------------------------------------------

    Kp = np.zeros((6, 6))
    Ki = np.zeros((6, 6))

    integral_error = np.zeros(6)


    # -------------------------------------------------
    # Storage
    # -------------------------------------------------

    configuration_history = [
        np.concatenate([
            configuration.copy(),
            [reference_trajectory[0][1]]
        ])
    ]

    error_history = []

    peak_command = 0.0


    # -------------------------------------------------
    # Main control / simulation loop
    # -------------------------------------------------

    for i in range(
        len(reference_trajectory) - 1
    ):

        Xd, _ = reference_trajectory[i]

        Xd_next, next_gripper_state = (
            reference_trajectory[i + 1]
        )


        # ---------------------------------------------
        # Actual current end-effector pose X
        # ---------------------------------------------

        X = end_effector_configuration(
            configuration[0:3],
            configuration[3:8]
        )


        # ---------------------------------------------
        # Feedforward control law
        # ---------------------------------------------

        V, Xerr, integral_error, Vd = (
            feedback_control(
                X,
                Xd,
                Xd_next,
                Kp,
                Ki,
                integral_error,
                DT
            )
        )


        # ---------------------------------------------
        # Full mobile-manipulator Jacobian
        # ---------------------------------------------

        Je = mobile_manipulator_jacobian(
            configuration[3:8]
        )


        # ---------------------------------------------
        # Wheel + arm velocities
        # ---------------------------------------------

        controls = calculate_controls(
            Je,
            V
        )

        peak_command = max(
            peak_command,
            np.max(np.abs(controls))
        )


        # ---------------------------------------------
        # Simulate one timestep
        # ---------------------------------------------

        configuration = next_state(
            configuration,
            controls,
            DT,
            MAX_SPEED
        )


        # ---------------------------------------------
        # Save resulting configuration
        # ---------------------------------------------

        configuration_history.append(
            np.concatenate([
                configuration.copy(),
                [next_gripper_state]
            ])
        )

        error_history.append(
            Xerr.copy()
        )


    # -------------------------------------------------
    # Convert histories to arrays
    # -------------------------------------------------

    configuration_history = np.array(
        configuration_history
    )

    error_history = np.array(
        error_history
    )


    # -------------------------------------------------
    # Write CoppeliaSim configuration CSV
    # -------------------------------------------------

    configuration_file = (
        OUTPUT_DIR /
        "feedforward_configuration.csv"
    )

    np.savetxt(
        configuration_file,
        configuration_history,
        delimiter=",",
        fmt="%.8f"
    )


    # -------------------------------------------------
    # Write error data
    # -------------------------------------------------

    error_file = (
        OUTPUT_DIR /
        "feedforward_error.csv"
    )

    np.savetxt(
        error_file,
        error_history,
        delimiter=",",
        fmt="%.8f"
    )


    # -------------------------------------------------
    # Diagnostics
    # -------------------------------------------------

    final_X = end_effector_configuration(
        configuration[0:3],
        configuration[3:8]
    )

    final_Xd = reference_trajectory[-1][0]

    final_error = np.linalg.norm(
        feedback_error(final_X, final_Xd)
    )


    print("\nFeedforward simulation complete.")

    print(
        "Configuration CSV shape:",
        configuration_history.shape
    )

    print(
        "Error data shape:",
        error_history.shape
    )

    print(
        "Peak requested wheel/joint speed:",
        peak_command
    )

    print(
        "Final end-effector error norm:",
        final_error
    )

    print(
        "\nConfiguration CSV:",
        configuration_file
    )

    print(
        "Error CSV:",
        error_file
    )


def feedback_error(X, Xd):
    """
    Return the six-vector configuration error.
    """

    import modern_robotics as mr

    return mr.se3ToVec(
        mr.MatrixLog6(
            mr.TransInv(X) @ Xd
        )
    )


if __name__ == "__main__":
    main()