from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

from pathlib import Path

import numpy as np
import modern_robotics as mr

from next_state import next_state

from trajectory_generator import (
    trajectory_generator,
    TSC_INITIAL,
    TSC_FINAL,
    TCE_GRASP,
    TCE_STANDOFF,
    TSE_INITIAL,
)

from feedback_control import (
    end_effector_configuration,
    feedback_control,
    mobile_manipulator_jacobian,
    calculate_controls,
)


DT = 0.01
MAX_SPEED = 100.0

OUTPUT_DIR = Path("results/final_feedforward")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


INITIAL_CONFIGURATION = np.array([
    np.pi / 4, 0.0, 0.0,
    0.0, 0.0, 0.2, -1.6, 0.0,
    0.0, 0.0, 0.0, 0.0
])


def main():

    configuration = INITIAL_CONFIGURATION.copy()

    # IMPORTANT:
    # Reference starts at the prescribed Tse,
    # NOT at the robot's actual starting X.
    reference = trajectory_generator(
        TSE_INITIAL,
        TSC_INITIAL,
        TSC_FINAL,
        TCE_GRASP,
        TCE_STANDOFF,
        dt=DT
    )

    Kp = np.zeros((6, 6))
    Ki = np.zeros((6, 6))

    integral_error = np.zeros(6)

    configurations = [
        np.concatenate([
            configuration.copy(),
            [reference[0][1]]
        ])
    ]

    errors = []

    for i in range(len(reference) - 1):

        Xd, _ = reference[i]
        Xd_next, next_gripper = reference[i + 1]

        X = end_effector_configuration(
            configuration[0:3],
            configuration[3:8]
        )

        V, Xerr, integral_error, _ = feedback_control(
            X,
            Xd,
            Xd_next,
            Kp,
            Ki,
            integral_error,
            DT
        )

        Je = mobile_manipulator_jacobian(
            configuration[3:8]
        )

        controls = calculate_controls(
            Je,
            V
        )

        configuration = next_state(
            configuration,
            controls,
            DT,
            MAX_SPEED
        )

        configurations.append(
            np.concatenate([
                configuration.copy(),
                [next_gripper]
            ])
        )

        errors.append(Xerr.copy())

    configurations = np.array(configurations)
    errors = np.array(errors)

    np.savetxt(
        OUTPUT_DIR / "feedforward_initial_error.csv",
        configurations,
        delimiter=",",
        fmt="%.8f"
    )

    np.savetxt(
        OUTPUT_DIR / "feedforward_initial_error_Xerr.csv",
        errors,
        delimiter=",",
        fmt="%.8f"
    )

    norms = np.linalg.norm(errors, axis=1)

    print("Initial Xerr:")
    print(errors[0])

    print("\nInitial error norm:", norms[0])
    print("Maximum error norm:", norms.max())
    print("Final error norm:", norms[-1])

    print(
        "\nCSV:",
        OUTPUT_DIR / "feedforward_initial_error.csv"
    )


if __name__ == "__main__":
    main()