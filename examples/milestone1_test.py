from pathlib import Path
import sys

SRC_DIR = Path(__file__).resolve().parents[1] / "src"
sys.path.insert(0, str(SRC_DIR))

from pathlib import Path

import numpy as np

from next_state import next_state


DT = 0.01
SIMULATION_TIME = 1.0
STEPS = int(SIMULATION_TIME / DT)

OUTPUT_DIR = Path("results/milestone1")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)


def run_test(name, controls, max_speed):
    configuration = np.zeros(12)

    # Include the initial configuration at t = 0.
    history = [configuration.copy()]

    for _ in range(STEPS):
        configuration = next_state(
            configuration,
            controls,
            DT,
            max_speed
        )

        history.append(configuration.copy())

    history = np.array(history)

    print(f"\n{name}")
    print("Final chassis [phi, x, y]:")
    print(configuration[0:3])

    print("Final wheel angles:")
    print(configuration[8:12])

    # Add gripper state = 0 (open) as the 13th column.
    gripper_state = np.zeros((history.shape[0], 1))

    csv_data = np.hstack([
        history,
        gripper_state
    ])

    filename = (
        name.lower()
        .replace(" ", "_")
        + ".csv"
    )

    output_path = OUTPUT_DIR / filename

    np.savetxt(
        output_path,
        csv_data,
        delimiter=",",
        fmt="%.8f"
    )

    print("CSV saved to:")
    print(output_path)

    return history


forward_controls = np.array([
    10, 10, 10, 10,
    0, 0, 0, 0, 0
])

sideways_controls = np.array([
    -10, 10, -10, 10,
    0, 0, 0, 0, 0
])

rotation_controls = np.array([
    -10, 10, 10, -10,
    0, 0, 0, 0, 0
])


forward_history = run_test(
    "Forward",
    forward_controls,
    max_speed=100
)

sideways_history = run_test(
    "Sideways",
    sideways_controls,
    max_speed=100
)

rotation_history = run_test(
    "Counterclockwise Rotation",
    rotation_controls,
    max_speed=100
)

forward_limited_history = run_test(
    "Forward Limited",
    forward_controls,
    max_speed=5
)

sideways_limited_history = run_test(
    "Sideways Limited",
    sideways_controls,
    max_speed=5
)

rotation_limited_history = run_test(
    "Rotation Limited",
    rotation_controls,
    max_speed=5
)