import numpy as np
import modern_robotics as mr


SQRT2_OVER_2 = np.sqrt(2) / 2


TCE_GRASP = np.array([
    [-SQRT2_OVER_2, 0.0,  SQRT2_OVER_2, 0.0],
    [0.0,             1.0,  0.0,          0.0],
    [-SQRT2_OVER_2, 0.0, -SQRT2_OVER_2, 0.0],
    [0.0,             0.0,  0.0,          1.0]
])

TCE_STANDOFF = np.array([
    [-SQRT2_OVER_2, 0.0,  SQRT2_OVER_2, 0.0],
    [0.0,             1.0,  0.0,          0.0],
    [-SQRT2_OVER_2, 0.0, -SQRT2_OVER_2, 0.10],
    [0.0,             0.0,  0.0,          1.0]
])

TSC_INITIAL = np.array([
    [1.0, 0.0, 0.0, 1.0],
    [0.0, 1.0, 0.0, 0.0],
    [0.0, 0.0, 1.0, 0.025],
    [0.0, 0.0, 0.0, 1.0]
])


TSC_FINAL = np.array([
    [0.0,  1.0, 0.0,  0.0],
    [-1.0, 0.0, 0.0, -1.0],
    [0.0,  0.0, 1.0,  0.025],
    [0.0,  0.0, 0.0,  1.0]
])

TSE_INITIAL = np.array([
    [0.0, 0.0,  1.0, 0.0],
    [0.0, 1.0,  0.0, 0.0],
    [-1.0, 0.0, 0.0, 0.5],
    [0.0, 0.0,  0.0, 1.0]
])


TSE_GRASP_INITIAL = TSC_INITIAL @ TCE_GRASP
TSE_STANDOFF_INITIAL = TSC_INITIAL @ TCE_STANDOFF

TSE_GRASP_FINAL = TSC_FINAL @ TCE_GRASP
TSE_STANDOFF_FINAL = TSC_FINAL @ TCE_STANDOFF

DT = 0.01
SEGMENT_1_TIME = 5.0
SEGMENT_2_TIME = 1.0
SEGMENT_4_TIME = 1.0
SEGMENT_5_TIME = 5.0
SEGMENT_6_TIME = 1.0
SEGMENT_8_TIME = 1.0

GRIPPER_HOLD_SAMPLES = 63
segment1_time = 5.0

segment1_samples = int(
    segment1_time / DT
) + 1

segment1 = mr.ScrewTrajectory(
    TSE_INITIAL,
    TSE_STANDOFF_INITIAL,
    segment1_time,
    segment1_samples,
    5
)

def screw_segment(start_pose, end_pose, duration, dt=0.01):
    """
    Generate a quintic screw trajectory between two SE(3) poses.
    """

    samples = int(duration / dt) + 1

    return mr.ScrewTrajectory(
        start_pose,
        end_pose,
        duration,
        samples,
        5
    )

def trajectory_generator(
    Tse_initial,
    Tsc_initial,
    Tsc_final,
    Tce_grasp,
    Tce_standoff,
    dt=0.01
):
    """
    Generate the eight-segment reference end-effector trajectory.

    Returns
    -------
    trajectory : list
        List of tuples:
        (Tse, gripper_state)
    """

    # -------------------------------------------------
    # Compute grasp and standoff poses in the space frame
    # -------------------------------------------------

    Tse_grasp_initial = Tsc_initial @ Tce_grasp
    Tse_standoff_initial = Tsc_initial @ Tce_standoff

    Tse_grasp_final = Tsc_final @ Tce_grasp
    Tse_standoff_final = Tsc_final @ Tce_standoff

    trajectory = []

    # -------------------------------------------------
    # Segment 1:
    # Initial reference pose -> initial standoff
    # Gripper open
    # -------------------------------------------------

    segment1 = screw_segment(
        Tse_initial,
        Tse_standoff_initial,
        SEGMENT_1_TIME,
        dt
    )

    for pose in segment1:
        trajectory.append((pose, 0))

    # -------------------------------------------------
    # Segment 2:
    # Initial standoff -> initial grasp
    # Gripper open
    # -------------------------------------------------

    segment2 = screw_segment(
        Tse_standoff_initial,
        Tse_grasp_initial,
        SEGMENT_2_TIME,
        dt
    )

    # Skip first point because it duplicates
    # the final point of Segment 1.
    for pose in segment2[1:]:
        trajectory.append((pose, 0))

    # -------------------------------------------------
    # Segment 3:
    # Hold grasp pose while closing gripper
    # -------------------------------------------------

    for _ in range(GRIPPER_HOLD_SAMPLES):
        trajectory.append(
            (Tse_grasp_initial.copy(), 1)
        )

    # -------------------------------------------------
    # Segment 4:
    # Initial grasp -> initial standoff
    # Gripper closed
    # -------------------------------------------------

    segment4 = screw_segment(
        Tse_grasp_initial,
        Tse_standoff_initial,
        SEGMENT_4_TIME,
        dt
    )

    for pose in segment4[1:]:
        trajectory.append((pose, 1))

    # -------------------------------------------------
    # Segment 5:
    # Initial standoff -> final standoff
    # Carry cube
    # -------------------------------------------------

    segment5 = screw_segment(
        Tse_standoff_initial,
        Tse_standoff_final,
        SEGMENT_5_TIME,
        dt
    )

    for pose in segment5[1:]:
        trajectory.append((pose, 1))

    # -------------------------------------------------
    # Segment 6:
    # Final standoff -> final grasp
    # Lower cube
    # -------------------------------------------------

    segment6 = screw_segment(
        Tse_standoff_final,
        Tse_grasp_final,
        SEGMENT_6_TIME,
        dt
    )

    for pose in segment6[1:]:
        trajectory.append((pose, 1))

    # -------------------------------------------------
    # Segment 7:
    # Hold final grasp pose while opening gripper
    # -------------------------------------------------

    for _ in range(GRIPPER_HOLD_SAMPLES):
        trajectory.append(
            (Tse_grasp_final.copy(), 0)
        )

    # -------------------------------------------------
    # Segment 8:
    # Final grasp -> final standoff
    # Gripper open
    # -------------------------------------------------

    segment8 = screw_segment(
        Tse_grasp_final,
        Tse_standoff_final,
        SEGMENT_8_TIME,
        dt
    )

    for pose in segment8[1:]:
        trajectory.append((pose, 0))

    return trajectory

def pose_to_csv_row(Tse, gripper_state):
    """
    Convert a 4x4 SE(3) pose and gripper state
    into the 13-value Milestone 2 CSV format.
    """

    R = Tse[0:3, 0:3]
    p = Tse[0:3, 3]

    row = np.concatenate([
        R.flatten(),
        p,
        [gripper_state]
    ])

    return row

def save_trajectory_csv(trajectory, filename):

    rows = []

    for pose, gripper_state in trajectory:
        row = pose_to_csv_row(
            pose,
            gripper_state
        )

        rows.append(row)

    data = np.array(rows)

    np.savetxt(
        filename,
        data,
        delimiter=",",
        fmt="%.8f"
    )

    return data

if __name__ == "__main__":

    trajectory = trajectory_generator(
        TSE_INITIAL,
        TSC_INITIAL,
        TSC_FINAL,
        TCE_GRASP,
        TCE_STANDOFF
    )

    data = save_trajectory_csv(
        trajectory,
        "results/milestone2/reference_trajectory.csv"
    )

    print("Trajectory generated successfully.")
    print("Number of configurations:", len(trajectory))
    print("CSV shape:", data.shape)

    print("\nFirst pose:")
    print(trajectory[0][0])
    print("Gripper:", trajectory[0][1])

    print("\nFinal pose:")
    print(trajectory[-1][0])
    print("Gripper:", trajectory[-1][1])