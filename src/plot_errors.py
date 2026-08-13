from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


DT = 0.01

CASES = {
    "best": Path("results/best/Xerr.csv"),
    "overshoot": Path("results/overshoot/Xerr.csv"),
    "newTask": Path("results/newTask/Xerr.csv"),
}


def plot_error(case_name, error_file):

    errors = np.loadtxt(
        error_file,
        delimiter=","
    )

    time = np.arange(len(errors)) * DT

    labels = [
        r"$\omega_x$",
        r"$\omega_y$",
        r"$\omega_z$",
        r"$v_x$",
        r"$v_y$",
        r"$v_z$",
    ]

    plt.figure(figsize=(10, 6))

    for i in range(6):
        plt.plot(
            time,
            errors[:, i],
            label=labels[i]
        )

    plt.xlabel("Time (s)")
    plt.ylabel("End-Effector Error")
    plt.title(
        f"End-Effector Tracking Error — {case_name}"
    )

    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    output_dir = error_file.parent

    plt.savefig(
        output_dir / "Xerr_plot.pdf"
    )

    plt.savefig(
        output_dir / "Xerr_plot.png",
        dpi=300
    )

    plt.close()

    print(
        f"Saved plots for {case_name}"
    )


def main():

    for case_name, error_file in CASES.items():

        plot_error(
            case_name,
            error_file
        )

    print("\nDone.")


if __name__ == "__main__":
    main()