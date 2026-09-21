"""
Plot the two-mass-spring simulation output from Chrono.

Reads a CSV with columns: t, applied_force, x_chrono, x_simscape, spring_force
Produces three stacked plots sharing the time axis:
  1. Both body x-positions vs time
  2. Spring force vs time
  3. Applied (driving) force vs time

Usage:
    python plot_two_mass_spring.py [path_to_csv]

If no path is given, defaults to ../results/two_mass_spring.csv
"""

import sys
import os
import pandas as pd
import matplotlib.pyplot as plt


def main():
    # Resolve CSV path: command-line arg, else default relative to this script
    if len(sys.argv) > 1:
        csv_path = sys.argv[1]
    else:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        csv_path = os.path.join(script_dir, "..", "results", "two_mass_spring.csv")

    if not os.path.isfile(csv_path):
        print(f"ERROR: CSV not found at: {csv_path}")
        sys.exit(1)

    # Load data
    df = pd.read_csv(csv_path)

    # Sanity check expected columns
    expected = {"t", "applied_force", "x_chrono", "x_simscape", "spring_force"}
    missing = expected - set(df.columns)
    if missing:
        print(f"ERROR: CSV is missing columns: {missing}")
        print(f"Found columns: {list(df.columns)}")
        sys.exit(1)

    # Three stacked subplots sharing the time axis
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 9), sharex=True)

    # Plot 1: body positions
    ax1.plot(df["t"], df["x_chrono"], label="body_chrono", linewidth=1.5)
    ax1.plot(df["t"], df["x_simscape"], label="body_simscape", linewidth=1.5)
    ax1.set_ylabel("x position (m)")
    ax1.set_title("Body x-positions vs time")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: spring force
    ax2.plot(df["t"], df["spring_force"], color="tab:green", linewidth=1.5)
    ax2.set_ylabel("spring force (N)")
    ax2.set_title("Spring force vs time")
    ax2.grid(True, alpha=0.3)

    # # Plot 3: applied driving force
    # ax3.plot(df["t"], df["applied_force"], color="tab:red", linewidth=1.5)
    # ax3.set_xlabel("time (s)")
    # ax3.set_ylabel("applied force (N)")
    # ax3.set_title("Applied force vs time")
    # ax3.grid(True, alpha=0.3)

    fig.tight_layout()

    # Save next to the CSV and also show
    out_path = os.path.splitext(csv_path)[0] + "_plot.png"
    fig.savefig(out_path, dpi=150)
    print(f"Saved plot to: {out_path}")

    plt.show()


if __name__ == "__main__":
    main()