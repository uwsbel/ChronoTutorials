"""
Compare monolithic vs co-simulation results for the two-mass-spring system.

Reads two CSVs, each with columns: t, applied_force, x_chrono, x_simscape, spring_force
  - monolithic CSV (the benchmark, from the Chrono-only build)
  - co-simulation CSV (from the Simscape <-> Chrono-FMU run)

Produces three stacked plots sharing the time axis:
  1. body_chrono position: monolithic vs co-sim
  2. body_simscape position: monolithic vs co-sim
  3. position error (co-sim - monolithic) for both bodies

The co-sim signals are interpolated onto the monolithic time grid before
overlay/error so the two runs can be compared even if their time vectors differ
(e.g. different communication step sizes).

Edit MONO_CSV and COSIM_CSV below to point at your two result files.
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# -----------------------------------------------------------------------------
# Hardcoded input paths -- edit these to point at your two CSV files.
# -----------------------------------------------------------------------------
MONO_CSV = r"C:\Users\ahmed\Documents\sbel\magic_2026\tutorials\spring_system\monolithic\results\two_mass_spring.csv"
COSIM_CSV = r"C:\Users\ahmed\Documents\sbel\magic_2026\tutorials\spring_system\fmi_implementation\results\cosim_results.csv"

# Legend position used for all three subplots
LEGEND_LOC = "upper right"

EXPECTED = {"t", "applied_force", "x_chrono", "x_simscape", "spring_force"}


def load_csv(path, label):
    if not os.path.isfile(path):
        print(f"ERROR: {label} CSV not found at: {path}")
        sys.exit(1)
    df = pd.read_csv(path)
    missing = EXPECTED - set(df.columns)
    if missing:
        print(f"ERROR: {label} CSV is missing columns: {missing}")
        print(f"Found columns: {list(df.columns)}")
        sys.exit(1)
    return df


def main():
    mono = load_csv(MONO_CSV, "monolithic")
    cosim = load_csv(COSIM_CSV, "co-sim")

    # Common time grid: use the monolithic time vector as the reference.
    # Restrict to the overlap of the two time ranges to avoid extrapolation.
    t = mono["t"].to_numpy()
    t_lo = max(mono["t"].min(), cosim["t"].min())
    t_hi = min(mono["t"].max(), cosim["t"].max())
    mask = (t >= t_lo) & (t <= t_hi)
    t = t[mask]

    def mono_on_grid(col):
        return mono[col].to_numpy()[mask]

    def cosim_on_grid(col):
        # interpolate the co-sim signal onto the monolithic time grid
        return np.interp(t, cosim["t"].to_numpy(), cosim[col].to_numpy())

    xc_mono = mono_on_grid("x_chrono")
    xs_mono = mono_on_grid("x_simscape")
    xc_cos = cosim_on_grid("x_chrono")
    xs_cos = cosim_on_grid("x_simscape")

    # Consistent visual encoding:
    #   monolithic = solid, co-sim = dashed
    #   chrono body = one color, simscape body = another
    c_chrono = "tab:blue"
    c_simscape = "tab:orange"

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

    # Plot 1: body_chrono position
    ax1.plot(t, xc_mono, color=c_chrono, linestyle="-", linewidth=1.5,
             label="monolithic")
    ax1.plot(t, xc_cos, color=c_chrono, linestyle="--", linewidth=1.5,
             label="co-sim")
    ax1.set_ylabel("x_chrono [m]")
    ax1.set_title("body_chrono position: monolithic vs co-sim")
    ax1.legend(loc=LEGEND_LOC)
    ax1.grid(True, alpha=0.3)

    # Plot 2: body_simscape position
    ax2.plot(t, xs_mono, color=c_simscape, linestyle="-", linewidth=1.5,
             label="monolithic")
    ax2.plot(t, xs_cos, color=c_simscape, linestyle="--", linewidth=1.5,
             label="co-sim")
    ax2.set_ylabel("x_simscape [m]")
    ax2.set_title("body_simscape position: monolithic vs co-sim")
    ax2.legend(loc=LEGEND_LOC)
    ax2.grid(True, alpha=0.3)

    # Plot 3: position error (co-sim - monolithic)
    ax3.plot(t, xc_cos - xc_mono, color=c_chrono, linestyle="-", linewidth=1.2,
             label="x_chrono error")
    ax3.plot(t, xs_cos - xs_mono, color=c_simscape, linestyle="-", linewidth=1.2,
             label="x_simscape error")
    ax3.axhline(0.0, color="k", linewidth=0.6, alpha=0.5)
    ax3.set_ylabel("error [m]")
    ax3.set_xlabel("time [s]")
    ax3.set_title("position error (co-sim - monolithic)")
    ax3.legend(loc=LEGEND_LOC)
    ax3.grid(True, alpha=0.3)

    fig.tight_layout()

    out_path = os.path.join(os.path.dirname(os.path.abspath(COSIM_CSV)),
                            "compare_mono_cosim.png")
    fig.savefig(out_path, dpi=150)
    print(f"Saved figure to: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()