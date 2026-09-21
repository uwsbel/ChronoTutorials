"""
Compute error metrics between monolithic (benchmark) and co-simulation results
for the two-mass-spring system.

For each of the four quantities (applied_force, x_chrono, x_simscape, spring_force)
reports:
  - RMSE        : root-mean-square error, in the quantity's physical units
  - NRMSE       : RMSE normalized by the monolithic signal's (max - min) range
  - max_abs_err : worst-case absolute deviation

The co-sim signals are interpolated onto the monolithic time grid (restricted to
the overlapping time range) so the two runs can be compared even when their time
vectors differ (e.g. different communication step sizes).

Edit MONO_CSV and COSIM_CSV below to point at your two result files.
"""

import sys
import os
import numpy as np
import pandas as pd


# -----------------------------------------------------------------------------
# Hardcoded input paths -- edit these to point at your two CSV files.
# -----------------------------------------------------------------------------
MONO_CSV = r"C:\Users\ahmed\Documents\sbel\magic_2026\tutorials\spring_system\monolithic\results\two_mass_spring.csv"
COSIM_CSV = r"C:\Users\ahmed\Documents\sbel\magic_2026\tutorials\spring_system\fmi_implementation\results\cosim_results.csv"

EXPECTED = ["applied_force", "x_chrono", "x_simscape", "spring_force"]
ALLCOLS = {"t", *EXPECTED}


def load_csv(path, label):
    if not os.path.isfile(path):
        print(f"ERROR: {label} CSV not found at: {path}")
        sys.exit(1)
    df = pd.read_csv(path)
    missing = ALLCOLS - set(df.columns)
    if missing:
        print(f"ERROR: {label} CSV is missing columns: {missing}")
        print(f"Found columns: {list(df.columns)}")
        sys.exit(1)
    return df


def main():
    mono = load_csv(MONO_CSV, "monolithic")
    cosim = load_csv(COSIM_CSV, "co-sim")

    # Common time grid = monolithic time vector, restricted to the overlap range
    t = mono["t"].to_numpy()
    t_lo = max(mono["t"].min(), cosim["t"].min())
    t_hi = min(mono["t"].max(), cosim["t"].max())
    mask = (t >= t_lo) & (t <= t_hi)
    t = t[mask]

    print(f"Comparing over t = [{t_lo:.4g}, {t_hi:.4g}] s "
          f"on {len(t)} monolithic time points\n")

    header = f"{'quantity':<15}{'RMSE':>14}{'NRMSE':>12}{'max_abs_err':>16}"
    print(header)
    print("-" * len(header))

    for col in EXPECTED:
        ref = mono[col].to_numpy()[mask]                              # benchmark
        cmp = np.interp(t, cosim["t"].to_numpy(),
                        cosim[col].to_numpy())                        # co-sim on grid

        diff = cmp - ref
        rmse = np.sqrt(np.mean(diff ** 2))

        rng = ref.max() - ref.min()
        nrmse = rmse / rng if rng > 0 else float("nan")

        max_abs = np.max(np.abs(diff))

        print(f"{col:<15}{rmse:>14.6e}{nrmse:>12.4%}{max_abs:>16.6e}")

    print()


if __name__ == "__main__":
    main()