"""
Compare monolithic vs co-simulation results for the two-mass-spring system.

Reads two CSVs, each with columns: t, applied_force, x_chrono, x_simscape, spring_force
  - monolithic CSV (the benchmark, from the Chrono-only build)
  - co-simulation CSV (from the Simscape <-> Chrono-FMU run)

Produces TWO vertically stacked plots sharing the time axis, sized for a poster
panel of 14 in wide x 10 in tall:
  1. Positions of body_chrono and body_simscape, monolithic vs co-sim
     (monolithic = solid, co-sim = dashed drawn ON TOP so the near-overlapping
      curves stay legible)
  2. Co-sim position error (co-sim - monolithic) for both bodies

The co-sim signals are interpolated onto the monolithic time grid before
overlay/error so the two runs can be compared even if their time vectors differ
(e.g. different communication step sizes).

Edit MONO_CSV and COSIM_CSV below to point at your two result files.
"""

import sys
import os
import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from matplotlib import font_manager


# -----------------------------------------------------------------------------
# Hardcoded input paths -- edit these to point at your two CSV files.
# -----------------------------------------------------------------------------
MONO_CSV = r"C:\Users\ahmed\Documents\sbel\magic_2026\tutorials\spring_system\monolithic\results\two_mass_spring.csv"
COSIM_CSV = r"C:\Users\ahmed\Documents\sbel\magic_2026\tutorials\spring_system\fmi_implementation\results\cosim_results.csv"

# Legend position used for both subplots
LEGEND_LOC = "upper right"

EXPECTED = {"t", "applied_force", "x_chrono", "x_simscape", "spring_force"}

# -----------------------------------------------------------------------------
# Poster sizing.
# Panel reserved on the poster: 14 in wide x 10 in tall.
# Font sizes are chosen so that, printed at this physical size, in-figure text
# visually matches ~36 pt poster body text and ~30 pt captions. matplotlib font
# sizes are in points and the figure is saved at its true physical size
# (14x10 in), so these point values ARE the printed point sizes.
# -----------------------------------------------------------------------------
FIG_W_IN = 14.0
FIG_H_IN = 10.0

FS_AXIS_LABEL = 32   # y/x axis labels  -> slightly under body text
FS_TICK       = 26   # tick numbers
FS_LEGEND     = 22   # legend text      -> noticeably smaller than before

LINE_MONO  = 2.6     # solid monolithic lines
LINE_COSIM = 2.2     # dashed co-sim lines (slightly thinner, drawn on top)
LINE_ERR   = 2.2

# -----------------------------------------------------------------------------
# Font: prefer Aptos (Microsoft's default since Office 2024/365), fall back
# through similar humanist sans fonts, then to matplotlib's bundled DejaVu Sans.
# On your Windows machine Aptos should be picked automatically. If you want to
# force a specific installed variant, set it first in this list.
# -----------------------------------------------------------------------------
PREFERRED_FONTS = ["Aptos", "Aptos Display", "Segoe UI", "Calibri",
                   "Helvetica", "Arial", "DejaVu Sans"]

_available = {f.name for f in font_manager.fontManager.ttflist}
_chosen = next((f for f in PREFERRED_FONTS if f in _available), "DejaVu Sans")
if _chosen not in ("Aptos", "Aptos Display"):
    print(f"NOTE: 'Aptos' not found on this machine; using '{_chosen}'. "
          f"On a machine with Aptos installed it will be used automatically.")

matplotlib.rcParams["font.family"] = "sans-serif"
matplotlib.rcParams["font.sans-serif"] = PREFERRED_FONTS
matplotlib.rcParams["axes.unicode_minus"] = False


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
    #   chrono body = one color, simscape body = another
    #   monolithic = solid, co-sim = dashed (drawn on top)
    c_chrono = "tab:blue"
    c_simscape = "tab:orange"

    fig, (ax1, ax2) = plt.subplots(
        2, 1, figsize=(FIG_W_IN, FIG_H_IN), sharex=True
    )

    # -- Plot 1: positions, monolithic (solid) with co-sim (dashed) on top ----
    # Draw solids first (lower zorder), then dashes on top (higher zorder) so
    # the dashed co-sim curve reads clearly where it nearly overlaps the solid.
    ax1.plot(t, xc_mono, color=c_chrono, linestyle="-", linewidth=LINE_MONO,
             zorder=1, label="body_chrono — monolithic")
    ax1.plot(t, xs_mono, color=c_simscape, linestyle="-", linewidth=LINE_MONO,
             zorder=1, label="body_simscape — monolithic")
    ax1.plot(t, xc_cos, color=c_chrono, linestyle=(0, (5, 3)),
             linewidth=LINE_COSIM, zorder=3, label="body_chrono — co-sim")
    ax1.plot(t, xs_cos, color=c_simscape, linestyle=(0, (5, 3)),
             linewidth=LINE_COSIM, zorder=3, label="body_simscape — co-sim")

    ax1.set_ylabel("Position [m]", fontsize=FS_AXIS_LABEL)
    ax1.tick_params(axis="both", labelsize=FS_TICK)
    ax1.grid(True, alpha=0.3)
    # Legend above the top plot, spanning full width, so it never covers the
    # near-overlapping position curves.
    ax1.legend(loc="lower center", bbox_to_anchor=(0.5, 1.02), ncol=2,
               fontsize=FS_LEGEND, framealpha=0.9, handlelength=2.6,
               columnspacing=1.4, labelspacing=0.3, borderaxespad=0.0)

    # -- Plot 2: co-sim position error --------------------------------------
    ax2.plot(t, xc_cos - xc_mono, color=c_chrono, linestyle="-",
             linewidth=LINE_ERR, label="body_chrono")
    ax2.plot(t, xs_cos - xs_mono, color=c_simscape, linestyle="-",
             linewidth=LINE_ERR, label="body_simscape")
    ax2.axhline(0.0, color="k", linewidth=0.8, alpha=0.5)
    ax2.set_ylabel("Error [m]", fontsize=FS_AXIS_LABEL)
    ax2.set_xlabel("Time [s]", fontsize=FS_AXIS_LABEL)
    ax2.tick_params(axis="both", labelsize=FS_TICK)
    ax2.grid(True, alpha=0.3)
    # Your real error stays near zero at the top-left and grows toward the
    # bottom-right, so the upper-left corner is the clear spot for the legend.
    ax2.legend(loc="upper left", fontsize=FS_LEGEND, ncol=1,
               framealpha=0.9, handlelength=2.6, columnspacing=1.4,
               labelspacing=0.3)

    # Reserve headroom at the top for the above-axes legend, then tighten.
    fig.tight_layout(pad=1.2, h_pad=1.8, rect=(0, 0, 1, 0.90))

    out_dir = os.path.dirname(os.path.abspath(COSIM_CSV))
    out_png = os.path.join(out_dir, "compare_mono_cosim.png")
    out_pdf = os.path.join(out_dir, "compare_mono_cosim.pdf")
    # 300 DPI raster for print, plus a vector PDF that scales perfectly on the poster.
    fig.savefig(out_png, dpi=300, bbox_inches="tight")
    fig.savefig(out_pdf, bbox_inches="tight")
    print(f"Saved figure to: {out_png}")
    print(f"Saved figure to: {out_pdf}")
    plt.show()


if __name__ == "__main__":
    main()