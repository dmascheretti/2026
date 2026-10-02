"""Plots and metrics for closed-loop simulation logs.

Usage:  python3 analysis/plot_sim.py sim/output/steps.csv [more.csv ...]

For each log writes docs/figures/sim_<name>_{position,attitude,thrust}.{png,pdf}
and prints a metrics table (also returned by compute_metrics for reuse).
"""
import csv
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

FIGURE_DIR = Path(__file__).resolve().parent.parent / "docs" / "figures"

# Dark theme. Series colors: validated categorical slots 1-3 (dark mode)
# of the reference palette; reference is a neutral dashed line.
SURFACE = "#1a1a19"
TEXT = "#ffffff"
TEXT_MUTED = "#c3c2b7"
GRID = "#3a3a38"
COLOR_TRUE = "#3987e5"   # blue
COLOR_EST = "#d95926"    # orange
COLOR_CMD = "#199e70"    # aqua
COLOR_REF = TEXT_MUTED

SETTLE_TIME = 2.0  # s, take-off transient excluded from tracking metrics


def setup_style():
    plt.rcParams.update({
        "figure.facecolor": SURFACE,
        "axes.facecolor": SURFACE,
        "savefig.facecolor": SURFACE,
        "axes.edgecolor": GRID,
        "axes.labelcolor": TEXT,
        "axes.titlecolor": TEXT,
        "xtick.color": TEXT_MUTED,
        "ytick.color": TEXT_MUTED,
        "text.color": TEXT,
        "grid.color": GRID,
        "grid.linewidth": 0.6,
        "axes.grid": True,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "lines.linewidth": 1.6,
        "legend.frameon": False,
        "font.size": 10,
    })


def load_log(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    return {key: np.array([float(r[key]) for r in rows]) for key in rows[0]}


def rms(x):
    return float(np.sqrt(np.mean(x**2)))


def wrap(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


def compute_metrics(d):
    """Metrics while the motors run, after the take-off transient
    (SETTLE_TIME after the first active sample, i.e. after arming)."""
    first_active = d["t"][np.argmax(d["stopped"] == 0)]
    active = (d["stopped"] == 0) & (d["t"] >= first_active + SETTLE_TIME)
    m = {}
    track = np.stack([d[f"true_p{a}"] - d[f"ref_p{a}"] for a in "xyz"])[:, active]
    est_p = np.stack([d[f"est_p{a}"] - d[f"true_p{a}"] for a in "xyz"])[:, active]
    est_v = np.stack([d[f"est_v{a}"] - d[f"true_v{a}"] for a in "xyz"])[:, active]
    m["tracking_rms_xy_m"] = rms(np.linalg.norm(track[:2], axis=0))
    m["tracking_rms_z_m"] = rms(track[2])
    m["estimation_rms_xy_m"] = rms(np.linalg.norm(est_p[:2], axis=0))
    m["estimation_rms_z_m"] = rms(est_p[2])
    m["estimation_rms_v_mps"] = rms(np.linalg.norm(est_v, axis=0))
    m["estimation_rms_roll_pitch_rad"] = rms(np.concatenate([
        d["est_roll"][active] - d["true_roll"][active],
        d["est_pitch"][active] - d["true_pitch"][active]]))
    m["tilt_rms_rad"] = rms(np.hypot(d["true_roll"][active], d["true_pitch"][active]))
    m["max_solve_time_ms"] = 1000.0 * float(d["solve_time"].max())
    stopped = np.nonzero((d["stopped"] > 0) & (d["t"] > first_active))[0]
    m["stop_time_s"] = float(d["t"][stopped[0]]) if len(stopped) else float("nan")
    return m


def save(fig, name):
    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    # No creation date in the metadata, so unchanged figures stay unchanged in git.
    fig.savefig(FIGURE_DIR / f"{name}.png", dpi=150, bbox_inches="tight", metadata={"Software": None})
    fig.savefig(FIGURE_DIR / f"{name}.pdf", bbox_inches="tight", metadata={"CreationDate": None})
    plt.close(fig)


def legend_above(ax):
    ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.0), ncol=3, fontsize=8)


def mark_stop(ax, d):
    first_active = d["t"][np.argmax(d["stopped"] == 0)]
    stopped = np.nonzero((d["stopped"] > 0) & (d["t"] > first_active))[0]
    if len(stopped):
        t_stop = d["t"][stopped[0]]
        ax.axvline(t_stop, color=TEXT_MUTED, linewidth=1.0, linestyle=":")
        ax.annotate("motors off", (t_stop, 1.0), xycoords=("data", "axes fraction"),
                    xytext=(4, -12), textcoords="offset points", color=TEXT_MUTED, fontsize=8)


def plot_position(d, name):
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    for ax, axis in zip(axes, "xyz"):
        ax.plot(d["t"], d[f"ref_p{axis}"], color=COLOR_REF, linestyle="--", linewidth=1.2, label="reference")
        ax.plot(d["t"], d[f"true_p{axis}"], color=COLOR_TRUE, label="true")
        ax.plot(d["t"], d[f"est_p{axis}"], color=COLOR_EST, label="EKF estimate")
        ax.set_ylabel(f"{axis} [m]")
        mark_stop(ax, d)
    legend_above(axes[0])
    fig.suptitle(f"Position - {name}", x=0.08, ha="left")
    axes[-1].set_xlabel("time [s]")
    save(fig, f"sim_{name}_position")


def plot_attitude(d, name):
    fig, axes = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    deg = 180.0 / np.pi
    for ax, key, label in zip(axes[:2], ["roll", "pitch"], ["roll", "pitch"]):
        ax.plot(d["t"], d[f"cmd_{key}"] * deg, color=COLOR_CMD, linewidth=1.2, label="MPC command")
        ax.plot(d["t"], d[f"true_{key}"] * deg, color=COLOR_TRUE, label="true")
        ax.plot(d["t"], d[f"est_{key}"] * deg, color=COLOR_EST, label="EKF estimate")
        ax.set_ylabel(f"{label} [deg]")
        mark_stop(ax, d)
    axes[2].plot(d["t"], d["ref_yaw"] * deg, color=COLOR_REF, linestyle="--", linewidth=1.2, label="reference")
    axes[2].plot(d["t"], d["true_yaw"] * deg, color=COLOR_TRUE, label="true")
    axes[2].plot(d["t"], d["est_yaw"] * deg, color=COLOR_EST, label="EKF estimate")
    axes[2].set_ylabel("yaw [deg]")
    mark_stop(axes[2], d)
    legend_above(axes[0])
    legend_above(axes[2])
    fig.suptitle(f"Attitude - {name}", x=0.08, ha="left")
    axes[-1].set_xlabel("time [s]")
    save(fig, f"sim_{name}_attitude")


def plot_thrust(d, name):
    fig, ax = plt.subplots(figsize=(8, 2.8))
    ax.plot(d["t"], d["cmd_thrust_cmd"] / 1000.0, color=COLOR_CMD)
    ax.set_ylabel("thrust cmd [1/1000]")
    ax.set_xlabel("time [s]")
    ax.set_title(f"Thrust command (per motor) - {name}", loc="left")
    mark_stop(ax, d)
    save(fig, f"sim_{name}_thrust")


def main(paths):
    setup_style()
    print(f"{'scenario':<14}{'track xy':>9}{'track z':>9}{'est xy':>8}{'est z':>8}"
          f"{'est v':>8}{'est att':>9}{'tilt':>7}{'solve':>8}{'stop':>7}")
    print(f"{'':<14}{'[m]':>9}{'[m]':>9}{'[m]':>8}{'[m]':>8}{'[m/s]':>8}{'[rad]':>9}{'[rad]':>7}{'[ms]':>8}{'[s]':>7}")
    for path in paths:
        name = Path(path).stem
        d = load_log(path)
        plot_position(d, name)
        plot_attitude(d, name)
        plot_thrust(d, name)
        m = compute_metrics(d)
        print(f"{name:<14}{m['tracking_rms_xy_m']:9.3f}{m['tracking_rms_z_m']:9.3f}"
              f"{m['estimation_rms_xy_m']:8.3f}{m['estimation_rms_z_m']:8.3f}"
              f"{m['estimation_rms_v_mps']:8.3f}{m['estimation_rms_roll_pitch_rad']:9.4f}"
              f"{m['tilt_rms_rad']:7.3f}{m['max_solve_time_ms']:8.2f}{m['stop_time_s']:7.2f}")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    main(sys.argv[1:])
