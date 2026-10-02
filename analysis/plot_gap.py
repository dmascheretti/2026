"""Sim-to-real gap figure: altitude with a 10 % heavier drone, with and
without the disturbance observer.

Usage: python3 analysis/plot_gap.py sim/output/gap_mass_no_observer.csv sim/output/gap_mass.csv
"""
import sys

import numpy as np

import plot_sim


def main(baseline_path, observer_path):
    plot_sim.setup_style()
    base = plot_sim.load_log(baseline_path)
    obs = plot_sim.load_log(observer_path)
    fig, ax = plot_sim.plt.subplots(figsize=(8, 3.6))
    ax.plot(base["t"], base["ref_pz"], color=plot_sim.COLOR_REF, linestyle="--", linewidth=1.2,
            label="reference")
    ax.plot(base["t"], base["true_pz"], color=plot_sim.COLOR_EST, label="linear MPC")
    ax.plot(obs["t"], obs["true_pz"], color=plot_sim.COLOR_TRUE, label="MPC + disturbance observer")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("true altitude z [m]")
    plot_sim.legend_above(ax)
    fig.suptitle("Drone 10 % heavier than the model", x=0.08, y=1.04, ha="left")
    plot_sim.save(fig, "gap_mass_altitude")
    for name, d in (("linear MPC", base), ("MPC + observer", obs)):
        hover = (d["t"] > 4) & (d["t"] < 8)
        print(f"{name:16s} mean altitude error 4-8 s: {np.mean(d['true_pz'][hover] - d['ref_pz'][hover]):+.4f} m")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
