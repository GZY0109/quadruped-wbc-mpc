"""Phase 3(A): disturbance-amplitude sweep -- flat vs hierarchical WBC QP.

The single fixed-amplitude benchmark (scripts/benchmark_summary.py,
+-0.05rad) saturates: every gait scenario is 100% fall_rate at that
amplitude, so there is no room left to compare the two controllers. This
script instead sweeps the init-noise amplitude and traces out
survival-time/fall-rate curves, which is where the "hierarchical QP is more
robust" claim (established directionally in PROGRESS.md at the single
+-0.05rad point) actually gets a quantitative, defensible number: the
amplitude at which each controller crosses 50% fall_rate.

Flagship sweep: walk-in-place (the most thoroughly diagnosed scenario) at a
fine 8-point amplitude grid, both controllers, n=5 seeds each.
Secondary sweep: trot and forward-walk (vx=0.3) at a coarser 4-point grid,
to check whether the flat-vs-hierarchical ranking generalizes to other
gaits, not to characterize them as finely.

Writes results/disturbance_sweep.json (raw per-trial data) and
results/disturbance_sweep.png (3-panel survival-time + fall-rate curves),
plus a printed table of each config's 50%-fall_rate crossing amplitude
(linear interpolation between the two bracketing grid points; None if the
grid never brackets 50%).

Run: python scripts/disturbance_sweep.py
"""

from __future__ import annotations

import json
import time
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.trot_demo import run_trials

RESULTS_DIR = Path(__file__).resolve().parents[1] / "results"
N_SEEDS = 5
SEEDS = list(range(N_SEEDS))

FLAGSHIP_AMPLITUDES = [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08]
SECONDARY_AMPLITUDES = [0.0, 0.02, 0.04, 0.05]

# (scenario_key, run_trot kwargs excluding hierarchical/init_noise_amplitude)
FLAGSHIP_SCENARIO = ("walk in-place", dict(
    secs=10.0, vx=0.0, gait_name="walk", gait_period=0.8, step_height=0.05,
))
SECONDARY_SCENARIOS = [
    ("trot", dict(secs=5.0, vx=0.0, gait_name="trot", gait_period=0.4, step_height=0.08)),
    ("walk vx=0.3", dict(secs=5.0, vx=0.3, gait_name="walk", gait_period=0.8, step_height=0.05)),
]


def crossing_amplitude(amplitudes, fall_rates):
    """Linear-interpolate the amplitude where fall_rate first crosses 0.5.

    Returns None if the grid never brackets 0.5 (e.g. always 100% or always
    below 50%) -- do not extrapolate past the tested range.
    """
    amplitudes = np.asarray(amplitudes, dtype=float)
    fall_rates = np.asarray(fall_rates, dtype=float)
    for i in range(len(amplitudes) - 1):
        f0, f1 = fall_rates[i], fall_rates[i + 1]
        if (f0 - 0.5) * (f1 - 0.5) <= 0 and f0 != f1:
            a0, a1 = amplitudes[i], amplitudes[i + 1]
            frac = (0.5 - f0) / (f1 - f0)
            return float(a0 + frac * (a1 - a0))
    return None


def sweep(scenario_kwargs, amplitudes, hierarchical) -> dict:
    """Run one controller across an amplitude grid; return per-amplitude summaries."""
    out = {"amplitudes": amplitudes, "trials": []}
    for amp in amplitudes:
        t0 = time.time()
        summary = run_trials(
            n=N_SEEDS, seeds=SEEDS, init_noise_amplitude=amp,
            hierarchical=hierarchical, **scenario_kwargs,
        )
        summary["amplitude"] = amp
        summary["wall_s"] = round(time.time() - t0, 1)
        out["trials"].append(summary)
        print(f"    amp={amp:.2f}  sim_time={summary['sim_time_mean']:.2f}"
              f"+-{summary['sim_time_std']:.2f}s  fall_rate={summary['fall_rate']:.0%}"
              f"  ({summary['wall_s']}s wall)")
    return out


def main():
    t_start = time.time()
    data = {"n_seeds": N_SEEDS, "seeds": SEEDS, "scenarios": {}}

    print(f"=== flagship: {FLAGSHIP_SCENARIO[0]} ({len(FLAGSHIP_AMPLITUDES)} amplitudes x 2 QP x "
          f"{N_SEEDS} seeds = {len(FLAGSHIP_AMPLITUDES) * 2 * N_SEEDS} runs) ===")
    key, kwargs = FLAGSHIP_SCENARIO
    data["scenarios"][key] = {}
    for label, hier in [("flat QP", False), ("hierarchical QP", True)]:
        print(f"  -- {label} --")
        data["scenarios"][key][label] = sweep(kwargs, FLAGSHIP_AMPLITUDES, hier)

    for key, kwargs in SECONDARY_SCENARIOS:
        print(f"=== secondary: {key} ({len(SECONDARY_AMPLITUDES)} amplitudes x 2 QP x "
              f"{N_SEEDS} seeds = {len(SECONDARY_AMPLITUDES) * 2 * N_SEEDS} runs) ===")
        data["scenarios"][key] = {}
        for label, hier in [("flat QP", False), ("hierarchical QP", True)]:
            print(f"  -- {label} --")
            data["scenarios"][key][label] = sweep(kwargs, SECONDARY_AMPLITUDES, hier)

    total_wall = time.time() - t_start
    print(f"\n=== total wall-clock: {total_wall/60:.1f} min ===")

    # crossing amplitudes
    crossings = {}
    for scen, controllers in data["scenarios"].items():
        crossings[scen] = {}
        for label, sw in controllers.items():
            amps = sw["amplitudes"]
            frs = [t["fall_rate"] for t in sw["trials"]]
            crossings[scen][label] = crossing_amplitude(amps, frs)
    data["crossing_amplitude_50pct"] = crossings
    data["total_wall_s"] = round(total_wall, 1)

    print("\n=== 50%-fall_rate crossing amplitude [rad] ===")
    for scen, controllers in crossings.items():
        print(f"  {scen}:")
        for label, amp in controllers.items():
            print(f"    {label}: {amp:.3f}" if amp is not None else f"    {label}: (not bracketed by grid)")

    RESULTS_DIR.mkdir(exist_ok=True)
    (RESULTS_DIR / "disturbance_sweep.json").write_text(json.dumps(data, indent=2))
    print(f"\nwrote {RESULTS_DIR / 'disturbance_sweep.json'}")

    # --- plot: flagship gets its own big panel, secondary scenarios small panels ---
    scen_keys = [FLAGSHIP_SCENARIO[0]] + [k for k, _ in SECONDARY_SCENARIOS]
    fig, axes = plt.subplots(2, len(scen_keys), figsize=(5.2 * len(scen_keys), 8.5))
    colors = {"flat QP": "#999999", "hierarchical QP": "#4c72b0"}

    for col, scen in enumerate(scen_keys):
        ax_t, ax_f = axes[0, col], axes[1, col]
        for label in ("flat QP", "hierarchical QP"):
            sw = data["scenarios"][scen][label]
            amps = sw["amplitudes"]
            means = [t["sim_time_mean"] for t in sw["trials"]]
            stds = [t["sim_time_std"] for t in sw["trials"]]
            frs = [t["fall_rate"] * 100 for t in sw["trials"]]
            ax_t.errorbar(amps, means, yerr=stds, marker="o", label=label,
                          color=colors[label], capsize=3)
            ax_f.plot(amps, frs, marker="o", label=label, color=colors[label])
        ax_t.set_title(scen)
        ax_t.set_ylabel("survival time (s)" if col == 0 else "")
        ax_f.set_xlabel("init joint-noise amplitude (rad)")
        ax_f.set_ylabel("fall rate (%)" if col == 0 else "")
        ax_f.axhline(50, color="k", linestyle=":", linewidth=1, alpha=0.6)
        ax_t.grid(alpha=0.3)
        ax_f.grid(alpha=0.3)
        if col == 0:
            ax_t.legend(fontsize=9)
    fig.suptitle(f"Disturbance-amplitude sweep -- flat vs hierarchical WBC QP "
                 f"({N_SEEDS} seeds/point)", fontsize=12)
    fig.tight_layout()
    out_png = RESULTS_DIR / "disturbance_sweep.png"
    fig.savefig(out_png, dpi=150)
    print(f"wrote {out_png}")


if __name__ == "__main__":
    main()
