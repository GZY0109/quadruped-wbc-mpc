"""Multi-trial survival-time comparison: flat vs hierarchical WBC QP.

Single deterministic runs (init_noise_seed=None) turned out not to be a
reliable basis for judging this controller -- it operates in a marginally
stable regime where floating-point differences between library point
releases shifted survival times by seconds (see PROGRESS.md Phase 2(4)).
This script instead runs each config across N seeded initial-noise trials
and reports the full distribution, not a single point estimate.

Writes results/benchmark_summary.json (raw per-trial numbers) and
results/hierarchical_vs_flat.png (bar chart, mean+-std with individual
trial dots), plus a representative (closest-to-mean) video for the
hierarchical walk-in-place config so the demo clip isn't a cherry-picked
lucky run.

Run: python scripts/benchmark_summary.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from scripts.trot_demo import run_trot

RESULTS_DIR = Path(__file__).resolve().parents[1] / "results"
N_TRIALS = 5
SEEDS = list(range(N_TRIALS))

CONFIGS = [
    ("flat QP\nwalk in-place", dict(secs=10.0, vx=0.0, gait_name="walk",
     gait_period=0.8, step_height=0.05, hierarchical=False)),
    ("hierarchical QP\nwalk in-place", dict(secs=10.0, vx=0.0, gait_name="walk",
     gait_period=0.8, step_height=0.05, hierarchical=True)),
    ("hierarchical QP\ntrot", dict(secs=5.0, vx=0.0, gait_name="trot",
     gait_period=0.4, step_height=0.08, hierarchical=True)),
    ("hierarchical QP\nwalk vx=0.3", dict(secs=5.0, vx=0.3, gait_name="walk",
     gait_period=0.8, step_height=0.05, hierarchical=True)),
]


def survival_times(seeds, **kwargs) -> list[float]:
    times = []
    for seed in seeds:
        result, _ = run_trot(init_noise_seed=seed, verbose=False, video=False, **kwargs)
        times.append(result["sim_time"])
    return times


def main():
    data = {}
    for label, kwargs in CONFIGS:
        flat_label = label.replace("\n", " ")
        print(f"=== {flat_label} ===")
        times = survival_times(SEEDS, **kwargs)
        data[label] = times
        print(f"  {np.mean(times):.2f}+-{np.std(times):.2f}s  "
              f"trials={[round(t, 2) for t in times]}")

    RESULTS_DIR.mkdir(exist_ok=True)
    (RESULTS_DIR / "benchmark_summary.json").write_text(json.dumps(
        {"n_trials": N_TRIALS, "seeds": SEEDS,
         "survival_s": {k: v for k, v in data.items()}}, indent=2))

    labels = list(data.keys())
    means = [float(np.mean(data[l])) for l in labels]
    stds = [float(np.std(data[l])) for l in labels]
    colors = ["#999999", "#4c72b0", "#dd8452", "#55a868"]

    fig, ax = plt.subplots(figsize=(7.5, 4.8))
    x = np.arange(len(labels))
    ax.bar(x, means, yerr=stds, capsize=5, color=colors, alpha=0.85)
    rng = np.random.default_rng(0)
    for i, l in enumerate(labels):
        jitter = rng.uniform(-0.08, 0.08, size=len(data[l]))
        ax.scatter(np.full(len(data[l]), i) + jitter, data[l], color="black", s=20, zorder=3)
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("survival time (s)")
    ax.set_title(f"Hierarchical vs flat WBC QP -- {N_TRIALS} seeded init-noise trials each\n"
                 f"(100% fall rate in every group)", fontsize=11)
    fig.tight_layout()
    out_png = RESULTS_DIR / "hierarchical_vs_flat.png"
    fig.savefig(out_png, dpi=150)
    print(f"wrote {out_png}")

    # representative (closest-to-mean) video for hierarchical walk-in-place,
    # so the clip reflects the typical outcome, not a lucky zero-noise run.
    hwalk_label, hwalk_kwargs = CONFIGS[1]
    hwalk_times = data[hwalk_label]
    rep_seed = SEEDS[int(np.argmin(np.abs(np.array(hwalk_times) - np.mean(hwalk_times))))]
    print(f"rendering representative video: seed={rep_seed} "
          f"(survival={hwalk_times[SEEDS.index(rep_seed)]:.2f}s, "
          f"mean={np.mean(hwalk_times):.2f}s)")
    _, _ = run_trot(init_noise_seed=rep_seed, verbose=True, video=True,
                     video_fps=50, **hwalk_kwargs)
    rendered = RESULTS_DIR / "trot_demo.mp4"
    target = RESULTS_DIR / "walk_hierarchical_representative.mp4"
    if rendered.exists():
        rendered.replace(target)
        print(f"wrote {target}")


if __name__ == "__main__":
    main()
