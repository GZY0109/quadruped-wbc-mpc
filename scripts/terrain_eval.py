"""Phase 3(B): terrain sensitivity -- flat vs slope vs rough, standing balance.

Scope, per project decision (see PROGRESS.md): the three gait scenarios
(walk-in-place/trot/walk-forward) already saturate near 100% fall_rate under
flat-ground joint-noise disturbance (scripts/disturbance_sweep.py), so
stacking terrain on top of them adds no information. This instead tests
pure standing balance (gait_name="stand") -- the one scenario where the
controller has real stability margin -- across three floor geometries, to
isolate the cost of the flat-ground assumption baked into src/mpc.py's and
src/wbc.py's friction-cone/foothold math (neither file is touched here).

Disturbance amplitude (0.15 rad) was calibrated specifically for the stand
scenario (the gait-scenario sweep's amplitudes don't transfer -- standing
is far more robust than walking and was still 0% fall_rate at those
amplitudes): a quick 5-point scan on FLAT ground found amp=0.15 gives the
best split for a baseline (flat QP 80% fall_rate, hierarchical QP 40%),
leaving room for slope/rough to move the needle either direction instead of
being pinned at 0% or 100% before terrain is even a factor.

Writes results/terrain_eval.json (raw per-trial data), results/terrain_eval.png
(fall-rate + survival-time bar chart per terrain x controller), and one
representative video per terrain (hierarchical QP, seed closest to that
config's mean survival time).

Run: python scripts/terrain_eval.py
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

from scripts.trot_demo import run_trot, run_trials

RESULTS_DIR = Path(__file__).resolve().parents[1] / "results"
ASSETS_DIR = Path(__file__).resolve().parents[1] / "assets" / "unitree_go2"
N_SEEDS = 5
SEEDS = list(range(N_SEEDS))
AMPLITUDE = 0.15  # calibrated for gait_name="stand" specifically, see module docstring
SECS = 5.0

# (terrain_key, run_trot kwargs -- scene_path/base_pitch/z_offset calibrated
# in the previous commit's spawn-pose testing)
TERRAINS = [
    ("flat", dict(scene_path=None, base_pitch=0.0, z_offset=0.0)),
    ("slope 5deg", dict(scene_path=str(ASSETS_DIR / "go2_scene_slope.xml"),
                         base_pitch=np.radians(5.0), z_offset=0.0)),
    ("rough", dict(scene_path=str(ASSETS_DIR / "go2_scene_rough.xml"),
                    base_pitch=0.0, z_offset=0.02)),
]


def main():
    data = {"n_seeds": N_SEEDS, "seeds": SEEDS, "amplitude": AMPLITUDE, "secs": SECS,
            "terrains": {}}

    for terrain_key, scene_kwargs in TERRAINS:
        print(f"=== terrain: {terrain_key} ===")
        data["terrains"][terrain_key] = {}
        for label, hier in [("flat QP", False), ("hierarchical QP", True)]:
            summary = run_trials(
                n=N_SEEDS, seeds=SEEDS, secs=SECS, vx=0.0, gait_name="stand",
                hierarchical=hier, init_noise_amplitude=AMPLITUDE, **scene_kwargs,
            )
            data["terrains"][terrain_key][label] = summary
            print(f"  {label}: {summary['sim_time_mean']:.2f}+-{summary['sim_time_std']:.2f}s "
                  f"fall_rate={summary['fall_rate']:.0%}")

    RESULTS_DIR.mkdir(exist_ok=True)
    (RESULTS_DIR / "terrain_eval.json").write_text(json.dumps(data, indent=2))
    print(f"\nwrote {RESULTS_DIR / 'terrain_eval.json'}")

    # --- plot: fall-rate and survival-time bars, terrain x controller ---
    terrain_keys = [k for k, _ in TERRAINS]
    fig, (ax_t, ax_f) = plt.subplots(1, 2, figsize=(11, 4.8))
    x = np.arange(len(terrain_keys))
    width = 0.35
    colors = {"flat QP": "#999999", "hierarchical QP": "#4c72b0"}
    for i, label in enumerate(("flat QP", "hierarchical QP")):
        means = [data["terrains"][t][label]["sim_time_mean"] for t in terrain_keys]
        stds = [data["terrains"][t][label]["sim_time_std"] for t in terrain_keys]
        frs = [data["terrains"][t][label]["fall_rate"] * 100 for t in terrain_keys]
        offset = (i - 0.5) * width
        ax_t.bar(x + offset, means, width, yerr=stds, capsize=4, label=label,
                  color=colors[label], alpha=0.85)
        ax_f.bar(x + offset, frs, width, label=label, color=colors[label], alpha=0.85)
    ax_t.set_xticks(x); ax_t.set_xticklabels(terrain_keys)
    ax_t.set_ylabel("survival time (s)")
    ax_t.set_title("standing survival time by terrain")
    ax_t.legend(fontsize=9)
    ax_f.set_xticks(x); ax_f.set_xticklabels(terrain_keys)
    ax_f.set_ylabel("fall rate (%)")
    ax_f.set_ylim(0, 105)
    ax_f.set_title("standing fall rate by terrain")
    fig.suptitle(f"Terrain sensitivity -- standing balance, {N_SEEDS} seeds/config, "
                 f"init noise amplitude={AMPLITUDE}rad", fontsize=11)
    fig.tight_layout()
    out_png = RESULTS_DIR / "terrain_eval.png"
    fig.savefig(out_png, dpi=150)
    print(f"wrote {out_png}")

    # --- representative videos (hierarchical QP, seed closest to mean) ---
    for terrain_key, scene_kwargs in TERRAINS:
        summary = data["terrains"][terrain_key]["hierarchical QP"]
        times = np.array([run_trot(
            secs=SECS, vx=0.0, gait_name="stand", hierarchical=True,
            init_noise_seed=s, init_noise_amplitude=AMPLITUDE, verbose=False,
            video=False, **scene_kwargs,
        )[0]["sim_time"] for s in SEEDS])
        rep_seed = SEEDS[int(np.argmin(np.abs(times - summary["sim_time_mean"])))]
        print(f"rendering {terrain_key}: seed={rep_seed} "
              f"(survival={times[SEEDS.index(rep_seed)]:.2f}s, mean={summary['sim_time_mean']:.2f}s)")
        run_trot(secs=SECS, vx=0.0, gait_name="stand", hierarchical=True,
                  init_noise_seed=rep_seed, init_noise_amplitude=AMPLITUDE,
                  verbose=False, video=True, video_fps=50, **scene_kwargs)
        rendered = RESULTS_DIR / "trot_demo.mp4"
        target = RESULTS_DIR / f"terrain_{terrain_key.replace(' ', '_')}_representative.mp4"
        if rendered.exists():
            rendered.replace(target)
            print(f"wrote {target}")


if __name__ == "__main__":
    main()
