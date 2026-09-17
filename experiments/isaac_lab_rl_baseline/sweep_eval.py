# Phase 4 evaluation: replay a trained Go2 rough-locomotion PPO checkpoint under the
# same protocol as Phase 3 (results/disturbance_sweep.py, results/terrain_eval.py) --
# initial joint-noise amplitude sweep x {flat, slope_5deg, rough} terrain, N parallel
# seeds per condition, survival time + fall rate recorded. Output schema mirrors
# results/disturbance_sweep.json / results/terrain_eval.json (see PROGRESS.md "Phase 4
# 接口约定") so the two controllers' numbers can be plotted on the same axes.
#
# Must run inside the IsaacLab uv-managed env:
#   cd IsaacLab && OMNI_KIT_ACCEPT_EULA=YES uv run python \
#       ../experiments/isaac_lab_rl_baseline/sweep_eval.py --checkpoint <path/to/model_*.pt>
import argparse
import json
import os
import sys
import time

parser = argparse.ArgumentParser(description="Phase 4 RL policy noise/terrain sweep")
parser.add_argument("--checkpoint", type=str, required=True, help="Path to a rsl_rl model_*.pt checkpoint")
parser.add_argument("--num_seeds", type=int, default=5)
parser.add_argument("--vx", type=float, default=0.3, help="Fixed forward velocity command [m/s]")
parser.add_argument(
    "--amplitudes",
    type=str,
    default="0.0,0.01,0.02,0.03,0.04,0.05,0.06,0.08",
    help="Comma-separated initial joint-noise half-widths [rad], matching Phase 3's grid",
)
parser.add_argument("--episode_secs", type=float, default=10.0)
parser.add_argument("--seed", type=int, default=1000)
parser.add_argument("--out", type=str, default="../../results/isaac_lab_rl_eval.json")
args_cli = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import importlib.metadata as metadata  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402

import isaaclab_tasks  # noqa: E402,F401 -- side effect: registers task/agent cfg entry points
from isaaclab_tasks.core.velocity.config.go2.agents.rsl_rl_ppo_cfg import UnitreeGo2RoughPPORunnerCfg  # noqa: E402
from isaaclab_tasks.utils.hydra import resolve_presets  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_cfgs import TERRAIN_CFGS  # noqa: E402


def run_condition(env, policy, num_envs, max_steps, step_dt):
    device = env.unwrapped.device
    recorded = torch.zeros(num_envs, dtype=torch.bool, device=device)
    survival_steps = torch.zeros(num_envs, dtype=torch.long, device=device)
    fell = torch.zeros(num_envs, dtype=torch.bool, device=device)
    obs = env.get_observations()
    for t in range(max_steps):
        with torch.inference_mode():
            actions = policy(obs)
            obs, _, dones, extras = env.step(actions)
        dones = dones.bool()
        policy.reset(dones)
        newly_done = dones & (~recorded)
        if newly_done.any():
            time_outs = extras.get("time_outs", torch.zeros_like(dones))
            survival_steps[newly_done] = t + 1
            fell[newly_done] = ~time_outs[newly_done]
            recorded |= newly_done
        if bool(recorded.all()):
            break
    still_running = ~recorded
    survival_steps[still_running] = max_steps
    fell[still_running] = False
    return (survival_steps.to(torch.float32) * step_dt), fell


def main():
    amplitudes = [float(a) for a in args_cli.amplitudes.split(",")]
    agent_cfg = UnitreeGo2RoughPPORunnerCfg()
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))

    results = {}
    t_wall_start = time.time()
    for terrain_name, cfg_cls in TERRAIN_CFGS.items():
        trials = []
        for amp in amplitudes:
            t0 = time.time()
            cfg = cfg_cls()
            cfg.scene.num_envs = args_cli.num_seeds
            cfg.episode_length_s = args_cli.episode_secs
            cfg.seed = args_cli.seed
            cfg.set_command(args_cli.vx)
            cfg.set_noise_amplitude(amp)
            cfg = resolve_presets(cfg)

            base_env = ManagerBasedRLEnv(cfg=cfg)
            env = RslRlVecEnvWrapper(base_env, clip_actions=agent_cfg.clip_actions)
            runner = OnPolicyRunner(env, agent_cfg.to_dict(), log_dir=None, device=agent_cfg.device)
            runner.load(args_cli.checkpoint)
            policy = runner.get_inference_policy(device=env.unwrapped.device)

            torch.manual_seed(args_cli.seed)
            env.reset()
            max_steps = int(round(args_cli.episode_secs / env.unwrapped.step_dt))
            survival_s, fell = run_condition(env, policy, args_cli.num_seeds, max_steps, env.unwrapped.step_dt)

            trial = {
                "n": args_cli.num_seeds,
                "seeds": list(range(args_cli.num_seeds)),
                "sim_time_mean": survival_s.mean().item(),
                "sim_time_std": survival_s.std(unbiased=False).item(),
                "sim_time_min": survival_s.min().item(),
                "sim_time_max": survival_s.max().item(),
                "fall_rate": fell.float().mean().item(),
                "secs_requested": args_cli.episode_secs,
                "amplitude": amp,
                "wall_s": round(time.time() - t0, 1),
            }
            trials.append(trial)
            print(f"[{terrain_name}] amp={amp:.3f} -> {trial}", flush=True)
            env.close()

        results[terrain_name] = {"amplitudes": amplitudes, "trials": trials}

    out = {
        "n_seeds": args_cli.num_seeds,
        "seeds": list(range(args_cli.num_seeds)),
        "vx_cmd": args_cli.vx,
        "checkpoint": args_cli.checkpoint,
        "config_label": "RL policy (rough-trained PPO)",
        "terrains": results,
        "total_wall_s": round(time.time() - t_wall_start, 1),
    }
    out_path = os.path.abspath(os.path.join(os.path.dirname(__file__), args_cli.out))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(out, f, indent=1)
    print(f"[INFO] wrote {out_path}")

    simulation_app.close()


if __name__ == "__main__":
    main()
