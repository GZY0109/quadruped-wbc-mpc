# Phase 4 slope-angle sweep: the initial joint-noise sweep (sweep_eval.py) saturates at
# 0% fall rate for the RL policy even at absurd amplitudes (verified: the noise event
# really does perturb joints -- the policy's direct position-tracking action recovers
# within a control step, unlike the classical torque-QP controller). Slope angle is a
# continuously-applied disturbance instead of a one-time initial kick, so it's a more
# likely axis to find where this specific policy actually breaks -- and it's directly
# comparable to Phase 3(B)'s "5 deg already saturates both classical QPs" finding.
import argparse
import json
import os
import sys
import time

parser = argparse.ArgumentParser(description="Phase 4 RL policy slope-angle sweep")
parser.add_argument("--checkpoint", type=str, required=True)
parser.add_argument("--num_seeds", type=int, default=5)
parser.add_argument("--vx", type=float, default=0.3)
parser.add_argument(
    "--slope_deg",
    type=str,
    default="5,10,15,20,22.9",
    help="Comma-separated slope angles [deg]; 22.9deg = training curriculum's max (0.4 rad)",
)
parser.add_argument("--episode_secs", type=float, default=10.0)
parser.add_argument("--seed", type=int, default=1000)
parser.add_argument("--out", type=str, default="../../results/isaac_lab_rl_slope_sweep.json")
args_cli = parser.parse_args()

from isaaclab.app import AppLauncher  # noqa: E402

app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

import math  # noqa: E402
import importlib.metadata as metadata  # noqa: E402
import torch  # noqa: E402
from rsl_rl.runners import OnPolicyRunner  # noqa: E402

from isaaclab.envs import ManagerBasedRLEnv  # noqa: E402
from isaaclab_rl.rsl_rl import RslRlVecEnvWrapper, handle_deprecated_rsl_rl_cfg  # noqa: E402

import isaaclab_tasks  # noqa: E402,F401
from isaaclab_tasks.core.velocity.config.go2.agents.rsl_rl_ppo_cfg import UnitreeGo2RoughPPORunnerCfg  # noqa: E402
from isaaclab_tasks.utils.hydra import resolve_presets  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_cfgs import Phase4EvalSlopeCfg  # noqa: E402


def run_condition(env, policy, num_envs, max_steps, step_dt):
    # duplicated from sweep_eval.py: that module parses argv at import time, so importing
    # it here would clobber this script's own CLI args.
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
    slope_degs = [float(a) for a in args_cli.slope_deg.split(",")]
    agent_cfg = UnitreeGo2RoughPPORunnerCfg()
    agent_cfg = handle_deprecated_rsl_rl_cfg(agent_cfg, metadata.version("rsl-rl-lib"))

    trials = []
    t_wall_start = time.time()
    for deg in slope_degs:
        t0 = time.time()
        cfg = Phase4EvalSlopeCfg()
        cfg.scene.num_envs = args_cli.num_seeds
        cfg.episode_length_s = args_cli.episode_secs
        cfg.seed = args_cli.seed
        cfg.set_command(args_cli.vx)
        cfg.set_noise_amplitude(0.0)  # isolate slope as the only variable
        cfg.set_slope(math.radians(deg))
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
            "slope_deg": deg,
            "wall_s": round(time.time() - t0, 1),
        }
        trials.append(trial)
        print(f"[slope={deg}deg] -> {trial}", flush=True)
        env.close()

    out = {
        "n_seeds": args_cli.num_seeds,
        "seeds": list(range(args_cli.num_seeds)),
        "vx_cmd": args_cli.vx,
        "noise_amplitude": 0.0,
        "checkpoint": args_cli.checkpoint,
        "config_label": "RL policy (rough-trained PPO)",
        "slope_degs": slope_degs,
        "trials": trials,
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
