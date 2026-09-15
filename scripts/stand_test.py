"""Phase 1 smoke test: joint-space PD stand.

Verifies the whole sim <-> control loop end to end:
  * torques go down through Go2Sim.step()
  * state (base pose, joint pos/vel, foot contacts, IMU) comes back sane
  * a trivial PD controller holding the home pose keeps the robot standing

This is NOT a controller from the plan (Convex MPC / WBC come in Phase 2); it is
the minimal closed loop that proves the environment interface works. Run:

    python scripts/stand_test.py                # headless, prints a report
    python scripts/stand_test.py --video        # also writes results/stand_test.mp4
"""

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src.sim_env import Go2Sim  # noqa: E402

# Home standing pose target (per leg: hip, thigh, calf), matching the MJCF keyframe.
HOME_JOINT_TARGET = np.array([0.0, 0.9, -1.8] * 4)


def run(duration: float = 3.0, kp: float = 60.0, kd: float = 3.0, video: bool = False):
    sim = Go2Sim(control_dt=0.002)
    state = sim.reset(keyframe="home")

    n_steps = int(duration / sim.control_dt)
    frames = []
    log = {k: [] for k in ("t", "base_z", "roll_deg", "pitch_deg", "n_contact", "max_tau")}

    for i in range(n_steps):
        # Joint-space PD toward the home pose.
        tau = kp * (HOME_JOINT_TARGET - state.joint_pos) - kd * state.joint_vel
        state = sim.step(tau)

        log["t"].append(state.time)
        log["base_z"].append(state.base_pos[2])
        log["roll_deg"].append(np.degrees(state.base_rpy[0]))
        log["pitch_deg"].append(np.degrees(state.base_rpy[1]))
        log["n_contact"].append(int(state.foot_contact.sum()))
        log["max_tau"].append(float(np.abs(tau).max()))

        if video and i % 10 == 0:  # ~50 fps at control_dt=0.002
            frames.append(sim.render(camera=-1))

    log = {k: np.asarray(v) for k, v in log.items()}

    # ---- settle window: last 1 s, when transients have died out ----
    settle = log["t"] >= (log["t"][-1] - 1.0)
    report = {
        "final_base_z": log["base_z"][-1],
        "mean_base_z_settled": log["base_z"][settle].mean(),
        "max_abs_roll_deg": np.abs(log["roll_deg"]).max(),
        "max_abs_pitch_deg": np.abs(log["pitch_deg"]).max(),
        "mean_contacts_settled": log["n_contact"][settle].mean(),
        "peak_torque_Nm": log["max_tau"].max(),
    }

    print("\n=== Phase 1 stand smoke test ===")
    print(f"model dt={sim.dt*1000:.1f} ms  control_dt={sim.control_dt*1000:.1f} ms  "
          f"substeps={sim.n_substeps}  duration={duration:.1f} s")
    print(f"initial base height : {log['base_z'][0]*100:6.2f} cm")
    print(f"final base height   : {report['final_base_z']*100:6.2f} cm")
    print(f"settled height (1s) : {report['mean_base_z_settled']*100:6.2f} cm")
    print(f"max |roll|          : {report['max_abs_roll_deg']:6.2f} deg")
    print(f"max |pitch|         : {report['max_abs_pitch_deg']:6.2f} deg")
    print(f"feet in contact     : {report['mean_contacts_settled']:.2f} / 4 (settled avg)")
    print(f"peak |torque|       : {report['peak_torque_Nm']:6.2f} Nm")

    # Pass criteria: robot stayed up, roughly level, all four feet planted.
    ok = (
        report["mean_base_z_settled"] > 0.20
        and report["max_abs_roll_deg"] < 10.0
        and report["max_abs_pitch_deg"] < 10.0
        and report["mean_contacts_settled"] > 3.5
    )
    print(f"\nRESULT: {'PASS  robot stands stably' if ok else 'FAIL  see values above'}")

    if video and frames:
        import imageio

        out = Path(__file__).resolve().parents[1] / "results" / "stand_test.mp4"
        out.parent.mkdir(exist_ok=True)
        imageio.mimsave(out, frames, fps=50)
        print(f"video written: {out}")

    sim.close()
    return 0 if ok else 1


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--duration", type=float, default=3.0)
    p.add_argument("--video", action="store_true")
    args = p.parse_args()
    sys.exit(run(duration=args.duration, video=args.video))
