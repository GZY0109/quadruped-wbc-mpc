"""Closed-loop trot demo: gait scheduler + Convex MPC + WBC on flat ground.

Phase 2 step 4 (see PROGRESS.md), the integration milestone. Wires the three
layers into one control loop:

  gait (src.gait)  -> stance/swing schedule + swing-foot references
  MPC  (src.mpc)   -> ground reaction forces at ~40 Hz (SRBD horizon QP)
  WBC  (src.wbc)   -> 12 joint torques at 500 Hz (whole-body QP)

The MPC runs at a lower rate (its forces are held between updates); the WBC and
physics run every 2 ms. Foothold targets come from the Raibert heuristic, and
each swing foot follows a cycloid arc from its lift-off point to that target.

Run:  python scripts/trot_demo.py [--secs 5] [--vx 0.4] [--video]
Prints a metrics summary and (with --video) writes results/trot_demo.mp4.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

# allow running as a script
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from src.sim_env import Go2Sim, LEGS, quat_to_rotmat
from src.gait import GaitScheduler, raibert_foothold, swing_foot_reference
from src.mpc import ConvexMPC, make_reference, composite_inertia_from_model, GRAVITY
from src.wbc import WholeBodyController


def run_trot(
    secs: float = 5.0,
    vx: float = 0.4,
    vy: float = 0.0,
    yaw_rate: float = 0.0,
    gait_period: float = 0.4,
    step_height: float = 0.08,
    mpc_dt: float = 0.03,
    mpc_horizon: int = 10,
    ramp: float = 1.0,
    stand_time: float = 0.5,
    wbc_gains=None,
    video: bool = False,
    video_fps: int = 50,
    verbose: bool = True,
):
    sim = Go2Sim(control_dt=0.002)
    st = sim.reset()
    dt = sim.control_dt
    walk_height = float(st.base_pos[2])          # hold the home trunk height

    mass, com0, inertia = composite_inertia_from_model(sim.model, sim.data)

    gait = GaitScheduler("trot", period=gait_period)
    mpc = ConvexMPC(mass, inertia, dt=mpc_dt, horizon=mpc_horizon, mu=0.6)
    wbc = WholeBodyController(
        sim.model, sim.foot_site_ids, sim.joint_qvel_adr, sim.torque_limits, mu=0.6,
        gains=wbc_gains,
    )

    hip_body_ids = np.array([sim.model.body(f"{leg}_hip").id for leg in LEGS])
    # Nominal foot offset from the base origin (home pose), in the base frame.
    # Foothold planning references this WIDE default foot line (feet at y=+-0.14)
    # rather than the hip-joint location (y=+-0.046) -- using the hips would land
    # feet far inboard and give a narrow, roll-unstable support.
    nominal_offset = (st.foot_pos - st.base_pos).copy()
    nominal_offset[:, 2] = 0.0

    # per-leg swing bookkeeping
    prev_contact = np.ones(4, dtype=bool)
    swing_start = st.foot_pos.copy()
    foothold = st.foot_pos.copy()

    mpc_forces = np.zeros((4, 3))
    mpc_forces[:, 2] = mass * GRAVITY / 4.0
    mpc_every = max(1, round(mpc_dt / dt))

    # logs
    T = int(secs / dt)
    log = dict(t=[], h=[], roll=[], pitch=[], yaw=[], vx=[], vy=[],
               n_contact=[], mpc_solve=[], wbc_solve=[], wbc_fail=0)
    frames = []
    frame_every = max(1, round((1.0 / video_fps) / dt))

    for k in range(T):
        t = k * dt
        # brief 4-foot stand warmup before the gait clock starts, so the robot
        # settles under WBC instead of lurching into a swing from rest.
        walking = t >= stand_time
        tg = t - stand_time                          # gait-clock time
        # commanded velocity with a smooth ramp-in (starts when walking)
        s = min(1.0, tg / ramp) if (walking and ramp > 0) else (1.0 if walking else 0.0)
        vel_cmd = np.array([vx * s, vy * s, 0.0])
        yr_cmd = yaw_rate * s

        if walking:
            gs = gait.eval(tg)
            contact = gs.contact
            swing_phase = gs.swing_phase
        else:
            contact = np.ones(4, dtype=bool)         # stand: all feet down
            swing_phase = np.zeros(4)

        # --- foothold planning on stance->swing transitions ---
        for i in range(4):
            if prev_contact[i] and not contact[i]:      # just lifted off
                swing_start[i] = st.foot_pos[i].copy()
                # reference point = base + yaw-rotated nominal (wide) foot offset
                cy, sy = np.cos(st.base_rpy[2]), np.sin(st.base_rpy[2])
                off = nominal_offset[i]
                ref_xy = st.base_pos[:2] + np.array([cy * off[0] - sy * off[1],
                                                     sy * off[0] + cy * off[1]])
                ref_pt = np.array([ref_xy[0], ref_xy[1], 0.0])
                foothold[i] = raibert_foothold(
                    ref_pt, st.base_lin_vel, vel_cmd, gait.stance_duration,
                    walk_height, yaw_rate=st.base_ang_vel[2], yaw_rate_cmd=yr_cmd,
                )
        prev_contact = contact.copy()

        # --- MPC at its lower rate ---
        if k % mpc_every == 0:
            x0 = np.zeros(13)
            x0[0:3] = st.base_rpy
            x0[3:6] = st.base_pos
            x0[6:9] = st.base_ang_vel
            x0[9:12] = st.base_lin_vel
            x0[12] = -GRAVITY
            if walking:
                contact_seq = gait.contact_sequence(tg, mpc_horizon, mpc_dt)
            else:
                contact_seq = np.ones((mpc_horizon, 4), dtype=bool)
            x_ref = make_reference(x0, vel_cmd, yr_cmd, walk_height, mpc_horizon, mpc_dt)
            mpc_forces, mpc_info = mpc.solve(
                x0, st.foot_pos, st.base_pos, contact_seq, x_ref
            )
            log["mpc_solve"].append(mpc_info["solve_time_ms"])

        # --- swing-foot references ---
        swing_pos = st.foot_pos.copy()
        swing_vel = np.zeros((4, 3))
        swing_acc = np.zeros((4, 3))
        for i in range(4):
            if not contact[i]:
                p, v, a = swing_foot_reference(
                    swing_phase[i], swing_start[i], foothold[i],
                    step_height, gait.swing_duration,
                )
                swing_pos[i], swing_vel[i], swing_acc[i] = p, v, a

        # --- WBC -> torques ---
        base_pos_des = np.array([st.base_pos[0], st.base_pos[1], walk_height])
        yaw_des = st.base_rpy[2]     # let yaw follow (no yaw command here)
        tau, wbc_info = wbc.solve(
            sim.data, mpc_forces, contact,
            base_pos_des=base_pos_des, base_vel_des=vel_cmd,
            base_rpy_des=np.array([0.0, 0.0, yaw_des]),
            base_omega_des=np.array([0.0, 0.0, yr_cmd]),
            swing_pos_ref=swing_pos, swing_vel_ref=swing_vel, swing_acc_ref=swing_acc,
        )
        if wbc_info["status_val"] not in (1, 2):
            log["wbc_fail"] += 1
        log["wbc_solve"].append(wbc_info["solve_time_ms"])

        st = sim.step(tau)

        # --- logging ---
        log["t"].append(t)
        log["h"].append(st.base_pos[2])
        log["roll"].append(np.degrees(st.base_rpy[0]))
        log["pitch"].append(np.degrees(st.base_rpy[1]))
        log["yaw"].append(np.degrees(st.base_rpy[2]))
        log["vx"].append(st.base_lin_vel[0])
        log["vy"].append(st.base_lin_vel[1])
        log["n_contact"].append(int(contact.sum()))

        if video and k % frame_every == 0:
            frames.append(sim.render(width=640, height=480))

        # early-abort if the robot clearly fell
        if st.base_pos[2] < 0.12 or abs(st.base_rpy[0]) > np.radians(60) \
                or abs(st.base_rpy[1]) > np.radians(60):
            if verbose:
                print(f"  !! fell at t={t:.2f}s (h={st.base_pos[2]*100:.1f}cm, "
                      f"roll={np.degrees(st.base_rpy[0]):.0f}, "
                      f"pitch={np.degrees(st.base_rpy[1]):.0f})")
            break

    # ---- summary ----
    dist = st.base_pos[0] - com0[0]
    walked = st.time
    steady = np.array(log["t"]) > ramp
    result = {
        "reached_end": k == T - 1,
        "sim_time": walked,
        "x_distance": float(st.base_pos[0]),
        "final_h_cm": float(st.base_pos[2] * 100),
        "mean_vx": float(np.mean(np.array(log["vx"])[steady])) if steady.any() else 0.0,
        "max_roll": float(np.max(np.abs(log["roll"]))),
        "max_pitch": float(np.max(np.abs(log["pitch"]))),
        "rms_roll": float(np.sqrt(np.mean(np.square(log["roll"])))),
        "rms_pitch": float(np.sqrt(np.mean(np.square(log["pitch"])))),
        "wbc_fail": log["wbc_fail"],
        "mpc_ms": float(np.mean(log["mpc_solve"])) if log["mpc_solve"] else 0.0,
        "wbc_ms": float(np.mean(log["wbc_solve"])) if log["wbc_solve"] else 0.0,
    }

    if verbose:
        print(f"  reached_end={result['reached_end']}  sim_time={result['sim_time']:.2f}s  "
              f"x_dist={result['x_distance']:.2f}m  final_h={result['final_h_cm']:.1f}cm")
        print(f"  mean_vx(steady)={result['mean_vx']:.3f} m/s (cmd {vx})  "
              f"max|roll|={result['max_roll']:.1f} max|pitch|={result['max_pitch']:.1f} deg  "
              f"rms(roll,pitch)=({result['rms_roll']:.2f},{result['rms_pitch']:.2f})")
        print(f"  solve times: MPC {result['mpc_ms']:.2f}ms  WBC {result['wbc_ms']:.2f}ms  "
              f"wbc_fail={result['wbc_fail']}")

    if video and frames:
        import imageio
        out = Path(__file__).resolve().parents[1] / "results" / "trot_demo.mp4"
        out.parent.mkdir(exist_ok=True)
        imageio.mimsave(out, frames, fps=video_fps)
        print(f"  wrote video -> {out} ({len(frames)} frames)")

    return result, log


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=5.0)
    ap.add_argument("--vx", type=float, default=0.4)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--yaw_rate", type=float, default=0.0)
    ap.add_argument("--period", type=float, default=0.4)
    ap.add_argument("--step_height", type=float, default=0.08)
    ap.add_argument("--video", action="store_true")
    args = ap.parse_args()
    print(f"=== trot demo: vx={args.vx} period={args.period} step_h={args.step_height} ===")
    run_trot(secs=args.secs, vx=args.vx, vy=args.vy, yaw_rate=args.yaw_rate,
             gait_period=args.period, step_height=args.step_height, video=args.video)


if __name__ == "__main__":
    main()
