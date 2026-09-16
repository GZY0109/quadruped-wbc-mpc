"""Closed-loop trot demo: gait scheduler + Convex MPC + WBC on flat ground.

Phase 2 step 4 (see PROGRESS.md), the integration milestone. Wires the three
layers into one control loop:

  gait (src.gait)  -> stance/swing schedule + swing-foot references
  MPC  (src.mpc)   -> ground reaction forces at ~40 Hz (SRBD horizon QP)
  WBC  (src.wbc)   -> 12 joint torques at 500 Hz (whole-body QP)

The MPC runs at a lower rate (its forces are held between updates); the WBC and
physics run every 2 ms. Foothold targets come from the Raibert heuristic, and
each swing foot follows a cycloid arc from its lift-off point to that target.

Contact-transition smoothing
-----------------------------
Debugging showed that walking instability consistently occurred at the exact
same event -- a leg's stance/swing state flipping -- regardless of which WBC
QP formulation was tried (flat weighted-sum, task-priority hierarchy, bounded
slack). That pointed at the *interface* between gait/MPC/WBC, not the WBC's
math, so two synchronization gaps are addressed here instead of in wbc.py:

  1. MPC event-triggered replan: the MPC normally only replans on its own
     ~30ms clock, so for up to one MPC cycle after a contact change the WBC's
     force-tracking task is fed a *stale* force distribution (e.g. still
     allocating weight to a leg that just started swinging). Now a contact
     change immediately triggers an out-of-cycle MPC solve.
  2. Swing-reference blending: instead of jumping straight to the full swing
     trajectory the instant a leg is marked swinging (a step discontinuity in
     the *target*, even though the trajectory itself -- see src/gait.py's
     minimum-jerk profile -- has zero velocity/acceleration at its own
     endpoint), the position/velocity/acceleration reference is blended from
     "stay where the foot currently is" up to the full trajectory over
     ``swing_ramp_time``. This absorbs any small mismatch between the
     scheduled liftoff instant and the leg's actual physical state.

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
from src.gait import GAIT_PRESETS, GaitScheduler, raibert_foothold, swing_foot_reference
from src.mpc import ConvexMPC, make_reference, composite_inertia_from_model, GRAVITY
from src.wbc import WholeBodyController


def run_trot(
    secs: float = 5.0,
    vx: float = 0.4,
    vy: float = 0.0,
    yaw_rate: float = 0.0,
    gait_name: str = "trot",
    gait_period: float = 0.4,
    step_height: float = 0.08,
    mpc_dt: float = 0.03,
    mpc_horizon: int = 10,
    ramp: float = 1.0,
    stand_time: float = 0.5,
    wbc_gains=None,
    mpc_weights=None,
    swing_ramp_time: float = 0.03,
    force_ramp_time: float = 0.03,
    # Default OFF: tested as a fix for the walk gait's instability (see
    # PROGRESS.md) and, contrary to the hypothesis, made things measurably
    # WORSE than doing nothing across every ramp duration tried (0.005-0.03s)
    # -- kept as opt-in experiments, not the default path, so run_trot()'s
    # default behavior stays the verified ~3.82s walk-in-place baseline.
    use_event_mpc: bool = False,
    use_swing_ramp: bool = False,
    use_force_ramp: bool = False,
    # Stance-overlap fix (candidate (b) in PROGRESS.md): the walk gait's
    # offsets/duty=0.75 make every handoff a same-tick event -- leg A's
    # touchdown lands on the exact same instant as leg B's liftoff (verified
    # analytically: liftoff_B = offset_B + duty = offset_A = touchdown_A for
    # the walk preset's offsets). That leaves the just-landed leg to absorb
    # its attitude-correction torque transient with one fewer stance leg to
    # share load with, right as it's already at its softest (zero built-up
    # contact stiffness). Bumping duty by stance_overlap/period delays every
    # leg's liftoff by stance_overlap seconds while leaving touchdown timing
    # (still purely offset-driven) untouched, inserting a brief 4-foot-stance
    # buffer after each touchdown before the next leg lifts. Pure gait-phase
    # change -- does not touch mpc.py or wbc.py.
    stance_overlap: float = 0.0,
    video: bool = False,
    video_fps: int = 50,
    verbose: bool = True,
    log_tau: bool = False,
):
    sim = Go2Sim(control_dt=0.002)
    st = sim.reset()
    dt = sim.control_dt
    walk_height = float(st.base_pos[2])          # hold the home trunk height

    mass, com0, inertia = composite_inertia_from_model(sim.model, sim.data)

    gait_duty = None
    if stance_overlap > 0:
        base_duty = GAIT_PRESETS[gait_name]["duty"]
        gait_duty = min(1.0, base_duty + stance_overlap / gait_period)
    gait = GaitScheduler(gait_name, period=gait_period, duty=gait_duty)
    mpc = ConvexMPC(mass, inertia, dt=mpc_dt, horizon=mpc_horizon, mu=0.6,
                    weights=mpc_weights)
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
               n_contact=[], mpc_solve=[], wbc_solve=[], wbc_fail=0,
               foot_force=[], mpc_force=[])
    if log_tau:
        log["tau"] = []
        log["contact"] = []
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
            stance_phase = gs.stance_phase
        else:
            contact = np.ones(4, dtype=bool)         # stand: all feet down
            swing_phase = np.zeros(4)
            stance_phase = np.zeros(4)

        # Any leg's stance/swing flag flipping since the last tick immediately
        # invalidates the MPC's last force allocation (e.g. it may still be
        # assigning weight to a leg that just started swinging) -- trigger an
        # out-of-cycle replan rather than waiting up to one full mpc_dt.
        contact_changed = walking and use_event_mpc and not np.array_equal(contact, prev_contact)

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

        # --- MPC: periodic clock OR immediately on a contact-schedule change ---
        if k % mpc_every == 0 or contact_changed:
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

        # --- swing-foot references, blended in over swing_ramp_time ---
        # Jumping straight to the full trajectory the instant a leg is marked
        # "swinging" is itself a step in the *target* even though the
        # trajectory's own endpoint is smooth (zero vel/accel, see
        # src/gait.py). Blending the reference from "stay at the foot's
        # current actual position" up to the full trajectory absorbs any
        # small mismatch between the scheduled liftoff instant and the leg's
        # true physical state at that instant.
        swing_pos = st.foot_pos.copy()
        swing_vel = np.zeros((4, 3))
        swing_acc = np.zeros((4, 3))
        sw_ramp_frac = (min(1.0, swing_ramp_time / gait.swing_duration)
                       if gait.swing_duration > 0 else 1.0)
        for i in range(4):
            if not contact[i]:
                p, v, a = swing_foot_reference(
                    swing_phase[i], swing_start[i], foothold[i],
                    step_height, gait.swing_duration,
                )
                if use_swing_ramp:
                    blend = min(1.0, swing_phase[i] / sw_ramp_frac) if sw_ramp_frac > 0 else 1.0
                    actual_p = st.foot_pos[i]
                    swing_pos[i] = actual_p + blend * (p - actual_p)
                    swing_vel[i] = blend * v
                    swing_acc[i] = blend * a
                else:
                    swing_pos[i], swing_vel[i], swing_acc[i] = p, v, a

        # --- force ramp for stance legs over force_ramp_time at touchdown and
        # liftoff, applied to the MPC force target fed into the WBC's (soft,
        # low-weight) force-tracking task -- avoids handing the WBC a step
        # change in target force right at a contact transition. ---
        force_scale = np.ones(4)
        if use_force_ramp and walking and gait.stance_duration > 0:
            fr_ramp_frac = min(1.0, force_ramp_time / gait.stance_duration)
            for i in range(4):
                if contact[i]:
                    sp_i = stance_phase[i]
                    force_scale[i] = min(1.0, sp_i / fr_ramp_frac, (1.0 - sp_i) / fr_ramp_frac)
        mpc_forces_wbc = mpc_forces * force_scale[:, None]

        # --- WBC -> torques ---
        base_pos_des = np.array([st.base_pos[0], st.base_pos[1], walk_height])
        # NOTE: tried tracking an explicit integrated yaw reference here (to
        # fix an observed yaw drift of 30+ deg during a walk run) but it made
        # things WORSE empirically (fall at 1.6s instead of 3.8s) -- forcing a
        # fixed yaw target adds a competing torque demand on the same actuators
        # already fighting to correct roll/pitch and swing the lifted leg,
        # apparently making the coupled system harder to stabilize, not easier.
        # Reverted to "yaw follows current" (no yaw feedback) since it measurably
        # performs better; kept as a documented negative result, not a fix.
        yaw_des = st.base_rpy[2]
        tau, wbc_info = wbc.solve(
            sim.data, mpc_forces_wbc, contact,
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
        log["foot_force"].append(st.foot_force.copy())     # measured, sensor-based
        log["mpc_force"].append(mpc_forces[:, 2].copy())   # MPC-commanded vertical force
        if log_tau:
            log["tau"].append(tau.copy())
            log["contact"].append(contact.copy())

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
    ap.add_argument("--gait", type=str, default="trot")
    ap.add_argument("--overlap", type=float, default=0.0,
                     help="stance-overlap fix: delay each leg's liftoff by this many "
                          "seconds after the preceding leg's touchdown (see run_trot)")
    ap.add_argument("--video", action="store_true")
    args = ap.parse_args()
    print(f"=== trot demo: vx={args.vx} period={args.period} step_h={args.step_height} ===")
    run_trot(secs=args.secs, vx=args.vx, vy=args.vy, yaw_rate=args.yaw_rate,
             gait_name=args.gait, gait_period=args.period, step_height=args.step_height,
             stance_overlap=args.overlap, video=args.video)


if __name__ == "__main__":
    main()
