"""Gait scheduling and swing-foot reference generation for the Go2 quadruped.

Phase 2 step 1 (see PROGRESS.md). This module is the *reference generator* that
sits above the Convex-MPC and WBC layers. It answers two questions on every
control tick:

  1. **Which legs are in stance vs swing, and at what phase?**  ``GaitScheduler``
     runs a normalized periodic clock and, per leg, splits it into a stance
     segment and a swing segment using a per-leg phase offset + duty factor.
     For a trot the diagonal pairs (FL,RR) and (FR,RL) are in phase, half a
     cycle apart -- the classic MIT Cheetah contact schedule.

  2. **Where should each swinging foot go, and along what trajectory?**
     ``raibert_foothold`` picks the touchdown location (Raibert heuristic +
     capture-point feedback) and ``swing_foot_reference`` interpolates a smooth
     cycloid arc from lift-off to touchdown, returning position / velocity /
     acceleration so the WBC can track it as a Cartesian task.

The module is deliberately self-contained (numpy only): it takes hip positions,
velocities and commands as plain arrays and returns references. No MuJoCo, no
controller state -- that plumbing lives in the integration loop.

Conventions match ``sim_env``: leg order ``[FL, FR, RL, RR]``, world-frame
Cartesian quantities, SI units.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

LEGS = ("FL", "FR", "RL", "RR")

# Standard gait phase offsets (fraction of a cycle) and duty factors.
# offset = when in the cycle the leg *touches down*; duty = stance fraction.
GAIT_PRESETS: dict[str, dict] = {
    # Trot: diagonal pairs move together, half a cycle apart. duty 0.5.
    "trot": {"offsets": (0.0, 0.5, 0.5, 0.0), "duty": 0.5},
    # Standing: all feet always in stance (offsets irrelevant, duty 1).
    "stand": {"offsets": (0.0, 0.0, 0.0, 0.0), "duty": 1.0},
    # Walk (crawl): one swing leg at a time, duty 0.75.
    "walk": {"offsets": (0.0, 0.5, 0.25, 0.75), "duty": 0.75},
    # Pace: lateral pairs together. Bound: front/rear pairs together.
    "pace": {"offsets": (0.0, 0.5, 0.0, 0.5), "duty": 0.5},
    "bound": {"offsets": (0.0, 0.0, 0.5, 0.5), "duty": 0.5},
}


@dataclass
class LegPhase:
    """Per-leg phase read-out at one instant."""

    in_stance: bool          # True if the foot should be on the ground
    phase: float             # progress within the current segment, [0, 1)
    stance_phase: float      # progress within stance, [0, 1); 0 while swinging
    swing_phase: float       # progress within swing,  [0, 1); 0 while in stance


@dataclass
class GaitState:
    """Full gait read-out for all four legs at one instant."""

    contact: np.ndarray               # (4,) bool stance flags
    stance_phase: np.ndarray          # (4,) float, progress in stance [0,1)
    swing_phase: np.ndarray           # (4,) float, progress in swing  [0,1)
    legs: list[LegPhase] = field(default_factory=list)


class GaitScheduler:
    """Periodic contact scheduler shared by the MPC (contact sequence) and the
    swing controller (swing phase).

    Parameters
    ----------
    gait:
        Name of a preset in :data:`GAIT_PRESETS` (``"trot"`` default), or pass
        ``offsets``/``duty`` explicitly to override.
    period:
        Full gait cycle duration [s]. A shorter period -> higher stepping
        frequency. ``0.5 s`` is a reasonable Go2 trot.
    offsets, duty:
        Optional explicit overrides for the preset (see :data:`GAIT_PRESETS`).
    """

    def __init__(
        self,
        gait: str = "trot",
        period: float = 0.5,
        offsets: tuple[float, ...] | None = None,
        duty: float | None = None,
    ) -> None:
        if gait not in GAIT_PRESETS and (offsets is None or duty is None):
            raise KeyError(
                f"unknown gait '{gait}'; known: {list(GAIT_PRESETS)} "
                "(or pass offsets+duty explicitly)"
            )
        preset = GAIT_PRESETS.get(gait, {})
        self.gait = gait
        self.period = float(period)
        self.offsets = np.asarray(
            offsets if offsets is not None else preset["offsets"], dtype=float
        )
        self.duty = float(duty if duty is not None else preset["duty"])
        if self.offsets.shape != (4,):
            raise ValueError("offsets must have length 4 (one per leg)")
        if not 0.0 < self.duty <= 1.0:
            raise ValueError("duty must be in (0, 1]")

    # ---------------------------------------------------------------- queries
    def phase(self, t: float) -> float:
        """Global normalized cycle phase in [0, 1)."""
        return (t % self.period) / self.period

    def eval(self, t: float) -> GaitState:
        """Contact flags and per-segment phases for all four legs at time ``t``."""
        global_phase = self.phase(t)
        contact = np.zeros(4, dtype=bool)
        stance_ph = np.zeros(4)
        swing_ph = np.zeros(4)
        legs: list[LegPhase] = []
        for i in range(4):
            # Local phase measured from this leg's touchdown moment.
            local = (global_phase - self.offsets[i]) % 1.0
            if local < self.duty:
                in_stance = True
                sp = local / self.duty if self.duty > 0 else 0.0
                stance_ph[i] = sp
                seg_phase = sp
            else:
                in_stance = False
                wp = (local - self.duty) / (1.0 - self.duty)
                swing_ph[i] = wp
                seg_phase = wp
            contact[i] = in_stance
            legs.append(
                LegPhase(
                    in_stance=in_stance,
                    phase=seg_phase,
                    stance_phase=stance_ph[i],
                    swing_phase=swing_ph[i],
                )
            )
        return GaitState(contact=contact, stance_phase=stance_ph, swing_phase=swing_ph, legs=legs)

    def contact_sequence(self, t: float, horizon: int, dt: float) -> np.ndarray:
        """Predicted contact table for the MPC prediction horizon.

        Returns
        -------
        (horizon, 4) bool array: row ``k`` is the stance state at ``t + k*dt``.
        """
        seq = np.zeros((horizon, 4), dtype=bool)
        for k in range(horizon):
            seq[k] = self.eval(t + k * dt).contact
        return seq

    @property
    def stance_duration(self) -> float:
        """Time [s] a leg spends in stance per cycle."""
        return self.duty * self.period

    @property
    def swing_duration(self) -> float:
        """Time [s] a leg spends in swing per cycle."""
        return (1.0 - self.duty) * self.period


# ----------------------------------------------------------- foothold planning
def raibert_foothold(
    hip_pos: np.ndarray,
    base_vel: np.ndarray,
    vel_cmd: np.ndarray,
    stance_duration: float,
    base_height: float,
    *,
    yaw_rate: float = 0.0,
    yaw_rate_cmd: float = 0.0,
    k_capture: float | None = None,
    ground_height: float = 0.0,
    g: float = 9.81,
) -> np.ndarray:
    """Raibert-style touchdown location for one swing foot (world frame).

    ``p = hip + (T_stance/2) * v + k * (v - v_cmd)``   (MIT Cheetah 3, eq. 14)

    The first term is the Raibert "neutral point": with the *actual* base
    velocity ``v`` it places the foot so the leg sweeps symmetrically about the
    hip during stance (no net accel at steady state). The second term is the
    linear-inverted-pendulum "capture point" feedback that corrects velocity
    error -- when ``v < v_cmd`` it steps the foot *behind* the hip so the stance
    push accelerates the body toward the command. Vertical target is the ground.

    At steady state (``v == v_cmd``) the capture term vanishes and the foot sits
    ``(T_stance/2)*v`` ahead of the hip -- forward of the hip when walking
    forward, as expected.

    Parameters
    ----------
    hip_pos:
        (3,) world position of the hip (shoulder) for this leg.
    base_vel, vel_cmd:
        (3,) actual and commanded world-frame linear velocity of the base.
    stance_duration:
        Time the foot will spend in stance [s] (``GaitScheduler.stance_duration``).
    base_height:
        Nominal CoM height above the feet [m] -- sets the capture-point gain.
    yaw_rate, yaw_rate_cmd:
        Optional turning: adds a tangential offset so feet lead the turn.
    k_capture:
        Capture-point gain; defaults to ``sqrt(base_height / g)`` (LIP model).
    ground_height:
        World z of the contact surface [m].
    """
    hip_pos = np.asarray(hip_pos, dtype=float)
    base_vel = np.asarray(base_vel, dtype=float)
    vel_cmd = np.asarray(vel_cmd, dtype=float)
    if k_capture is None:
        k_capture = float(np.sqrt(max(base_height, 1e-3) / g))

    foot = hip_pos.copy()
    # Neutral point (uses actual velocity) + capture-point feedback.
    foot[:2] = (
        hip_pos[:2]
        + 0.5 * stance_duration * base_vel[:2]
        + k_capture * (base_vel[:2] - vel_cmd[:2])
    )
    # Turning: shift foot tangentially so the stance foot helps rotate the base.
    if yaw_rate_cmd != 0.0 or yaw_rate != 0.0:
        r = hip_pos[:2] - hip_pos[:2].mean()  # placeholder radius ~ 0; kept minimal
        # tangential = omega x r ; here use commanded yaw rate about base z.
        tangential = 0.5 * stance_duration * yaw_rate_cmd * np.array([-r[1], r[0]])
        foot[:2] += tangential
    foot[2] = ground_height
    return foot


# ----------------------------------------------------------- swing trajectory
def _cycloid(s: float) -> tuple[float, float, float]:
    """Cycloid interpolation factor and its 1st/2nd derivatives w.r.t. phase s.

    ``c(s) = s - sin(2*pi*s)/(2*pi)`` maps [0,1]->[0,1] with zero endpoint
    velocity (smooth lift-off / touchdown). Derivatives are w.r.t. the
    normalized phase ``s``; callers scale by ``1/T_swing`` for time derivatives.
    """
    two_pi = 2.0 * np.pi
    c = s - np.sin(two_pi * s) / two_pi
    dc = 1.0 - np.cos(two_pi * s)            # d c / d s
    ddc = two_pi * np.sin(two_pi * s)        # d^2 c / d s^2
    return c, dc, ddc


def swing_foot_reference(
    swing_phase: float,
    p_lift: np.ndarray,
    p_touchdown: np.ndarray,
    step_height: float,
    swing_duration: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Cartesian swing-foot reference (position, velocity, acceleration).

    Horizontal motion is a cycloid from ``p_lift`` to ``p_touchdown``. Vertical
    motion is the same cycloid baseline plus a raised-cosine "bell" of amplitude
    ``step_height`` that peaks at mid-swing and returns to the interpolated
    endpoint height, giving zero vertical velocity at lift-off and touchdown.

    Parameters
    ----------
    swing_phase:
        Progress through swing in [0, 1] (``LegPhase.swing_phase``).
    p_lift, p_touchdown:
        (3,) world foot positions at lift-off and planned touchdown.
    step_height:
        Peak foot clearance above the straight-line path [m].
    swing_duration:
        Duration of the swing phase [s], used to turn phase-derivatives into
        time-derivatives for velocity/acceleration.

    Returns
    -------
    (pos, vel, acc): each (3,) world-frame arrays.
    """
    p_lift = np.asarray(p_lift, dtype=float)
    p_touchdown = np.asarray(p_touchdown, dtype=float)
    s = float(np.clip(swing_phase, 0.0, 1.0))
    T = max(swing_duration, 1e-6)
    inv_T = 1.0 / T

    c, dc, ddc = _cycloid(s)
    delta = p_touchdown - p_lift

    # Baseline interpolation (all 3 axes).
    pos = p_lift + c * delta
    vel = dc * delta * inv_T
    acc = ddc * delta * inv_T * inv_T

    # Vertical clearance bell: h/2 * (1 - cos(2*pi*s)) -> 0 at ends, peak h at s=0.5.
    two_pi = 2.0 * np.pi
    bell = 0.5 * step_height * (1.0 - np.cos(two_pi * s))
    bell_d = 0.5 * step_height * two_pi * np.sin(two_pi * s)
    bell_dd = 0.5 * step_height * two_pi * two_pi * np.cos(two_pi * s)
    pos[2] += bell
    vel[2] += bell_d * inv_T
    acc[2] += bell_dd * inv_T * inv_T

    return pos, vel, acc


# ------------------------------------------------------------------- self-test
def _self_test() -> bool:
    """Validate the contact schedule and swing trajectory; render a plot.

    Checks (all asserted):
      * trot diagonal pairs (FL,RR) and (FR,RL) share identical contact state;
      * duty factor is respected (stance fraction ~ duty over a cycle);
      * exactly 2 feet in stance at all times for a 0.5-duty trot;
      * swing trajectory hits its endpoints with zero velocity and reaches the
        commanded clearance at mid-swing.
    """
    import sys

    ok = True

    def check(name: str, cond: bool) -> None:
        nonlocal ok
        ok = ok and cond
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")

    print("=== gait.py self-test ===")
    T = 0.5
    sched = GaitScheduler("trot", period=T)

    # --- sample a full cycle ---
    n = 500
    ts = np.linspace(0, T, n, endpoint=False)
    contacts = np.array([sched.eval(t).contact for t in ts])  # (n, 4)

    # Diagonal pairs move together: FL(0)==RR(3), FR(1)==RL(2).
    check("trot diagonal FL==RR in phase", bool(np.all(contacts[:, 0] == contacts[:, 3])))
    check("trot diagonal FR==RL in phase", bool(np.all(contacts[:, 1] == contacts[:, 2])))

    # Duty factor: each leg in stance ~50% of the cycle.
    stance_frac = contacts.mean(axis=0)
    check(
        f"duty ~0.5 per leg (got {np.round(stance_frac, 3).tolist()})",
        bool(np.all(np.abs(stance_frac - 0.5) < 0.02)),
    )

    # Exactly 2 feet down at all times (trot).
    n_down = contacts.sum(axis=1)
    check(
        f"exactly 2 feet in stance always (min={n_down.min()}, max={n_down.max()})",
        bool(np.all(n_down == 2)),
    )

    # Diagonals are half a cycle out of phase with the other pair.
    check(
        "diagonal pairs are anti-phase",
        bool(np.all(contacts[:, 0] != contacts[:, 1])),
    )

    # --- contact-sequence table for MPC ---
    seq = sched.contact_sequence(0.0, horizon=10, dt=0.03)
    check("contact_sequence shape (10,4)", seq.shape == (10, 4))

    # --- swing trajectory endpoints ---
    p0 = np.array([0.20, 0.15, 0.0])
    p1 = np.array([0.28, 0.15, 0.0])
    h = 0.08
    Tsw = sched.swing_duration
    pos0, vel0, _ = swing_foot_reference(0.0, p0, p1, h, Tsw)
    pos1, vel1, _ = swing_foot_reference(1.0, p0, p1, h, Tsw)
    posm, _, _ = swing_foot_reference(0.5, p0, p1, h, Tsw)
    check("swing starts at lift-off point", bool(np.allclose(pos0, p0, atol=1e-9)))
    check("swing ends at touchdown point", bool(np.allclose(pos1, p1, atol=1e-9)))
    check("swing zero vertical velocity at endpoints",
          bool(abs(vel0[2]) < 1e-9 and abs(vel1[2]) < 1e-9))
    check(f"swing clearance ~step_height at mid (got {posm[2]:.4f})",
          bool(abs(posm[2] - h) < 1e-6))

    # --- Raibert foothold sanity: zero cmd/vel -> foot under hip ---
    hip = np.array([0.19, 0.14, 0.30])
    fh = raibert_foothold(hip, np.zeros(3), np.zeros(3),
                          sched.stance_duration, base_height=0.30)
    check("foothold under hip when static", bool(np.allclose(fh[:2], hip[:2], atol=1e-9)))
    # Steady-state forward walk (v == v_cmd): capture term vanishes, foot leads
    # the hip by (T_stance/2)*v.
    v_fwd = np.array([0.5, 0, 0])
    fh_fwd = raibert_foothold(hip, v_fwd, v_fwd,
                              sched.stance_duration, base_height=0.30)
    expected_lead = 0.5 * sched.stance_duration * v_fwd[0]
    check(f"steady forward: foot leads hip by T/2*v (got {fh_fwd[0]-hip[0]:.4f}, "
          f"want {expected_lead:.4f})",
          bool(abs((fh_fwd[0] - hip[0]) - expected_lead) < 1e-9))
    # Velocity error drives capture feedback: v<v_cmd steps foot behind hip.
    fh_accel = raibert_foothold(hip, np.zeros(3), v_fwd,
                                sched.stance_duration, base_height=0.30)
    check("capture: v<v_cmd steps foot behind hip", bool(fh_accel[0] < hip[0]))

    # --- optional plot ---
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from pathlib import Path

        fig, (ax0, ax1) = plt.subplots(2, 1, figsize=(9, 6))
        # contact schedule gantt
        for i, leg in enumerate(LEGS):
            ax0.fill_between(ts, i + 0.1, i + 0.9, where=contacts[:, i],
                             step="mid", alpha=0.8, label=leg)
        ax0.set_yticks([i + 0.5 for i in range(4)])
        ax0.set_yticklabels(LEGS)
        ax0.set_xlabel("time [s]"); ax0.set_title(f"trot contact schedule (T={T}s)")
        # swing foot arc (x-z)
        ss = np.linspace(0, 1, 100)
        arc = np.array([swing_foot_reference(s, p0, p1, h, Tsw)[0] for s in ss])
        ax1.plot(arc[:, 0], arc[:, 2], "-o", ms=2)
        ax1.set_xlabel("foot x [m]"); ax1.set_ylabel("foot z [m]")
        ax1.set_title("swing foot trajectory (cycloid + clearance bell)")
        ax1.axis("equal"); ax1.grid(True, alpha=0.3)
        fig.tight_layout()
        out = Path(__file__).resolve().parents[1] / "results" / "gait_schedule.png"
        out.parent.mkdir(exist_ok=True)
        fig.savefig(out, dpi=110)
        print(f"  wrote plot -> {out}")
    except Exception as e:  # plotting is optional, never fail the test on it
        print(f"  (plot skipped: {e})")

    print(f"RESULT: {'PASS' if ok else 'FAIL'}")
    if not ok:
        sys.exit(1)
    return ok


if __name__ == "__main__":
    _self_test()
