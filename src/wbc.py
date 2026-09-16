"""Whole-Body Control (WBC) torque QP for the Go2 quadruped.

Phase 2 step 3 (see PROGRESS.md). The Convex MPC (step 2) outputs desired ground
reaction forces for the stance feet, assuming a single rigid body. The WBC turns
those forces -- together with base pose/height tasks and swing-foot tracking --
into the 12 joint torques that are actually commanded, while respecting the full
floating-base dynamics, contact/friction constraints and actuator limits.

QP (decision x = [qddot(18), f(12), tau(12)], 42 vars):

  subject to (P0, always hard):
    M qddot + h = S^T tau + Jc^T f            (floating-base EoM, 18 eq)
    f_i = 0            for swing feet          (no force off-ground)
    |f_x|,|f_y| <= mu f_z,  f_min <= f_z <= f_max   for stance feet (friction)
    tau_min <= tau <= tau_max                 (actuator limits)

Tasks are solved as a strict priority hierarchy (task-priority / hierarchical
QP), not a single flat weighted sum:

  Level 1 (highest): stance no-slip     -- planted feet must not accelerate.
  Level 2:           base attitude + height PD.
  Level 3 (lowest):  swing-foot tracking + MPC force tracking + regularization
                     (a weighted sum *within* this level is fine -- these are
                     the "nice to have" tasks, none of which should be allowed
                     to steal capacity from level 1/2).

Each level is solved as its own QP subject to P0 plus an equality constraint
freezing every higher-priority level's already-achieved task value (a standard
sequential-QP formulation of task-priority WBC). This replaces an earlier flat
weighted-sum formulation that turned out to be fragile at multi-contact
transitions: when a leg's own actuators saturated during swing, a single QP
could satisfy the swing task "for free" by accelerating the *base* instead
(same cost, since a leg Jacobian has nonzero base columns), launching the
trunk every time a leg lifted off. Strict priority prevents a lower-priority
task from ever trading away a higher-priority one's optimum, which a flat
weighted sum cannot guarantee no matter how the weights are tuned.

Dynamics quantities come straight from MuJoCo: mass matrix ``mj_fullM``, bias
force ``data.qfrc_bias`` (Coriolis + gravity), and foot Jacobians ``mj_jacSite``.
The Jacobian-derivative bias ``Jdot*qvel`` is obtained by finite-differencing the
foot Jacobian on a scratch MjData (exact to first order, no cacc/gravity
bookkeeping). Solved with OSQP.

Conventions (verified against the model): free-joint ``qvel[0:3]`` is world-frame
linear velocity, ``qvel[3:6]`` is body-frame angular velocity; actuated dofs are
``qvel`` indices 6..17; leg order ``[FL, FR, RL, RR]``.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np
import osqp
import scipy.sparse as sp

from src.sim_env import quat_to_rotmat

N_LEGS = 4
NV = 18            # floating base (6) + joints (12)
N_JOINTS = 12
N_FORCE = 3 * N_LEGS
N_DEC = NV + N_FORCE + N_JOINTS   # 42

# decision-vector slices
QDD = slice(0, NV)
FRC = slice(NV, NV + N_FORCE)
TAU = slice(NV + N_FORCE, N_DEC)

GRAVITY = 9.81


@dataclass
class WBCGains:
    """PD gains and task weights.

    The base orientation/height tasks carry large weights so that -- with all
    stance feet planted (hard contact) -- they behave like near-hard tasks and
    actually stabilize the (inherently unstable, inverted-pendulum-like) trunk.
    Softer weights let force-tracking/regularization dilute them and the robot
    topples. Horizontal (x,y) base position is intentionally *not* tracked
    (kp=0) so the robot can translate while walking; velocity is tracked via kd.
    """

    # base orientation (roll, pitch, yaw), body frame
    kp_ori: tuple[float, float, float] = (2000.0, 2000.0, 800.0)
    kd_ori: tuple[float, float, float] = (100.0, 100.0, 60.0)
    # base position (x, y, z), world frame. x,y position untracked (kp=0).
    kp_pos: tuple[float, float, float] = (0.0, 0.0, 1500.0)
    kd_pos: tuple[float, float, float] = (30.0, 30.0, 120.0)
    # swing foot Cartesian PD
    kp_swing: float = 500.0
    kd_swing: float = 25.0

    # task weights
    w_ori: float = 1000.0
    w_pos: float = 500.0
    w_swing: float = 200.0
    w_contact: float = 300.0   # soft stance no-slip (kept soft so qddot stays
                               # free to satisfy the EoM -> QP stays feasible)
    w_force: float = 1.0       # track MPC reaction forces (soft preference)
    # regularization
    w_reg_qdd: float = 1e-3
    w_reg_f: float = 1e-4
    w_reg_tau: float = 1e-4
    # task-acceleration saturation: keeps strong gains from demanding explosive
    # forces on a large transient error (which would saturate torque / launch
    # the trunk). Small errors are still corrected stiffly; big ones are capped.
    a_lin_max: float = 8.0     # m/s^2   (per horizontal/vertical axis)
    a_ang_max: float = 30.0    # rad/s^2 (per axis)
    a_swing_max: float = 60.0  # m/s^2   (swing foot)


def _so3_log(R: np.ndarray) -> np.ndarray:
    """Small-angle rotation-vector of R (body-frame attitude error)."""
    return 0.5 * np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])


class WholeBodyController:
    """Maps MPC forces + base/swing references to 12 joint torques via a QP."""

    def __init__(
        self,
        model: mujoco.MjModel,
        foot_site_ids: np.ndarray,
        joint_dof_adr: np.ndarray,
        torque_limits: np.ndarray,
        mu: float = 0.6,
        f_max: float = 250.0,
        f_min: float = 1.0,
        gains: WBCGains | None = None,
    ) -> None:
        self.model = model
        self.foot_site_ids = np.asarray(foot_site_ids)
        self.joint_dof_adr = np.asarray(joint_dof_adr)   # length 12, values 6..17
        self.torque_limits = np.asarray(torque_limits, dtype=float)  # (12,2)
        self.mu = float(mu)
        self.f_max = float(f_max)
        self.f_min = float(f_min)
        self.g = gains or WBCGains()

        # Selection matrix S^T (18x12): identity on actuated dofs.
        self._St = np.zeros((NV, N_JOINTS))
        for j, dof in enumerate(self.joint_dof_adr):
            self._St[dof, j] = 1.0

        self._scratch = mujoco.MjData(model)   # for Jdot*qvel finite differencing
        # One cached OSQP instance + sparsity pattern per hierarchy level (1,2,3)
        # so each level still gets warm-started tick-to-tick.
        self._prob: dict[int, osqp.OSQP] = {}
        self._pattern: dict[int, tuple] = {}
        self._last_tau = np.zeros(N_JOINTS)    # fallback on a failed solve
        self.last_info: dict = {}

    # -------------------------------------------------------- dynamics reads
    def _mass_matrix(self, data) -> np.ndarray:
        M = np.zeros((NV, NV))
        mujoco.mj_fullM(self.model, data, M)  # (model, data, dst)
        return M

    def _foot_jacobians(self, data) -> np.ndarray:
        """(4, 3, 18) translational foot Jacobians at the current config."""
        J = np.zeros((N_LEGS, 3, NV))
        Jp = np.zeros((3, NV))
        Jr = np.zeros((3, NV))
        for i, sid in enumerate(self.foot_site_ids):
            mujoco.mj_jacSite(self.model, data, Jp, Jr, sid)
            J[i] = Jp
        return J

    def _foot_acc_bias(self, data, J0: np.ndarray, eps: float = 1e-6) -> np.ndarray:
        """(4, 3) Jdot*qvel per foot, via finite difference on scratch data."""
        s = self._scratch
        s.qpos[:] = data.qpos
        s.qvel[:] = data.qvel
        # advance qpos along current qvel by eps, recompute kinematics only.
        mujoco.mj_integratePos(self.model, s.qpos, s.qvel, eps)
        mujoco.mj_kinematics(self.model, s)
        mujoco.mj_comPos(self.model, s)
        Jp = np.zeros((3, NV))
        Jr = np.zeros((3, NV))
        bias = np.zeros((N_LEGS, 3))
        qvel = data.qvel
        for i, sid in enumerate(self.foot_site_ids):
            mujoco.mj_jacSite(self.model, s, Jp, Jr, sid)
            Jdot = (Jp - J0[i]) / eps
            bias[i] = Jdot @ qvel
        return bias

    # --------------------------------------------------------------- solve
    def solve(
        self,
        data,
        mpc_forces: np.ndarray,
        contact: np.ndarray,
        *,
        base_pos_des: np.ndarray,
        base_vel_des: np.ndarray,
        base_rpy_des: np.ndarray,
        base_omega_des: np.ndarray,
        swing_pos_ref: np.ndarray | None = None,
        swing_vel_ref: np.ndarray | None = None,
        swing_acc_ref: np.ndarray | None = None,
    ) -> tuple[np.ndarray, dict]:
        """Solve the WBC task hierarchy and return 12 joint torques.

        Parameters
        ----------
        data:
            Live ``mujoco.MjData`` (must be forward-consistent with the state).
        mpc_forces:
            (4, 3) desired world-frame reaction forces from the MPC.
        contact:
            (4,) bool stance flags for the current instant.
        base_pos_des, base_vel_des:
            (3,) desired CoM position / velocity (world).
        base_rpy_des, base_omega_des:
            (3,) desired base orientation (roll,pitch,yaw) and body-frame angular
            velocity.
        swing_pos_ref, swing_vel_ref, swing_acc_ref:
            (4, 3) Cartesian references for swing feet (ignored for stance feet).
            May be ``None`` if all feet are in stance.

        Returns
        -------
        (tau, info): tau is (12,) joint torques; info has solver status + residuals.
        """
        mpc_forces = np.asarray(mpc_forces, dtype=float).reshape(N_LEGS, 3)
        contact = np.asarray(contact, dtype=bool).reshape(N_LEGS)

        M = self._mass_matrix(data)
        h = data.qfrc_bias.copy()                    # (18,)
        J = self._foot_jacobians(data)               # (4,3,18)
        bias = self._foot_acc_bias(data, J)          # (4,3)
        qvel = data.qvel.copy()

        # --- current base state ---
        p = data.qpos[0:3].copy()
        v = qvel[0:3].copy()                         # world linear vel
        omega = qvel[3:6].copy()                     # body angular vel
        R = quat_to_rotmat(data.qpos[3:7])           # body->world

        gns = self.g

        # ---------- P0: hard constraints, shared by every priority level ----------
        Acon0, l0, u0 = self._hard_constraints(M, h, J, contact)

        # ---------- Level 1: stance no-slip ----------
        stance_idx = [i for i in range(N_LEGS) if contact[i]]
        extra_eq: list[tuple[np.ndarray, np.ndarray]] = []
        level_status = []
        if stance_idx:
            A1 = np.zeros((3 * len(stance_idx), N_DEC))
            b1 = np.zeros(3 * len(stance_idx))
            for row, i in enumerate(stance_idx):
                A1[3 * row:3 * row + 3, QDD] = J[i]
                b1[3 * row:3 * row + 3] = -bias[i]
            x1, ok1, info1 = self._solve_level(1, Acon0, l0, u0, [(A1, b1, 1.0)], [])
            level_status.append(info1["status_val"])
            if ok1:
                extra_eq.append((A1, A1 @ x1))

        # ---------- Level 2: base attitude + height ----------
        R_des = _rpy_to_rotmat(base_rpy_des)
        e_ori = _so3_log(R.T @ R_des)                # body-frame attitude error
        a_ang = (np.asarray(gns.kp_ori) * e_ori
                 + np.asarray(gns.kd_ori) * (np.asarray(base_omega_des) - omega))
        a_ang = np.clip(a_ang, -gns.a_ang_max, gns.a_ang_max)
        a_lin = (np.asarray(gns.kp_pos) * (np.asarray(base_pos_des) - p)
                 + np.asarray(gns.kd_pos) * (np.asarray(base_vel_des) - v))
        a_lin = np.clip(a_lin, -gns.a_lin_max, gns.a_lin_max)
        A2 = np.zeros((6, N_DEC)); b2 = np.zeros(6)
        A2[0:3, 3:6] = np.eye(3) * np.sqrt(gns.w_ori); b2[0:3] = a_ang * np.sqrt(gns.w_ori)
        A2[3:6, 0:3] = np.eye(3) * np.sqrt(gns.w_pos); b2[3:6] = a_lin * np.sqrt(gns.w_pos)
        x2, ok2, info2 = self._solve_level(2, Acon0, l0, u0, [(A2, b2, 1.0)], extra_eq)
        level_status.append(info2["status_val"])
        if ok2:
            extra_eq.append((A2, A2 @ x2))

        # ---------- Level 3 (lowest): swing tracking + force tracking + reg ----------
        level3_tasks: list[tuple[np.ndarray, np.ndarray, float]] = []
        for i in range(N_LEGS):
            if not contact[i]:
                foot_v = J[i] @ qvel
                pref = swing_pos_ref[i] if swing_pos_ref is not None else np.zeros(3)
                vref = swing_vel_ref[i] if swing_vel_ref is not None else np.zeros(3)
                aref = swing_acc_ref[i] if swing_acc_ref is not None else np.zeros(3)
                foot_p = data.site_xpos[self.foot_site_ids[i]].copy()
                a_des = (aref + gns.kp_swing * (pref - foot_p)
                         + gns.kd_swing * (vref - foot_v))
                a_des = np.clip(a_des, -gns.a_swing_max, gns.a_swing_max)
                # Decouple from the base: a leg's foot Jacobian has nonzero
                # columns in both the base (0:6) and that leg's own 3 joints,
                # so without this a lower-priority swing task could still try
                # to "cheat" via the base within its own level (though the
                # attitude/height task is already frozen above it by then, this
                # keeps the swing task's own qddot cost from wasting weight on
                # a direction that level 2 has already locked down).
                A_foot = np.zeros((3, N_DEC)); A_foot[:, QDD] = J[i]
                A_foot[:, 0:6] = 0.0
                level3_tasks.append((A_foot, a_des - bias[i], gns.w_swing))
            else:
                A_f = np.zeros((3, N_DEC))
                A_f[:, NV + 3 * i:NV + 3 * i + 3] = np.eye(3)
                level3_tasks.append((A_f, mpc_forces[i], gns.w_force))
        # regularization (only at the final level -- a tie-breaker among
        # otherwise-equivalent solutions, not something that should compete
        # with any actual task).
        A_reg = np.eye(N_DEC)
        b_reg = np.zeros(N_DEC)
        reg_w = np.zeros(N_DEC)
        reg_w[QDD] = gns.w_reg_qdd
        reg_w[FRC] = gns.w_reg_f
        reg_w[TAU] = gns.w_reg_tau
        level3_tasks.append((A_reg, b_reg, reg_w))  # per-row weight vector

        x3, ok3, info3 = self._solve_level(3, Acon0, l0, u0, level3_tasks, extra_eq)
        level_status.append(info3["status_val"])

        qdd = x3[QDD]
        f = x3[FRC].reshape(N_LEGS, 3)
        tau = x3[TAU].copy()

        # Guard: if the final level's QP was not solved to optimality, don't
        # command garbage (OSQP returns large arbitrary values when
        # infeasible). Hold the last good torque instead.
        if not ok3:
            tau = self._last_tau.copy()
        else:
            self._last_tau = tau.copy()

        # residuals for diagnostics
        Jc = J.reshape(N_LEGS * 3, NV)
        eom_res = float(np.linalg.norm(M @ qdd + h - self._St @ tau - Jc.T @ x3[FRC]))
        info = dict(info3)
        info.update(eom_residual=eom_res,
                    level_status=level_status,
                    force_track_err=float(np.linalg.norm((f - mpc_forces)[contact]))
                    if contact.any() else 0.0)
        return tau, info

    # ------------------------------------------------------------- hierarchy
    def _hard_constraints(
        self, M: np.ndarray, h: np.ndarray, J: np.ndarray, contact: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """P0: EoM equality + swing f=0 + stance friction/normal + torque box.

        Shared, unchanged, across all three priority levels for a given tick.
        """
        A_rows: list[np.ndarray] = []
        l_rows: list[np.ndarray] = []
        u_rows: list[np.ndarray] = []

        # (1) EoM equality: [M | -Jc^T | -S^T] x = -h
        Jc = J.reshape(N_LEGS * 3, NV)              # (12,18)
        A_eom = np.zeros((NV, N_DEC))
        A_eom[:, QDD] = M
        A_eom[:, FRC] = -Jc.T
        A_eom[:, TAU] = -self._St
        A_rows.append(A_eom); l_rows.append(-h); u_rows.append(-h)

        # (2) swing feet: f_i = 0 (no force off the ground). Stance no-slip is
        #     the level-1 priority task, not a hard constraint here, so qddot
        #     stays free enough that the EoM is always satisfiable.
        BIG = 1e6
        for i in range(N_LEGS):
            if not contact[i]:
                A_z = np.zeros((3, N_DEC))
                A_z[:, NV + 3 * i:NV + 3 * i + 3] = np.eye(3)
                A_rows.append(A_z); l_rows.append(np.zeros(3)); u_rows.append(np.zeros(3))

        # (3) stance friction pyramid + normal bounds
        cone = np.array([
            [1.0, 0.0, -self.mu],
            [-1.0, 0.0, -self.mu],
            [0.0, 1.0, -self.mu],
            [0.0, -1.0, -self.mu],
        ])
        for i in range(N_LEGS):
            if contact[i]:
                cols = slice(NV + 3 * i, NV + 3 * i + 3)
                A_c = np.zeros((4, N_DEC)); A_c[:, cols] = cone
                A_rows.append(A_c)
                l_rows.append(np.full(4, -BIG)); u_rows.append(np.zeros(4))
                A_n = np.zeros((1, N_DEC)); A_n[0, NV + 3 * i + 2] = 1.0
                A_rows.append(A_n)
                l_rows.append(np.array([self.f_min])); u_rows.append(np.array([self.f_max]))

        # (4) torque box
        A_t = np.zeros((N_JOINTS, N_DEC)); A_t[:, TAU] = np.eye(N_JOINTS)
        A_rows.append(A_t)
        l_rows.append(self.torque_limits[:, 0]); u_rows.append(self.torque_limits[:, 1])

        return np.vstack(A_rows), np.concatenate(l_rows), np.concatenate(u_rows)

    def _solve_level(
        self,
        level: int,
        Acon0: np.ndarray, l0: np.ndarray, u0: np.ndarray,
        tasks: list[tuple[np.ndarray, np.ndarray, float | np.ndarray]],
        extra_eq: list[tuple[np.ndarray, np.ndarray]],
    ) -> tuple[np.ndarray, bool, dict]:
        """Solve one priority level: minimize the weighted task(s) subject to P0
        plus equality constraints freezing every higher-priority level's
        already-achieved value (``extra_eq``). Returns (x, solved_ok, info).

        Each task's weight ``w`` may be a scalar (uniform weight on that task's
        rows) or a per-decision-variable vector matching ``A``'s row count
        (used for the level-3 regularization task, whose rows are one per
        decision variable with different weights for qddot/f/tau).
        """
        P = np.zeros((N_DEC, N_DEC))
        q = np.zeros(N_DEC)
        for A, b, w in tasks:
            if np.isscalar(w):
                P += 2.0 * w * (A.T @ A)
                q += -2.0 * w * (A.T @ b)
            else:
                w = np.asarray(w)
                Wa = w[:, None] * A   # diag(w) @ A
                P += 2.0 * (A.T @ Wa)
                q += -2.0 * (A.T @ (w * b))
        P[np.diag_indices_from(P)] += 1e-8  # numerical conditioning only

        A_rows = [Acon0]; l_rows = [l0]; u_rows = [u0]
        for A_prev, e_prev in extra_eq:
            tol = np.maximum(1e-4, 1e-3 * np.abs(e_prev))
            A_rows.append(A_prev)
            l_rows.append(e_prev - tol)
            u_rows.append(e_prev + tol)
        A_con = np.vstack(A_rows)
        l = np.concatenate(l_rows)
        u = np.concatenate(u_rows)

        P_sp = sp.triu(sp.csc_matrix(P), format="csc")
        A_sp = sp.csc_matrix(A_con)
        x, info = self._solve_osqp(level, P_sp, q, A_sp, l, u)
        ok = info["status_val"] in (1, 2)
        return x, ok, info

    def _solve_osqp(
        self, level: int, P_sp, q, A_sp, l, u
    ) -> tuple[np.ndarray, dict]:
        """Solve one level's QP, warm-started via a cached OSQP instance kept
        per priority level (level 1/2/3 each get their own persisted problem +
        sparsity-pattern cache across ticks, since their constraint counts
        differ from each other but are usually stable tick-to-tick).
        """
        pattern = (P_sp.indptr.tobytes(), P_sp.indices.tobytes(),
                   A_sp.indptr.tobytes(), A_sp.indices.tobytes(),
                   A_sp.shape)
        if level not in self._prob or pattern != self._pattern.get(level):
            prob = osqp.OSQP()
            prob.setup(P=P_sp, q=q, A=A_sp, l=l, u=u, verbose=False,
                      warm_starting=True, eps_abs=1e-5, eps_rel=1e-5,
                      max_iter=4000, polish=True)
            self._prob[level] = prob
            self._pattern[level] = pattern
        else:
            self._prob[level].update(Px=P_sp.data, Ax=A_sp.data, q=q, l=l, u=u)
        res = self._prob[level].solve()
        info = {
            "status": res.info.status,
            "status_val": int(res.info.status_val),
            "iter": int(res.info.iter),
            "solve_time_ms": float(res.info.solve_time) * 1e3,
        }
        if res.x is None or np.any(np.isnan(res.x)):
            return np.zeros(N_DEC), info
        return res.x, info


# ------------------------------------------------------------------ helpers
def _rpy_to_rotmat(rpy: np.ndarray) -> np.ndarray:
    """Roll-pitch-yaw (XYZ) -> rotation matrix (body->world)."""
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp_ = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp_], [0, 1, 0], [-sp_, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


# ------------------------------------------------------------------ self-test
def _self_test() -> bool:
    import sys
    from src.sim_env import Go2Sim
    from src.mpc import ConvexMPC, make_reference, composite_inertia_from_model

    ok = True

    def check(name: str, cond: bool) -> None:
        nonlocal ok
        ok = ok and bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")

    print("=== wbc.py self-test ===")
    sim = Go2Sim(control_dt=0.002)
    st = sim.reset()
    mass, com, inertia = composite_inertia_from_model(sim.model, sim.data)
    Wg = mass * GRAVITY

    wbc = WholeBodyController(
        sim.model, sim.foot_site_ids, sim.joint_qvel_adr, sim.torque_limits,
        mu=0.6,
    )

    home_pos = st.base_pos.copy()
    home_rpy = np.zeros(3)

    # --- Test 1: standing WBC, MPC forces = even gravity support ---
    contact = np.ones(4, dtype=bool)
    mpc_f = np.zeros((4, 3))
    mpc_f[:, 2] = Wg / 4.0
    tau, info = wbc.solve(
        sim.data, mpc_f, contact,
        base_pos_des=home_pos, base_vel_des=np.zeros(3),
        base_rpy_des=home_rpy, base_omega_des=np.zeros(3),
    )
    print(f"  [stand] status={info['status']} iter={info['iter']} "
          f"t={info['solve_time_ms']:.2f}ms  eom_res={info['eom_residual']:.2e}  "
          f"|tau|max={np.abs(tau).max():.2f}Nm  ftrack={info['force_track_err']:.3f}")
    check("solver optimal (stand)", info["status_val"] == 1)
    check("EoM residual ~ 0", info["eom_residual"] < 1e-6)
    check("torques within limits",
          bool(np.all(tau >= sim.torque_limits[:, 0] - 1e-6)
               and np.all(tau <= sim.torque_limits[:, 1] + 1e-6)))
    check("WBC tracks MPC forces (err small)", info["force_track_err"] < 1.0)

    # --- Test 2: closed-loop WBC-only stand should stay upright ~2s ---
    sim.reset()
    heights, rolls, pitches = [], [], []
    steps = int(2.0 / sim.control_dt)
    for _ in range(steps):
        tau, info = wbc.solve(
            sim.data, mpc_f, contact,
            base_pos_des=home_pos, base_vel_des=np.zeros(3),
            base_rpy_des=home_rpy, base_omega_des=np.zeros(3),
        )
        s = sim.step(tau)
        heights.append(s.base_pos[2])
        rolls.append(abs(np.degrees(s.base_rpy[0])))
        pitches.append(abs(np.degrees(s.base_rpy[1])))
    hz = np.array(heights)
    print(f"  [closed-loop 2s] final h={hz[-1]*100:.1f}cm  "
          f"drift={abs(hz[-1]-home_pos[2])*100:.2f}cm  "
          f"max|roll|={max(rolls):.2f}deg  max|pitch|={max(pitches):.2f}deg")
    check("stays standing (height 20-30cm)", 0.20 < hz[-1] < 0.30)
    check("small attitude drift (<5deg)", max(rolls) < 5.0 and max(pitches) < 5.0)

    # --- Test 3: one swing foot -> zero contact force there, EoM still exact ---
    sim.reset()
    contact_sw = np.array([False, True, True, False])  # FL, RR swing (trot)
    mpc_f2 = np.zeros((4, 3))
    mpc_f2[[1, 2], 2] = Wg / 2.0  # FR, RL carry weight
    swing_pos = st.foot_pos.copy()
    swing_pos[[0, 3], 2] += 0.05  # ask FL,RR feet to lift 5cm
    tau, info = wbc.solve(
        sim.data, mpc_f2, contact_sw,
        base_pos_des=home_pos, base_vel_des=np.zeros(3),
        base_rpy_des=home_rpy, base_omega_des=np.zeros(3),
        swing_pos_ref=swing_pos, swing_vel_ref=np.zeros((4, 3)),
        swing_acc_ref=np.zeros((4, 3)),
    )
    check("solver optimal (swing)", info["status_val"] == 1)
    check("EoM residual ~ 0 (swing)", info["eom_residual"] < 1e-4)  # within OSQP tol
    print(f"  [swing] eom_res={info['eom_residual']:.2e}  |tau|max={np.abs(tau).max():.2f}Nm")

    print(f"RESULT: {'PASS' if ok else 'FAIL'}")
    if not ok:
        sys.exit(1)
    return ok


if __name__ == "__main__":
    _self_test()
