"""Convex Model-Predictive Control for the Go2 trunk (SRBD).

Phase 2 step 2 (see PROGRESS.md). This is the MIT Cheetah 3 "Convex MPC"
(Di Carlo et al. 2018): the robot is approximated as a *single rigid body*
(SRBD) -- a floating trunk of fixed mass/inertia acted on by ground reaction
forces at the (scheduled) stance feet. The controller solves, over a short
horizon, for the foot forces that best track a desired base trajectory subject
to friction-cone and unilateral (push-only) contact constraints. Those forces
are handed to the WBC (step 3), which turns them into joint torques.

Model
-----
State (13):  x = [theta(3), p(3), omega(3), v(3), g]
  theta  -- base orientation as roll/pitch/yaw [rad]
  p      -- CoM position (world) [m]
  omega  -- base angular velocity (world) [rad/s]
  v      -- CoM linear velocity (world) [m/s]
  g      -- gravity augmentation state (constant -9.81), makes the affine
            dynamics linear so it fits a QP.

Continuous linearized dynamics (small roll/pitch, yaw = psi):
  d/dt theta = Rz(psi)^T omega
  d/dt p     = v
  d/dt omega = I_world^-1 * sum_i (r_i x f_i)
  d/dt v     = (1/m) sum_i f_i + g_vec
with I_world = Rz Ibody Rz^T and r_i = (foot_i - CoM). Euler-discretized to
Ad = I + A*dt, Bd = B*dt and condensed into a dense QP over the horizon.

Decision variables: stacked foot forces U = [u_0..u_{N-1}], u_k in R^{3*n_legs}.
Cost: sum_k (x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k.
Constraints (per foot, per step): linearized friction pyramid |f_xy| <= mu*f_z
and 0 <= f_z <= f_max where f_max = 0 for a swing (non-contact) foot -> its
force is driven to zero.

Solved with OSQP (sparse, warm-started across ticks).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import osqp
import scipy.sparse as sp

N_LEGS = 4
N_STATE = 13
N_U_PER_LEG = 3

GRAVITY = 9.81


def _rz(psi: float) -> np.ndarray:
    """Rotation about the world z-axis by yaw ``psi`` (body->world)."""
    c, s = np.cos(psi), np.sin(psi)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _skew(v: np.ndarray) -> np.ndarray:
    """Skew-symmetric matrix so that skew(a) @ b == a x b."""
    x, y, z = v
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


@dataclass
class MPCWeights:
    """Diagonal state-tracking weights ``Q`` (len 13) and force weight ``R``."""

    # theta(3), p(3), omega(3), v(3), g
    q_state: tuple[float, ...] = (
        20.0, 20.0, 5.0,      # roll, pitch, yaw  -- keep trunk level
        1.0, 1.0, 50.0,       # x, y, z position  -- height matters most
        0.5, 0.5, 0.5,        # angular velocity
        1.0, 1.0, 1.0,        # linear velocity
        0.0,                  # gravity aug (not tracked)
    )
    r_force: float = 1e-4     # penalize/regularize foot forces


class ConvexMPC:
    """SRBD convex MPC returning ground reaction forces for the stance feet.

    Parameters
    ----------
    mass:
        Total robot mass [kg].
    inertia:
        (3, 3) body-frame rotational inertia of the whole robot about its CoM
        (locked/composite inertia at a nominal pose). Use
        :func:`composite_inertia_from_model`.
    dt:
        MPC discretization timestep [s] (e.g. 0.03).
    horizon:
        Number of prediction steps ``N`` (e.g. 10).
    mu:
        Coulomb friction coefficient for the linearized pyramid.
    f_max:
        Per-foot normal force upper bound [N] when in contact.
    f_min:
        Per-foot normal force lower bound [N] when in contact (keeps stance
        feet loaded; 0 allows lift-off within the horizon).
    weights:
        :class:`MPCWeights` cost tuning.
    """

    def __init__(
        self,
        mass: float,
        inertia: np.ndarray,
        dt: float = 0.03,
        horizon: int = 10,
        mu: float = 0.6,
        f_max: float = 250.0,
        f_min: float = 1.0,
        weights: MPCWeights | None = None,
    ) -> None:
        self.mass = float(mass)
        self.inertia = np.asarray(inertia, dtype=float).reshape(3, 3)
        self.dt = float(dt)
        self.N = int(horizon)
        self.mu = float(mu)
        self.f_max = float(f_max)
        self.f_min = float(f_min)
        self.w = weights or MPCWeights()

        self.nu = N_LEGS * N_U_PER_LEG            # 12 forces per step
        self.n_dec = self.nu * self.N             # total decision vars

        # Constant cost blocks.
        q_state = np.asarray(self.w.q_state, dtype=float)
        self._Qbar = np.tile(q_state, self.N)                        # (13N,)
        self._Rbar = np.full(self.n_dec, self.w.r_force)             # (12N,)

        # Constant friction-pyramid block for one foot force [fx,fy,fz]:
        #   fx - mu fz <= 0 ;  -fx - mu fz <= 0 ; fy - mu fz <= 0 ; -fy - mu fz <= 0
        mu = self.mu
        self._cone = np.array(
            [
                [1.0, 0.0, -mu],
                [-1.0, 0.0, -mu],
                [0.0, 1.0, -mu],
                [0.0, -1.0, -mu],
                [0.0, 0.0, 1.0],   # normal-force bound row
            ]
        )
        self._rows_per_foot = self._cone.shape[0]  # 5

        self._prob: osqp.OSQP | None = None
        self._pattern: tuple | None = None
        self.last_info: dict = {}

    # ------------------------------------------------------------ dynamics
    def _discrete_dynamics(
        self, yaw: float, foot_rel: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        """Discrete Ad (13,13), Bd (13, 12) at the current yaw and foot geometry.

        ``foot_rel`` is (4, 3): stance foot positions relative to the CoM
        (world frame), held constant over the horizon.
        """
        A = np.zeros((N_STATE, N_STATE))
        Rz_T = _rz(yaw).T
        A[0:3, 6:9] = Rz_T          # d theta = Rz^T omega
        A[3:6, 9:12] = np.eye(3)    # d p = v
        A[11, 12] = 1.0             # d v_z += g (state g = -9.81)

        I_world = _rz(yaw) @ self.inertia @ _rz(yaw).T
        I_inv = np.linalg.inv(I_world)

        B = np.zeros((N_STATE, self.nu))
        for i in range(N_LEGS):
            r = foot_rel[i]
            B[6:9, 3 * i:3 * i + 3] = I_inv @ _skew(r)   # torque -> d omega
            B[9:12, 3 * i:3 * i + 3] = np.eye(3) / self.mass  # force -> d v
        Ad = np.eye(N_STATE) + A * self.dt
        Bd = B * self.dt
        return Ad, Bd

    def _condense(self, Ad: np.ndarray, Bd: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Build condensed Aqp (13N,13) and Bqp (13N, 12N) with constant Ad/Bd."""
        N = self.N
        Aqp = np.zeros((N_STATE * N, N_STATE))
        Bqp = np.zeros((N_STATE * N, self.nu * N))
        # Precompute powers Ad^k for k=1..N.
        powers = [np.eye(N_STATE)]
        for _ in range(N):
            powers.append(powers[-1] @ Ad)
        for k in range(N):                       # predicted step k -> x_{k+1}
            Aqp[N_STATE * k:N_STATE * (k + 1)] = powers[k + 1]
            for j in range(k + 1):               # input u_j影响 x_{k+1}
                blk = powers[k - j] @ Bd
                Bqp[N_STATE * k:N_STATE * (k + 1), self.nu * j:self.nu * (j + 1)] = blk
        return Aqp, Bqp

    # --------------------------------------------------------------- solve
    def solve(
        self,
        state13: np.ndarray,
        foot_pos_world: np.ndarray,
        com_pos: np.ndarray,
        contact_seq: np.ndarray,
        x_ref: np.ndarray,
    ) -> tuple[np.ndarray, dict]:
        """Solve the horizon QP and return the first-step foot forces.

        Parameters
        ----------
        state13:
            (13,) current SRBD state [theta, p, omega, v, g].
        foot_pos_world:
            (4, 3) current foot positions in the world (held over the horizon).
        com_pos:
            (3,) current CoM position (world) -- to form r_i = foot - CoM.
        contact_seq:
            (N, 4) bool stance schedule from ``GaitScheduler.contact_sequence``.
        x_ref:
            (N, 13) desired state at each predicted step.

        Returns
        -------
        (forces, info): forces is (4, 3) world-frame ground reaction forces for
        the *first* horizon step (swing feet are ~0). ``info`` has solver status.
        """
        state13 = np.asarray(state13, dtype=float).reshape(N_STATE)
        foot_pos_world = np.asarray(foot_pos_world, dtype=float).reshape(N_LEGS, 3)
        com_pos = np.asarray(com_pos, dtype=float).reshape(3)
        contact_seq = np.asarray(contact_seq, dtype=bool).reshape(self.N, N_LEGS)
        x_ref = np.asarray(x_ref, dtype=float).reshape(self.N, N_STATE)

        yaw = state13[2]
        foot_rel = foot_pos_world - com_pos
        Ad, Bd = self._discrete_dynamics(yaw, foot_rel)
        Aqp, Bqp = self._condense(Ad, Bd)

        # --- cost: 0.5 U^T P U + q^T U ---
        Qbar = self._Qbar
        P = Bqp.T @ (Qbar[:, None] * Bqp)
        P[np.diag_indices_from(P)] += self._Rbar
        P = 2.0 * P
        P[np.diag_indices_from(P)] += 1e-8       # numerical PD
        err = Aqp @ state13 - x_ref.reshape(-1)  # (Aqp x0 - Xref)
        q = 2.0 * (Bqp.T @ (Qbar * err))

        # --- constraints: friction pyramid + normal bounds per foot/step ---
        Acon, lb, ub = self._build_constraints(contact_seq)

        P_sp = sp.triu(sp.csc_matrix(P), format="csc")  # OSQP wants upper triangle
        A_sp = sp.csc_matrix(Acon)
        self._solve_osqp(P_sp, q, A_sp, lb, ub)

        U = self._solution
        u0 = U[: self.nu].reshape(N_LEGS, N_U_PER_LEG)
        info = dict(self.last_info)
        return u0, info

    def _build_constraints(
        self, contact_seq: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Stack per-foot friction/normal rows over the whole horizon."""
        rpf = self._rows_per_foot
        n_rows = self.N * N_LEGS * rpf
        Acon = np.zeros((n_rows, self.n_dec))
        lb = np.zeros(n_rows)
        ub = np.zeros(n_rows)
        BIG = 1e6
        row = 0
        for k in range(self.N):
            for i in range(N_LEGS):
                col = self.nu * k + N_U_PER_LEG * i
                Acon[row:row + rpf, col:col + N_U_PER_LEG] = self._cone
                in_contact = bool(contact_seq[k, i])
                fmax = self.f_max if in_contact else 0.0
                fmin = self.f_min if in_contact else 0.0
                # cone rows 0..3:  expr <= 0  -> [-BIG, 0]
                lb[row:row + 4] = -BIG
                ub[row:row + 4] = 0.0
                # normal row 4:  fmin <= fz <= fmax
                lb[row + 4] = fmin
                ub[row + 4] = fmax
                row += rpf
        return Acon, lb, ub

    def _solve_osqp(self, P_sp, q, A_sp, lb, ub) -> None:
        """Setup-or-update OSQP, keeping warm start when sparsity is unchanged."""
        pattern = (P_sp.indptr.tobytes(), P_sp.indices.tobytes(),
                   A_sp.indptr.tobytes(), A_sp.indices.tobytes())
        if self._prob is None or pattern != self._pattern:
            self._prob = osqp.OSQP()
            self._prob.setup(P=P_sp, q=q, A=A_sp, l=lb, u=ub,
                             verbose=False, warm_starting=True,
                             eps_abs=1e-5, eps_rel=1e-5, max_iter=4000)
            self._pattern = pattern
        else:
            self._prob.update(Px=P_sp.data, Ax=A_sp.data, q=q, l=lb, u=ub)
        res = self._prob.solve()
        status = res.info.status
        if res.x is None or (np.any(np.isnan(res.x))):
            # Fallback: gravity-compensating vertical forces on all feet.
            x = np.zeros(self.n_dec)
            x[2::3] = self.mass * GRAVITY / N_LEGS
            self._solution = x
        else:
            self._solution = res.x
        self.last_info = {
            "status": status,
            "status_val": int(res.info.status_val),
            "iter": int(res.info.iter),
            "obj": float(res.info.obj_val),
            "solve_time_ms": float(res.info.solve_time) * 1e3,
        }


# ---------------------------------------------------- reference generation
def make_reference(
    state13: np.ndarray,
    vel_cmd_world: np.ndarray,
    yaw_rate_cmd: float,
    height: float,
    horizon: int,
    dt: float,
) -> np.ndarray:
    """Build an (N, 13) desired-state trajectory from velocity commands.

    Trunk is kept level (roll=pitch=0) at the commanded height; position and
    yaw integrate the commanded velocities forward.
    """
    state13 = np.asarray(state13, dtype=float).reshape(N_STATE)
    v = np.asarray(vel_cmd_world, dtype=float).reshape(3).copy()
    v[2] = 0.0
    p0 = state13[3:6].copy()
    yaw0 = state13[2]
    ref = np.zeros((horizon, N_STATE))
    for k in range(horizon):
        t = (k + 1) * dt
        ref[k, 0] = 0.0                     # roll
        ref[k, 1] = 0.0                     # pitch
        ref[k, 2] = yaw0 + yaw_rate_cmd * t  # yaw
        ref[k, 3] = p0[0] + v[0] * t        # x
        ref[k, 4] = p0[1] + v[1] * t        # y
        ref[k, 5] = height                  # z
        ref[k, 6:9] = (0.0, 0.0, yaw_rate_cmd)
        ref[k, 9:12] = v
        ref[k, 12] = -GRAVITY
    return ref


def composite_inertia_from_model(model, data) -> tuple[float, np.ndarray, np.ndarray]:
    """Whole-body mass, CoM (world) and composite inertia about the CoM.

    Computes the locked/rigid inertia of the robot in its *current* configuration
    by summing each body's inertia (rotated to world) with a parallel-axis term.
    Call after ``mj_forward`` at the nominal (home) pose.

    Returns ``(total_mass, com_world, inertia_3x3_body)``. Since we ignore yaw
    here (home pose has identity yaw), the returned inertia is in the world frame,
    which coincides with the body frame at the home pose.
    """
    import mujoco  # local import; sim-only dependency

    m, d = model, data
    masses = m.body_mass.copy()
    total = float(masses.sum())
    com = (masses[:, None] * d.xipos).sum(axis=0) / total  # (3,)

    I = np.zeros((3, 3))
    for b in range(m.nbody):
        mb = float(masses[b])
        if mb <= 0.0:
            continue
        R = d.ximat[b].reshape(3, 3)                 # body inertial frame -> world
        Ib = np.diag(m.body_inertia[b])              # principal inertia
        I_world = R @ Ib @ R.T
        dvec = d.xipos[b] - com
        I += I_world + mb * (float(dvec @ dvec) * np.eye(3) - np.outer(dvec, dvec))
    return total, com, I


# ------------------------------------------------------------------ self-test
def _self_test() -> bool:
    import sys
    from src.sim_env import Go2Sim

    ok = True

    def check(name: str, cond: bool) -> None:
        nonlocal ok
        ok = ok and bool(cond)
        print(f"  [{'PASS' if cond else 'FAIL'}] {name}")

    print("=== mpc.py self-test ===")
    sim = Go2Sim()
    st = sim.reset()
    mass, com, inertia = composite_inertia_from_model(sim.model, sim.data)
    print(f"  model: mass={mass:.3f} kg  com={np.round(com,3).tolist()}  "
          f"Idiag={np.round(np.diag(inertia),4).tolist()}")
    Wg = mass * GRAVITY
    print(f"  weight m*g = {Wg:.1f} N")

    dt, N = 0.03, 10
    mpc = ConvexMPC(mass, inertia, dt=dt, horizon=N, mu=0.6)

    # --- current SRBD state at home stand ---
    x0 = np.zeros(13)
    x0[0:3] = st.base_rpy
    x0[3:6] = st.base_pos
    x0[6:9] = st.base_ang_vel
    x0[9:12] = st.base_lin_vel
    x0[12] = -GRAVITY
    feet = st.foot_pos.copy()
    height = st.base_pos[2]

    # === Test 1: 4-foot stance, zero velocity command -> hold still ===
    from src.gait import GaitScheduler
    stance_all = np.ones((N, 4), dtype=bool)
    ref = make_reference(x0, np.zeros(3), 0.0, height, N, dt)
    u0, info = mpc.solve(x0, feet, st.base_pos, stance_all, ref)
    fz = u0[:, 2]
    print(f"  [stand] status={info['status']} iter={info['iter']} "
          f"t={info['solve_time_ms']:.2f}ms  fz={np.round(fz,1).tolist()}  "
          f"sum_fz={fz.sum():.1f}N")
    check("solver optimal (stand)", info["status_val"] == 1)
    check(f"sum vertical force ~ m*g (got {fz.sum():.1f}, want {Wg:.1f})",
          abs(fz.sum() - Wg) < 0.05 * Wg)
    check("all normal forces >= 0", bool(np.all(fz > -1e-6)))
    check("friction cone satisfied (|f_xy| <= mu fz)",
          bool(np.all(np.abs(u0[:, :2]) <= mpc.mu * fz[:, None] + 1e-3)))
    # By symmetry the front/rear split loads the CoM; just check near-even L/R.
    check("left/right forces balanced",
          abs((fz[0] + fz[2]) - (fz[1] + fz[3])) < 0.05 * Wg)

    # === Test 2: swing feet get zero force ===
    contact = np.tile(np.array([True, False, False, True]), (N, 1))  # FL,RR stance
    ref = make_reference(x0, np.zeros(3), 0.0, height, N, dt)
    u0, info = mpc.solve(x0, feet, st.base_pos, contact, ref)
    swing_force = np.linalg.norm(u0[[1, 2]], axis=1)
    stance_fz = u0[[0, 3], 2]
    print(f"  [trot] stance fz={np.round(stance_fz,1).tolist()} "
          f"swing |f|={np.round(swing_force,3).tolist()}  sum_fz={u0[:,2].sum():.1f}N")
    check("swing feet force ~ 0", bool(np.all(swing_force < 1e-2)))
    check("2-leg stance still supports m*g",
          abs(u0[:, 2].sum() - Wg) < 0.08 * Wg)

    # === Test 3: forward velocity command -> net forward force ===
    v_cmd = np.array([0.6, 0.0, 0.0])
    ref = make_reference(x0, v_cmd, 0.0, height, N, dt)
    u0, info = mpc.solve(x0, feet, st.base_pos, stance_all, ref)
    net_fx = u0[:, 0].sum()
    print(f"  [accel] net_fx={net_fx:.2f}N (should be >0 to accelerate forward)")
    check("forward cmd -> net forward force", net_fx > 1.0)

    # === Test 4: warm-start path (repeat solve reuses OSQP instance) ===
    prob_before = mpc._prob
    u0b, infob = mpc.solve(x0, feet, st.base_pos, stance_all, ref)
    check("warm-start reuses solver instance (no re-setup)",
          mpc._prob is prob_before)
    check("warm-started solve stays optimal & physically identical",
          infob["status_val"] == 1 and np.allclose(u0, u0b, atol=0.5))
    print(f"  warm-started solve time = {infob['solve_time_ms']:.2f} ms, "
          f"max force diff = {np.max(np.abs(u0 - u0b)):.3f} N")

    print(f"RESULT: {'PASS' if ok else 'FAIL'}")
    if not ok:
        sys.exit(1)
    return ok


if __name__ == "__main__":
    _self_test()
