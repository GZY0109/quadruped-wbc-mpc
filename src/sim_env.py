"""MuJoCo simulation environment for the Unitree Go2 quadruped.

Phase 1 deliverable (see PROJECT_PLAN.md): a thin, well-documented wrapper around
the MuJoCo model that exposes exactly the signals a real quadruped controller
consumes and produces:

  * state read-out  -> base pose/twist, joint pos/vel, IMU, foot contacts/forces
  * command down    -> 12 joint torques (direct-drive motors)

The Convex-MPC (base reaction forces) and WBC (joint-torque QP) layers in Phase 2
are built on top of this interface, so it deliberately stays controller-agnostic:
no gait logic, no reference generation, just sim <-> control plumbing.

Conventions
-----------
Leg order is ``[FL, FR, RL, RR]`` everywhere (front-left, front-right, rear-left,
rear-right). Each leg has 3 joints in order ``[hip(abduction), thigh, calf(knee)]``,
so all 12-vectors are laid out ``[FL_hip, FL_thigh, FL_calf, FR_hip, ...]`` which
matches the MJCF actuator/joint order in ``go2.xml``.

Quaternions are MuJoCo/Hamilton convention, scalar-first ``(w, x, y, z)``.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

# Default to software (OSMesa) GL so offscreen rendering works on headless GPU
# pods where EGL device access is restricted. Set MUJOCO_GL before importing
# mujoco; callers can override by exporting MUJOCO_GL=egl/glfw themselves.
os.environ.setdefault("MUJOCO_GL", "osmesa")

import mujoco
import numpy as np

# Repo layout: this file is <repo>/src/sim_env.py
_REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = _REPO_ROOT / "assets" / "unitree_go2" / "go2_scene.xml"

LEGS = ("FL", "FR", "RL", "RR")
JOINTS_PER_LEG = ("hip", "thigh", "calf")
JOINT_NAMES = tuple(f"{leg}_{j}_joint" for leg in LEGS for j in JOINTS_PER_LEG)
ACTUATOR_NAMES = tuple(f"{leg}_{j}" for leg in LEGS for j in JOINTS_PER_LEG)
FOOT_SITES = tuple(f"{leg}_foot" for leg in LEGS)
FOOT_GEOMS = LEGS  # foot collision geoms are named FL/FR/RL/RR


@dataclass
class RobotState:
    """Snapshot of everything a controller might read on a given control tick.

    All arrays are copies (safe to store across steps). Base quantities marked
    ``*_world`` are ground-truth from the simulator; ``imu_*`` are the noisier
    on-board signals a physical Go2 actually exposes.
    """

    time: float
    # Base (floating trunk), ground-truth
    base_pos: np.ndarray          # (3,) world position of trunk origin
    base_quat: np.ndarray         # (4,) world orientation, (w, x, y, z)
    base_rpy: np.ndarray          # (3,) roll, pitch, yaw [rad]
    base_lin_vel: np.ndarray      # (3,) world-frame linear velocity
    base_ang_vel: np.ndarray      # (3,) world-frame angular velocity
    # Joints
    joint_pos: np.ndarray         # (12,)
    joint_vel: np.ndarray         # (12,)
    # Feet
    foot_pos: np.ndarray          # (4, 3) world positions
    foot_contact: np.ndarray      # (4,) bool, in contact with ground
    foot_force: np.ndarray        # (4,) normal contact force [N]
    # On-board IMU (trunk site)
    imu_quat: np.ndarray          # (4,) orientation (w, x, y, z)
    imu_gyro: np.ndarray          # (3,) body-frame angular velocity
    imu_acc: np.ndarray           # (3,) body-frame linear acceleration (incl. gravity)


class Go2Sim:
    """MuJoCo-backed Unitree Go2 environment with a torque-control interface.

    Parameters
    ----------
    scene_path:
        Path to the MJCF scene. Defaults to the vendored flat-ground Go2 scene.
    control_dt:
        Duration of one ``step()`` call in seconds. The underlying physics runs
        at the model timestep (``0.002 s`` -> 500 Hz); each control tick advances
        ``round(control_dt / timestep)`` physics substeps with the commanded
        torque held constant. Default ``0.002`` means 1:1 (control == physics).
        Phase 2 typically uses a lower WBC/MPC rate, e.g. ``control_dt=0.005``.
    contact_force_threshold:
        Normal force [N] above which a foot is reported as "in contact".
    """

    def __init__(
        self,
        scene_path: str | Path = DEFAULT_SCENE,
        control_dt: float = 0.002,
        contact_force_threshold: float = 1.0,
    ) -> None:
        scene_path = Path(scene_path)
        if not scene_path.exists():
            raise FileNotFoundError(f"Scene file not found: {scene_path}")
        self.model = mujoco.MjModel.from_xml_path(str(scene_path))
        self.data = mujoco.MjData(self.model)

        self.dt = float(self.model.opt.timestep)
        self.control_dt = float(control_dt)
        self.n_substeps = max(1, round(self.control_dt / self.dt))
        self.contact_force_threshold = float(contact_force_threshold)

        self._cache_indices()

        # Torque limits from the actuator ctrlrange (motors are direct torque).
        self.torque_limits = self.model.actuator_ctrlrange.copy()  # (12, 2)

        self._renderer: mujoco.Renderer | None = None

        self.reset()

    # ------------------------------------------------------------------ setup
    def _cache_indices(self) -> None:
        m = self.model

        def _id(objtype, name):
            i = mujoco.mj_name2id(m, objtype, name)
            if i < 0:
                raise KeyError(f"{objtype} '{name}' not found in model")
            return i

        self.actuator_ids = np.array(
            [_id(mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ACTUATOR_NAMES]
        )
        self.joint_ids = np.array(
            [_id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in JOINT_NAMES]
        )
        # qpos/qvel addresses for the 12 actuated joints (skip the 7-dof freejoint).
        self.joint_qpos_adr = np.array([m.jnt_qposadr[j] for j in self.joint_ids])
        self.joint_qvel_adr = np.array([m.jnt_dofadr[j] for j in self.joint_ids])

        self.foot_site_ids = np.array(
            [_id(mujoco.mjtObj.mjOBJ_SITE, n) for n in FOOT_SITES]
        )
        self.foot_geom_ids = np.array(
            [_id(mujoco.mjtObj.mjOBJ_GEOM, n) for n in FOOT_GEOMS]
        )
        self.imu_site_id = _id(mujoco.mjtObj.mjOBJ_SITE, "imu")

        # Sensor address lookup (name -> slice into data.sensordata).
        self._sensor_adr: dict[str, tuple[int, int]] = {}
        for i in range(m.nsensor):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i)
            adr, dim = int(m.sensor_adr[i]), int(m.sensor_dim[i])
            self._sensor_adr[name] = (adr, dim)

        self.home_key_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, "home")

    def _sensor(self, name: str) -> np.ndarray:
        adr, dim = self._sensor_adr[name]
        return self.data.sensordata[adr : adr + dim].copy()

    # --------------------------------------------------------------- lifecycle
    def reset(
        self,
        keyframe: str | None = "home",
        add_noise: bool = False,
        seed: int | None = None,
        noise_amplitude: float = 0.05,
        base_pitch: float = 0.0,
        z_offset: float = 0.0,
    ) -> RobotState:
        """Reset to a keyframe (default the ``home`` standing pose) and return state.

        ``seed`` draws the init-noise perturbation from a local RNG instead of
        the global ``np.random`` state, so repeated trials with the same seed
        reproduce exactly (used for multi-trial statistics across a marginally
        stable controller, where floating-point-level differences between
        environment/library versions have been observed to shift outcomes by
        seconds -- see PROGRESS.md). ``noise_amplitude`` is the uniform
        per-joint perturbation range [rad] (default 0.05 matches the value
        that was previously hardcoded here, so existing callers/benchmarks
        are unaffected); Phase 3's disturbance-amplitude sweep varies it.

        ``base_pitch``/``z_offset`` let a caller spawn the (still flat-ground
        ``home`` keyframe) pose pre-tilted/raised -- used by Phase 3's terrain
        eval to seat the robot on a sloped floor without the naive flat-pose
        reset leaving two feet dangling in the air (the slope's ground drops
        below the flat-keyframe assumption on the downhill side). Joint
        angles are left at their home values; only the base orientation/
        height are adjusted, then physics + controller settle from there.
        """
        if keyframe is not None and self.home_key_id >= 0:
            kid = (
                self.home_key_id
                if keyframe == "home"
                else mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
            )
            mujoco.mj_resetDataKeyframe(self.model, self.data, kid)
        else:
            mujoco.mj_resetData(self.model, self.data)
        if base_pitch != 0.0:
            # pure y-axis rotation quaternion (w,x,y,z), composed onto the
            # keyframe's identity base orientation.
            half = base_pitch / 2.0
            self.data.qpos[3:7] = np.array([np.cos(half), 0.0, np.sin(half), 0.0])
        if z_offset != 0.0:
            self.data.qpos[2] += z_offset
        if add_noise and noise_amplitude > 0:
            rng = np.random.default_rng(seed) if seed is not None else np.random
            self.data.qpos[self.joint_qpos_adr] += rng.uniform(
                -noise_amplitude, noise_amplitude, size=12
            )
        mujoco.mj_forward(self.model, self.data)
        return self.get_state()

    def set_state(self, qpos: np.ndarray, qvel: np.ndarray) -> None:
        """Directly set full generalized coordinates (used by experiment resets)."""
        self.data.qpos[:] = qpos
        self.data.qvel[:] = qvel
        mujoco.mj_forward(self.model, self.data)

    # ------------------------------------------------------------------- step
    def step(self, torques: np.ndarray) -> RobotState:
        """Apply 12 joint torques (Nm) and advance one control tick.

        Torques are clipped to the actuator limits and held constant across the
        physics substeps. Returns the post-step :class:`RobotState`.
        """
        tau = np.asarray(torques, dtype=float).reshape(12)
        tau = np.clip(tau, self.torque_limits[:, 0], self.torque_limits[:, 1])
        self.data.ctrl[self.actuator_ids] = tau
        for _ in range(self.n_substeps):
            mujoco.mj_step(self.model, self.data)
        return self.get_state()

    def apply_external_wrench(
        self, force: np.ndarray = (0, 0, 0), torque: np.ndarray = (0, 0, 0)
    ) -> None:
        """Set a persistent external wrench on the trunk (world frame).

        Used in Phase 3 to inject push disturbances. Call with zeros to clear.
        The wrench stays applied until overwritten (MuJoCo clears xfrc_applied
        only on reset), so callers manage the pulse duration.
        """
        base_body = self.model.body("base").id
        self.data.xfrc_applied[base_body, :3] = np.asarray(force, dtype=float)
        self.data.xfrc_applied[base_body, 3:] = np.asarray(torque, dtype=float)

    # ------------------------------------------------------------------ state
    def get_state(self) -> RobotState:
        d = self.data
        base_quat = d.qpos[3:7].copy()
        return RobotState(
            time=float(d.time),
            base_pos=d.qpos[0:3].copy(),
            base_quat=base_quat,
            base_rpy=quat_to_rpy(base_quat),
            base_lin_vel=self._sensor("base_linvel"),
            base_ang_vel=self._sensor("base_angvel"),
            joint_pos=d.qpos[self.joint_qpos_adr].copy(),
            joint_vel=d.qvel[self.joint_qvel_adr].copy(),
            foot_pos=np.stack([self._sensor(f"{leg}_pos") for leg in LEGS]),
            foot_contact=self.foot_contacts(),
            foot_force=self.foot_forces(),
            imu_quat=self._sensor("imu_quat"),
            imu_gyro=self._sensor("imu_gyro"),
            imu_acc=self._sensor("imu_acc"),
        )

    def foot_forces(self) -> np.ndarray:
        """(4,) foot normal contact forces [N] from the touch sensors."""
        return np.array([self._sensor(f"{leg}_touch")[0] for leg in LEGS])

    def foot_contacts(self) -> np.ndarray:
        """(4,) bool contact flags (touch force above threshold)."""
        return self.foot_forces() > self.contact_force_threshold

    # ----------------------------------------------------------------- render
    def render(self, width: int = 640, height: int = 480, camera: str | int = -1) -> np.ndarray:
        """Return an RGB frame (H, W, 3) uint8 via an offscreen renderer."""
        if self._renderer is None or (
            self._renderer.height != height or self._renderer.width != width
        ):
            if self._renderer is not None:
                self._renderer.close()
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
        self._renderer.update_scene(self.data, camera=camera)
        return self._renderer.render()

    def close(self) -> None:
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None


# ---------------------------------------------------------------- math utils
def quat_to_rpy(quat: np.ndarray) -> np.ndarray:
    """Scalar-first quaternion (w, x, y, z) -> roll, pitch, yaw (XYZ) [rad]."""
    w, x, y, z = quat
    # roll (x-axis)
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    # pitch (y-axis), clamped for numerical safety near +/-90 deg
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))
    # yaw (z-axis)
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def quat_to_rotmat(quat: np.ndarray) -> np.ndarray:
    """Scalar-first quaternion (w, x, y, z) -> 3x3 rotation matrix (body->world)."""
    mat = np.zeros(9)
    mujoco.mju_quat2Mat(mat, np.asarray(quat, dtype=float))
    return mat.reshape(3, 3)
