# Eval-only env configs for the Phase 4 RL-vs-classical-control comparison.
#
# All three conditions inherit from the *Rough* Go2 task cfg (not the separate Flat task cfg)
# so the observation/action schema exactly matches the checkpoint trained on
# Isaac-Velocity-Rough-UnitreeGo2 -- only the ground geometry changes between conditions.
# This mirrors the Phase 3 MuJoCo protocol: same controller, terrain swapped underneath it.
import math

from isaaclab.envs.mdp.events import reset_joints_by_offset
from isaaclab.terrains import HfPyramidSlopedTerrainCfg, HfRandomUniformTerrainCfg, TerrainGeneratorCfg
from isaaclab.utils import configclass

from isaaclab_tasks.core.velocity.config.go2.rough_env_cfg import UnitreeGo2RoughEnvCfg

SLOPE_RAD_5DEG = math.radians(5.0)
ROUGH_NOISE_M = 0.03


def _single_subterrain_generator(sub_terrain):
    return TerrainGeneratorCfg(
        size=(8.0, 8.0),
        border_width=20.0,
        num_rows=2,
        num_cols=2,
        horizontal_scale=0.1,
        vertical_scale=0.005,
        slope_threshold=0.75,
        use_cache=False,
        curriculum=False,
        sub_terrains={"only": sub_terrain},
    )


@configclass
class Phase4EvalEnvCfg(UnitreeGo2RoughEnvCfg):
    """Shared eval-mode overrides: fixed forward-velocity command, joint-noise-only reset
    (matching Go2Sim.reset(noise_amplitude=...) in the MuJoCo project), no continuous
    pushes/random spawn -- isolates the same single variable Phase 3 swept."""

    def __post_init__(self):
        super().__post_init__()
        self.play_mode()

        # fixed command (default: overridden per-instance below)
        self.commands.base_velocity.rel_standing_envs = 0.0
        self.commands.base_velocity.heading_command = False
        self.commands.base_velocity.resampling_time_range = (1.0e9, 1.0e9)

        # deterministic spawn: no random base pose/velocity jitter
        self.events.reset_base.params["pose_range"] = {"x": (0.0, 0.0), "y": (0.0, 0.0), "yaw": (0.0, 0.0)}
        self.events.reset_base.params["velocity_range"] = {
            "x": (0.0, 0.0), "y": (0.0, 0.0), "z": (0.0, 0.0),
            "roll": (0.0, 0.0), "pitch": (0.0, 0.0), "yaw": (0.0, 0.0),
        }
        self.events.base_external_force_torque = None

        # additive joint-angle noise sweep, not the training-time multiplicative scale reset
        self.events.reset_robot_joints.func = reset_joints_by_offset
        self.events.reset_robot_joints.params = {"position_range": (0.0, 0.0), "velocity_range": (0.0, 0.0)}

    def set_command(self, vx: float):
        r = self.commands.base_velocity.ranges
        r.lin_vel_x = (vx, vx)
        r.lin_vel_y = (0.0, 0.0)
        r.ang_vel_z = (0.0, 0.0)
        r.heading = (0.0, 0.0)

    def set_noise_amplitude(self, amp: float):
        self.events.reset_robot_joints.params["position_range"] = (-amp, amp)


@configclass
class Phase4EvalFlatCfg(Phase4EvalEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_type = "plane"
        self.scene.terrain.terrain_generator = None
        self.curriculum.terrain_levels = None


@configclass
class Phase4EvalSlopeCfg(Phase4EvalEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.set_slope(SLOPE_RAD_5DEG)
        self.curriculum.terrain_levels = None

    def set_slope(self, angle_rad: float):
        self.scene.terrain.terrain_generator = _single_subterrain_generator(
            HfPyramidSlopedTerrainCfg(
                proportion=1.0, slope_range=(angle_rad, angle_rad), platform_width=0.5, border_width=0.25
            )
        )


@configclass
class Phase4EvalRoughCfg(Phase4EvalEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.scene.terrain.terrain_generator = _single_subterrain_generator(
            HfRandomUniformTerrainCfg(
                proportion=1.0, noise_range=(ROUGH_NOISE_M, ROUGH_NOISE_M), noise_step=0.005, border_width=0.25
            )
        )
        self.curriculum.terrain_levels = None


TERRAIN_CFGS = {
    "flat": Phase4EvalFlatCfg,
    "slope_5deg": Phase4EvalSlopeCfg,
    "rough": Phase4EvalRoughCfg,
}
