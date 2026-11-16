# © 2025 ETH Zurich, Robotic Systems Lab
# Licensed under the Apache License 2.0

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.utils import configclass

from pace_sim2real.utils import PaceDCMotorCfg, project_root
from pace_sim2real import PaceSim2realEnvCfg, PaceSim2realSceneCfg, PaceCfg

# Generated from the Duatic description with scripts/dynaarm/build_usd.py
_USD_PATH = str(
    project_root() / "source" / "pace_sim2real" / "pace_sim2real" / "assets" / "dynaarm" / "usd" / "dynaarm_baracuda12.usd"
)

# Joint velocity limits from Duatic/duatic_dynaarm (duatic_dynaarm_description, baracuda12)
_VELOCITY_LIMIT = {
    "shoulder_rotation": 10.64,
    "shoulder_flexion": 10.64,
    "elbow_flexion": 10.64,
    "forearm_rotation": 13.647,
    "wrist_flexion": 13.647,
    "wrist_rotation": 13.647,
}

# Gains and limits match the recording setup: ros2_control pid_controller at 1 kHz on the effort interface,
# without feed-forward or gravity compensation, and a 40 Nm torque limit on all joints in the driver
DYNAARM_PACE_ACTUATOR_CFG = PaceDCMotorCfg(
    joint_names_expr=[".*"],
    saturation_effort=120.0,  # peak torque of the shoulder and elbow drives (Duatic)
    effort_limit=40.0,
    velocity_limit=_VELOCITY_LIMIT,
    # set explicitly, otherwise the PhysX joint velocity limit is taken from the USD
    velocity_limit_sim=_VELOCITY_LIMIT,
    stiffness={
        "shoulder_rotation": 80.0,
        "shoulder_flexion": 80.0,
        "elbow_flexion": 80.0,
        "forearm_rotation": 60.0,
        "wrist_flexion": 60.0,
        "wrist_rotation": 60.0,
    },
    damping={
        "shoulder_rotation": 2.0,
        "shoulder_flexion": 2.0,
        "elbow_flexion": 2.0,
        "forearm_rotation": 1.0,
        "wrist_flexion": 1.0,
        "wrist_rotation": 1.0,
    },
    encoder_bias={".*": 0.0},  # encoder bias in radians
    friction={".*": 0.0},  # static friction coefficient (Nm)
    dynamic_friction={".*": 0.0},  # dynamic friction coefficient (Nm)
    viscous_friction={".*": 0.0},  # viscous friction coefficient (Nm s/rad)
    max_delay=10,  # max delay in simulation steps
)

DYNAARM_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path=_USD_PATH,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=16,
            solver_velocity_iteration_count=1,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),  # mounted upright on a table top
        joint_pos={  # home position of the chirp recordings
            "shoulder_rotation": 0.0,
            "shoulder_flexion": -0.5,
            "elbow_flexion": 1.6,
            "forearm_rotation": 0.0,
            "wrist_flexion": 0.0,
            "wrist_rotation": 0.0,
        },
    ),
    actuators={"joints": DYNAARM_PACE_ACTUATOR_CFG},
    soft_joint_pos_limit_factor=0.95,
)


@configclass
class DynaArmPaceCfg(PaceCfg):
    """Pace configuration for DynaArm robot."""
    robot_name: str = "dynaarm"
    data_dir: str = "dynaarm/chirp_data.pt"  # located in pace_sim2real/data/dynaarm/chirp_data.pt
    bounds_params: torch.Tensor = torch.zeros((25, 2))  # 6 + 6 + 6 + 6 + 1 = 25 parameters to optimize
    joint_order: list[str] = [
        "shoulder_rotation",
        "shoulder_flexion",
        "elbow_flexion",
        "forearm_rotation",
        "wrist_flexion",
        "wrist_rotation",
    ]

    def __post_init__(self):
        # set bounds for parameters
        # lower bound keeps the explicit PD stable at 400Hz, the wrist rotation chatters at its velocity limit with
        # armature <= 1e-3 kgm2 (tested by replaying the chirp data)
        self.bounds_params[:6, 0] = 2e-3
        self.bounds_params[:6, 1] = 1.0  # armature between 2e-3 - 1.0 [kgm2]
        self.bounds_params[6:12, 1] = 7.0  # dof_damping between 0.0 - 7.0 [Nm s/rad]
        self.bounds_params[12:18, 1] = 5.0  # friction between 0.0 - 5.0 [Nm]
        self.bounds_params[18:24, 0] = -0.1
        self.bounds_params[18:24, 1] = 0.1  # bias between -0.1 - 0.1 [rad]
        self.bounds_params[24, 1] = 10.0  # delay between 0.0 - 10.0 [sim steps]


@configclass
class DynaArmPaceSceneCfg(PaceSim2realSceneCfg):
    """Configuration for DynaArm robot in Pace Sim2Real environment."""
    robot: ArticulationCfg = DYNAARM_CFG


@configclass
class DynaArmPaceEnvCfg(PaceSim2realEnvCfg):

    scene: DynaArmPaceSceneCfg = DynaArmPaceSceneCfg()
    sim2real: PaceCfg = DynaArmPaceCfg()

    def __post_init__(self):
        # post init of parent
        super().__post_init__()

        # robot sim and control settings, must match the sampling rate of the recorded data (chirp_data.pt at 400Hz)
        self.sim.dt = 0.0025  # 400Hz simulation
        self.decimation = 1  # 400Hz control
