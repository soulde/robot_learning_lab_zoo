"""Isaac Lab articulation configurations for the Deep Robotics DR02 robots."""

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg

from soulde_robot_zoo import DR02_PRO_URDF, DR02_STANDARD_URDF


_USD_CACHE_DIR = os.path.realpath(os.path.expanduser(os.environ.get("TMPDIR", "/tmp")))


def _motor(
    joint_names_expr: list[str],
    effort: float,
    velocity: float,
) -> DCMotorCfg:
    return DCMotorCfg(
        joint_names_expr=joint_names_expr,
        effort_limit=effort,
        saturation_effort=effort,
        velocity_limit=velocity,
        stiffness=625.0,
        damping=0.25,
        friction=0.0,
    )


DR02_STANDARD_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        asset_path=str(DR02_STANDARD_URDF),
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=1.0,
        ),
        usd_dir=os.path.join(_USD_CACHE_DIR, "IsaacLab", "dr02_standard"),
        usd_file_name="dr02_std.usd",
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.95),
        joint_pos={
            ".*hip.*": 0.0,
            ".*knee_joint": 0.0,
            ".*ankle.*": 0.0,
            ".*shoulder.*": 0.0,
            ".*elbow_joint": 0.0,
            "waist_z_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "leg_pitch_roll_knee": _motor([".*_hip_[xy]_joint", ".*_knee_joint"], 363.0, 20.0),
        "hip_yaw": _motor([".*_hip_z_joint"], 137.0, 19.38),
        "ankle_pitch": _motor([".*_ankle_y_joint"], 137.0, 19.38),
        "ankle_roll": _motor([".*_ankle_x_joint"], 50.0, 23.76),
        "waist_arms": _motor(["waist_z_joint", ".*_shoulder_[xyz]_joint", ".*_elbow_joint"], 137.0, 19.38),
    },
)


DR02_PRO_CFG = ArticulationCfg(
    spawn=DR02_STANDARD_CFG.spawn.replace(
        asset_path=str(DR02_PRO_URDF),
        usd_dir=os.path.join(_USD_CACHE_DIR, "IsaacLab", "dr02_pro"),
        usd_file_name="dr02_pro.usd",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.95),
        joint_pos={
            "^(?!.*(?:_elbow_joint|_shoulder_z_joint)$).*$": 0.0,
            "left_shoulder_z_joint": 0.765,
            "right_shoulder_z_joint": -0.765,
            ".*_elbow_joint": 1.25,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "waist_yaw": _motor(["waist_z_joint"], 137.0, 19.38),
        "waist_roll": _motor(["waist_x_joint"], 137.0, 19.38),
        "waist_pitch": _motor(["waist_y_joint"], 363.0, 20.0),
        "arms": _motor([".*_shoulder_[xyz]_joint", ".*_elbow_joint"], 137.0, 19.38),
        "wrists": _motor([".*_wrist_[xyz]_joint"], 50.0, 23.76),
        "hip_pitch_roll": _motor([".*_hip_[xy]_joint"], 363.0, 20.0),
        "knees": _motor([".*_knee_joint"], 363.0, 20.0),
        "hip_yaw": _motor([".*_hip_z_joint"], 137.0, 19.38),
        "ankle_pitch": _motor([".*_ankle_y_joint"], 137.0, 19.38),
        "ankle_roll": _motor([".*_ankle_x_joint"], 50.0, 23.76),
    },
)
