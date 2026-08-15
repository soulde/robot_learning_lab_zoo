# Copyright (c) 2024-2026 Ziqi Fan
# SPDX-License-Identifier: Apache-2.0

import os

import isaaclab.sim as sim_utils
from isaaclab.actuators import DCMotorCfg
from isaaclab.assets.articulation import ArticulationCfg
from soulde_robot_zoo import ROBOTS_DIR


def _require_user_tmp_dir() -> str:
    tmp_dir = os.environ.get("TMPDIR")
    if not tmp_dir:
        tmp_dir = "/tmp/IsaacLab"
    return os.path.realpath(os.path.abspath(os.path.expanduser(tmp_dir)))


_USER_TMP_DIR = _require_user_tmp_dir()

DEEPROBOTICS_LITE3_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        asset_path=f"{ROBOTS_DIR}/deeprobotics/lite3_description/urdf/lite3.urdf",
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
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.35),
        joint_pos={
            ".*HipX_joint": 0.0,
            ".*HipY_joint": -0.8,
            ".*Knee_joint": 1.6,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "Hip": DCMotorCfg(
            joint_names_expr=[".*_Hip[X,Y]_joint"],
            effort_limit=24.0,
            saturation_effort=24.0,
            velocity_limit=26.2,
            stiffness=30.0,
            damping=0.5,
            friction=0.0,
        ),
        "Knee": DCMotorCfg(
            joint_names_expr=[".*_Knee_joint"],
            effort_limit=36.0,
            saturation_effort=36.0,
            velocity_limit=17.3,
            stiffness=30.0,
            damping=0.5,
            friction=0.0,
        ),
    },
)

DEEPROBOTICS_M20_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        asset_path=f"{ROBOTS_DIR}/deeprobotics/m20_description/urdf/m20.urdf",
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
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0, damping=0)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.52),
        joint_pos={
            ".*hipx_joint": 0.0,
            "f[l,r]_hipy_joint": -0.6,
            "h[l,r]_hipy_joint": 0.6,
            "f[l,r]_knee_joint": 1.0,
            "h[l,r]_knee_joint": -1.0,
            ".*wheel_joint": 0.0,
        },
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "joint": DCMotorCfg(
            joint_names_expr=[".*hipx_joint", ".*hipy_joint", ".*knee_joint"],
            effort_limit=76.4,
            saturation_effort=76.4,
            velocity_limit=22.4,
            stiffness=80.0,
            damping=2.0,
            friction=0.0,
        ),
        "wheel": DCMotorCfg(
            joint_names_expr=[".*_wheel_joint"],
            effort_limit=21.6,
            saturation_effort=21.6,
            velocity_limit=79.3,
            stiffness=0.0,
            damping=0.6,
            friction=0.0,
        ),
    },
)

DEEPROBOTICS_DR02_STANDARD_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        fix_base=False,
        merge_fixed_joints=True,
        replace_cylinders_with_capsules=False,
        asset_path=f"{ROBOTS_DIR}/deeprobotics/dr02_standard_description/urdf/dr02_std.urdf",
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
        usd_dir=os.path.join(_USER_TMP_DIR, "IsaacLab", "dr02_standard"),
        usd_file_name="dr02_std.usd",
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False, solver_position_iteration_count=4, solver_velocity_iteration_count=0
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
        "leg_pitch_roll_knee": DCMotorCfg(
            joint_names_expr=[".*_hip_[xy]_joint", ".*_knee_joint"],
            effort_limit=363.0,
            saturation_effort=363.0,
            velocity_limit=20.0,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "hip_yaw": DCMotorCfg(
            joint_names_expr=[".*_hip_z_joint"],
            effort_limit=137.0,
            saturation_effort=137.0,
            velocity_limit=19.38,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "ankle_pitch": DCMotorCfg(
            joint_names_expr=[".*_ankle_y_joint"],
            effort_limit=137.0,
            saturation_effort=137.0,
            velocity_limit=19.38,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "ankle_roll": DCMotorCfg(
            joint_names_expr=[".*_ankle_x_joint"],
            effort_limit=50.0,
            saturation_effort=50.0,
            velocity_limit=23.76,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "waist_arms": DCMotorCfg(
            joint_names_expr=["waist_z_joint", ".*_shoulder_[xyz]_joint", ".*_elbow_joint"],
            effort_limit=137.0,
            saturation_effort=137.0,
            velocity_limit=19.38,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
    },
)

DEEPROBOTICS_DR02_PRO_CFG = ArticulationCfg(
    spawn=DEEPROBOTICS_DR02_STANDARD_CFG.spawn.replace(
        asset_path=f"{ROBOTS_DIR}/deeprobotics/dr02_pro_description/urdf/dr02_pro.urdf",
        usd_dir=os.path.join(_USER_TMP_DIR, "IsaacLab", "dr02_pro"),
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
        "waist_yaw": DCMotorCfg(
            joint_names_expr=["waist_z_joint"],
            effort_limit=137.0,
            saturation_effort=137.0,
            velocity_limit=19.38,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "waist_roll": DCMotorCfg(
            joint_names_expr=["waist_x_joint"],
            effort_limit=137.0,
            saturation_effort=137.0,
            velocity_limit=19.38,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "waist_pitch": DCMotorCfg(
            joint_names_expr=["waist_y_joint"],
            effort_limit=363.0,
            saturation_effort=363.0,
            velocity_limit=20.0,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "arms": DCMotorCfg(
            joint_names_expr=[".*_shoulder_[xyz]_joint", ".*_elbow_joint"],
            effort_limit=137.0,
            saturation_effort=137.0,
            velocity_limit=19.38,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "wrists": DCMotorCfg(
            joint_names_expr=[".*_wrist_[xyz]_joint"],
            effort_limit=50.0,
            saturation_effort=50.0,
            velocity_limit=23.76,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "hip_pitch_roll": DCMotorCfg(
            joint_names_expr=[".*_hip_[xy]_joint"],
            effort_limit=363.0,
            saturation_effort=363.0,
            velocity_limit=20.0,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "knees": DCMotorCfg(
            joint_names_expr=[".*_knee_joint"],
            effort_limit=363.0,
            saturation_effort=363.0,
            velocity_limit=20.0,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "hip_yaw": DCMotorCfg(
            joint_names_expr=[".*_hip_z_joint"],
            effort_limit=137.0,
            saturation_effort=137.0,
            velocity_limit=19.38,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "ankle_pitch": DCMotorCfg(
            joint_names_expr=[".*_ankle_y_joint"],
            effort_limit=137.0,
            saturation_effort=137.0,
            velocity_limit=19.38,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
        "ankle_roll": DCMotorCfg(
            joint_names_expr=[".*_ankle_x_joint"],
            effort_limit=50.0,
            saturation_effort=50.0,
            velocity_limit=23.76,
            stiffness=625.0,
            damping=0.25,
            friction=0.0,
        ),
    },
)
