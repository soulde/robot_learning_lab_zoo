"""Unitree B2 constants for MJLab."""

from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

UNITREE_B2_XML: Path = ROBOTS_DIR / "unitree" / "b2_description" / "xmls" / "b2_description.xml"

UNITREE_B2_JOINT_NAMES = (
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
)
UNITREE_B2_FOOT_SITE_NAMES = ("FR_foot", "FL_foot", "RR_foot", "RL_foot")
UNITREE_B2_FOOT_BODY_NAMES = ("FR_calf", "FL_calf", "RR_calf", "RL_calf")
UNITREE_B2_FOOT_GEOM_NAMES = tuple(f"{site}_collision" for site in UNITREE_B2_FOOT_SITE_NAMES)
UNITREE_B2_ACTION_SCALE = {".*_hip_joint": 0.125, "^(?!.*_hip_joint).*": 0.25}


def get_spec() -> mujoco.MjSpec:
    """Load the B2 MJCF and add names/sites used by MJLab sensors."""
    spec = mujoco.MjSpec.from_file(str(UNITREE_B2_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, site_name in zip(UNITREE_B2_FOOT_BODY_NAMES, UNITREE_B2_FOOT_SITE_NAMES, strict=True):
        body = spec.body(body_name)
        body.geoms[-1].name = f"{site_name}_collision"
        body.add_site(name=site_name, pos=[0.0, 0.0, -0.35], size=[0.01])
    return spec


STANDING_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.58),
    joint_pos={
        ".*L_hip_joint": 0.0,
        ".*R_hip_joint": 0.0,
        "F.*_thigh_joint": 0.8,
        "R.*_thigh_joint": 0.8,
        ".*_calf_joint": -1.5,
    },
    joint_vel={".*": 0.0},
)

FOOT_COLLISION = CollisionCfg(
    geom_names_expr=UNITREE_B2_FOOT_GEOM_NAMES,
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(0.6,),
)

UNITREE_B2_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        BuiltinPositionActuatorCfg(
            target_names_expr=(".*_hip_joint", ".*_thigh_joint"),
            stiffness=160.0,
            damping=5.0,
            effort_limit=200.0,
        ),
        BuiltinPositionActuatorCfg(
            target_names_expr=(".*_calf_joint",),
            stiffness=160.0,
            damping=5.0,
            effort_limit=320.0,
        ),
    ),
    soft_joint_pos_limit_factor=0.9,
)

UNITREE_B2_CFG = EntityCfg(
    init_state=STANDING_KEYFRAME,
    collisions=(FOOT_COLLISION,),
    spec_fn=get_spec,
    articulation=UNITREE_B2_ARTICULATION,
)
