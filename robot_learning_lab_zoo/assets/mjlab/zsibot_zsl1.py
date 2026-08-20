"""Zsibot ZSL1 constants for MJLab."""

from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

ZSIBOT_ZSL1_XML: Path = ROBOTS_DIR / "zsibot" / "zsl1_description" / "xmls" / "zsl1.xml"
ZSIBOT_ZSL1_JOINT_NAMES = tuple(
    f"{leg}_{joint}_JOINT"
    for leg in ("FAR", "FBL", "RAR", "RBL")
    for joint in ("ABAD", "HIP", "KNEE")
)
ZSIBOT_ZSL1_FOOT_SITE_NAMES = ("FR_FOOT_LINK", "FL_FOOT_LINK", "RR_FOOT_LINK", "RL_FOOT_LINK")
ZSIBOT_ZSL1_FOOT_BODY_NAMES = ("FR_KNEE_LINK", "FL_KNEE_LINK", "RR_KNEE_LINK", "RL_KNEE_LINK")
ZSIBOT_ZSL1_FOOT_GEOM_NAMES = tuple(f"{name}_collision" for name in ZSIBOT_ZSL1_FOOT_SITE_NAMES)
ZSIBOT_ZSL1_ACTION_SCALE = {".*_ABAD_JOINT": 0.125, "^(?!.*_ABAD_JOINT).*": 0.25}


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(ZSIBOT_ZSL1_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, site_name in zip(ZSIBOT_ZSL1_FOOT_BODY_NAMES, ZSIBOT_ZSL1_FOOT_SITE_NAMES, strict=True):
        body = spec.body(body_name)
        foot_geom = body.geoms[-1]
        foot_geom.name = f"{site_name}_collision"
        body.add_site(name=site_name, pos=foot_geom.pos, size=[0.01])
    return spec


STANDING_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.4),
    joint_pos={".*_ABAD_JOINT": 0.0, ".*_HIP_JOINT": 0.8, ".*_KNEE_JOINT": -1.5},
    joint_vel={".*": 0.0},
)
FOOT_COLLISION = CollisionCfg(
    geom_names_expr=ZSIBOT_ZSL1_FOOT_GEOM_NAMES,
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(0.6,),
)
ZSIBOT_ZSL1_CFG = EntityCfg(
    init_state=STANDING_KEYFRAME,
    collisions=(FOOT_COLLISION,),
    spec_fn=get_spec,
    articulation=EntityArticulationInfoCfg(
        actuators=(
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_(ABAD|HIP|KNEE)_JOINT",),
                stiffness=20.0,
                damping=0.7,
                effort_limit=28.0,
            ),
        ),
        soft_joint_pos_limit_factor=0.9,
    ),
)
