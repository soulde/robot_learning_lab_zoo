"""Agibot D1 constants for MJLab."""

from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

AGIBOT_D1_XML: Path = ROBOTS_DIR / "agibot" / "d1" / "xmls" / "edu.xml"
AGIBOT_D1_JOINT_NAMES = tuple(
    f"{leg}_{joint}_JOINT"
    for leg in ("FR", "FL", "RR", "RL")
    for joint in ("ABAD", "HIP", "KNEE")
)
AGIBOT_D1_FOOT_SITE_NAMES = tuple(f"{leg}_FOOT_LINK" for leg in ("FR", "FL", "RR", "RL"))
AGIBOT_D1_FOOT_BODY_NAMES = tuple(f"{leg}_KNEE_LINK" for leg in ("FR", "FL", "RR", "RL"))
AGIBOT_D1_FOOT_GEOM_NAMES = tuple(f"{name}_collision" for name in AGIBOT_D1_FOOT_SITE_NAMES)
AGIBOT_D1_ACTION_SCALE = {".*_ABAD_JOINT": 0.125, "^(?!.*_ABAD_JOINT).*": 0.25}


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(AGIBOT_D1_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, site_name in zip(AGIBOT_D1_FOOT_BODY_NAMES, AGIBOT_D1_FOOT_SITE_NAMES, strict=True):
        body = spec.body(body_name)
        foot_geom = body.geoms[-1]
        foot_geom.name = f"{site_name}_collision"
        body.add_site(name=site_name, pos=foot_geom.pos, size=[0.01])
    return spec


STANDING_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.42),
    joint_pos={".*_ABAD_JOINT": 0.0, ".*_HIP_JOINT": 0.8, ".*_KNEE_JOINT": -1.5},
    joint_vel={".*": 0.0},
)
FOOT_COLLISION = CollisionCfg(
    geom_names_expr=AGIBOT_D1_FOOT_GEOM_NAMES,
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(0.6,),
)
AGIBOT_D1_CFG = EntityCfg(
    init_state=STANDING_KEYFRAME,
    collisions=(FOOT_COLLISION,),
    spec_fn=get_spec,
    articulation=EntityArticulationInfoCfg(
        actuators=(
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_(ABAD|HIP|KNEE)_JOINT",),
                stiffness=20.0,
                damping=0.5,
                effort_limit=33.5,
            ),
        ),
        soft_joint_pos_limit_factor=0.9,
    ),
)
