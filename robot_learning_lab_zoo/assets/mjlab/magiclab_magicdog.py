"""MagicLab MagicDog constants for MJLab."""

from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

MAGICDOG_XML: Path = ROBOTS_DIR / "magiclab" / "magicdog" / "xmls" / "magicdog.xml"
MAGICDOG_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL") for joint in ("hip", "thigh", "calf")
)
MAGICDOG_FOOT_SITE_NAMES = tuple(f"{leg}_foot" for leg in ("FR", "FL", "RR", "RL"))
MAGICDOG_FOOT_BODY_NAMES = tuple(f"{leg}_calf" for leg in ("FR", "FL", "RR", "RL"))
MAGICDOG_FOOT_GEOM_NAMES = tuple(f"{name}_collision" for name in MAGICDOG_FOOT_SITE_NAMES)
MAGICDOG_ACTION_SCALE = {".*_hip_joint": 1.0, "^(?!.*_hip_joint).*": 0.25}


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(MAGICDOG_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, site_name in zip(MAGICDOG_FOOT_BODY_NAMES, MAGICDOG_FOOT_SITE_NAMES, strict=True):
        body = spec.body(body_name)
        foot_geom = body.geoms[-1]
        foot_geom.name = f"{site_name}_collision"
        body.add_site(name=site_name, pos=foot_geom.pos, size=[0.01])
    return spec


STANDING_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.5),
    joint_pos={".*_hip_joint": 0.0, ".*_thigh_joint": 0.6683, ".*_calf_joint": -1.312},
    joint_vel={".*": 0.0},
)
FOOT_COLLISION = CollisionCfg(
    geom_names_expr=MAGICDOG_FOOT_GEOM_NAMES,
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(0.6,),
)
MAGICDOG_CFG = EntityCfg(
    init_state=STANDING_KEYFRAME,
    collisions=(FOOT_COLLISION,),
    spec_fn=get_spec,
    articulation=EntityArticulationInfoCfg(
        actuators=(
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_joint",),
                stiffness=30.0,
                damping=1.0,
                effort_limit=25.0,
            ),
        ),
        soft_joint_pos_limit_factor=1.0,
    ),
)
