"""ANYbotics ANYmal-D constants for MJLab."""

from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

ANYMAL_D_XML: Path = ROBOTS_DIR / "anybotics" / "anymal_d_description" / "xmls" / "anymal_d.xml"
ANYMAL_D_JOINT_NAMES = tuple(
    f"{leg}_{joint}" for leg in ("LF", "RF", "LH", "RH") for joint in ("HAA", "HFE", "KFE")
)
ANYMAL_D_FOOT_SITE_NAMES = tuple(f"{leg}_FOOT" for leg in ("LF", "RF", "LH", "RH"))
ANYMAL_D_FOOT_BODY_NAMES = tuple(f"{leg}_SHANK" for leg in ("LF", "RF", "LH", "RH"))
ANYMAL_D_FOOT_GEOM_NAMES = tuple(f"{name}_collision" for name in ANYMAL_D_FOOT_SITE_NAMES)


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(ANYMAL_D_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, site_name in zip(ANYMAL_D_FOOT_BODY_NAMES, ANYMAL_D_FOOT_SITE_NAMES, strict=True):
        body = spec.body(body_name)
        foot_geom = body.geoms[-1]
        foot_geom.name = f"{site_name}_collision"
        body.add_site(name=site_name, pos=foot_geom.pos, size=[0.01])
    return spec


STANDING_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.6),
    joint_pos={
        ".*HAA": 0.0,
        ".*F_HFE": 0.4,
        ".*H_HFE": -0.4,
        ".*F_KFE": -0.8,
        ".*H_KFE": 0.8,
    },
    joint_vel={".*": 0.0},
)
FOOT_COLLISION = CollisionCfg(
    geom_names_expr=ANYMAL_D_FOOT_GEOM_NAMES,
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(0.6,),
)
ANYMAL_D_CFG = EntityCfg(
    init_state=STANDING_KEYFRAME,
    collisions=(FOOT_COLLISION,),
    spec_fn=get_spec,
    articulation=EntityArticulationInfoCfg(
        actuators=(
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*HAA", ".*HFE", ".*KFE"),
                stiffness=40.0,
                damping=5.0,
                effort_limit=80.0,
            ),
        ),
        soft_joint_pos_limit_factor=0.95,
    ),
)
