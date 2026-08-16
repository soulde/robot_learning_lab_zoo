"""MJLab configuration for the Deep Robotics DR02 Standard."""

from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

DR02_XML: Path = ROBOTS_DIR / "deeprobotics" / "dr02_standard_description" / "xmls" / "dr02_std.xml"
DR02_ACTION_SCALE = 0.25
DR02_FOOT_SITE_NAMES = ("left_foot", "right_foot")
DR02_FOOT_BODY_NAMES = ("left_ankle_x_link", "right_ankle_x_link")
DR02_FOOT_GEOM_NAMES = ("left_foot_collision", "right_foot_collision")


def get_spec() -> mujoco.MjSpec:
    """Load the generated XML and add names/sites used by task sensors."""
    spec = mujoco.MjSpec.from_file(str(DR02_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, geom_name, site_name in zip(
        DR02_FOOT_BODY_NAMES,
        DR02_FOOT_GEOM_NAMES,
        DR02_FOOT_SITE_NAMES,
        strict=True,
    ):
        body = spec.body(body_name)
        body.geoms[-1].name = geom_name
        body.add_site(name=site_name, pos=[0.045, 0.0, -0.075], size=[0.01])
    return spec


STANDING_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.95),
    joint_pos={".*": 0.0},
    joint_vel={".*": 0.0},
)

FULL_COLLISION = CollisionCfg(
    geom_names_expr=(".*_collision.*",),
    contype=1,
    conaffinity=1,
    condim={"^(left|right)_foot_collision$": 3, ".*": 1},
    priority={"^(left|right)_foot_collision$": 1, ".*": 0},
    friction={"^(left|right)_foot_collision$": (0.6,)},
)

DR02_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        BuiltinPositionActuatorCfg(
            target_names_expr=(".*_hip_[xy]_joint", ".*_knee_joint"),
            stiffness=625.0,
            damping=0.25,
            effort_limit=363.0,
        ),
        BuiltinPositionActuatorCfg(
            target_names_expr=(".*_hip_z_joint", ".*_ankle_y_joint"),
            stiffness=625.0,
            damping=0.25,
            effort_limit=137.0,
        ),
        BuiltinPositionActuatorCfg(
            target_names_expr=(".*_ankle_x_joint",),
            stiffness=625.0,
            damping=0.25,
            effort_limit=50.0,
        ),
        BuiltinPositionActuatorCfg(
            target_names_expr=("waist_z_joint", ".*_shoulder_[xyz]_joint", ".*_elbow_joint"),
            stiffness=625.0,
            damping=0.25,
            effort_limit=137.0,
        ),
    ),
    soft_joint_pos_limit_factor=0.9,
)

DEEPROBOTICS_DR02_STANDARD_CFG = EntityCfg(
    init_state=STANDING_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=DR02_ARTICULATION,
)
