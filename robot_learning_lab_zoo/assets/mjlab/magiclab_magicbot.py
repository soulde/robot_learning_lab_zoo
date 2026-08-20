"""MJLab configurations for MagicLab MagicBot humanoids."""

from functools import partial
from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

GEN1_XML = ROBOTS_DIR / "magiclab" / "magicbot-Gen1" / "xmls" / "MAGICBOT.xml"
Z1_XML = ROBOTS_DIR / "magiclab" / "magicbot-Z1" / "xmls" / "MagicBotZ1.xml"
MAGICBOT_FOOT_BODY_NAMES = ("LINK_ANKLE_ROLL_L", "LINK_ANKLE_ROLL_R")
MAGICBOT_FOOT_SITE_NAMES = ("left_foot", "right_foot")
MAGICBOT_FOOT_GEOM_NAMES = ("left_foot_collision", "right_foot_collision")
MAGICBOT_ACTION_JOINT_NAMES = (
    "JOINT_HIP_ROLL_L",
    "JOINT_HIP_YAW_L",
    "JOINT_HIP_PITCH_L",
    "JOINT_KNEE_PITCH_L",
    "JOINT_ANKLE_PITCH_L",
    "JOINT_ANKLE_ROLL_L",
    "JOINT_HIP_ROLL_R",
    "JOINT_HIP_YAW_R",
    "JOINT_HIP_PITCH_R",
    "JOINT_KNEE_PITCH_R",
    "JOINT_ANKLE_PITCH_R",
    "JOINT_ANKLE_ROLL_R",
    "joint_la1",
    "joint_ra1",
)


def get_spec(xml_path: Path) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(xml_path))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, geom_name, site_name in zip(
        MAGICBOT_FOOT_BODY_NAMES, MAGICBOT_FOOT_GEOM_NAMES, MAGICBOT_FOOT_SITE_NAMES, strict=True
    ):
        body = spec.body(body_name)
        body.geoms[-1].name = geom_name
        body.add_site(name=site_name, size=[0.01])
    pelvis = spec.body("pelvis")
    pelvis.add_site(name="imu", size=[0.01])
    for name, sensor_type in (
        ("imu_lin_vel", mujoco.mjtSensor.mjSENS_VELOCIMETER),
        ("imu_ang_vel", mujoco.mjtSensor.mjSENS_GYRO),
    ):
        spec.add_sensor(name=name, type=sensor_type, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec


COLLISION = CollisionCfg(
    geom_names_expr=(".*_collision.*",),
    contype=1,
    conaffinity=1,
    condim={"^(left|right)_foot_collision$": 3, ".*": 1},
    priority={"^(left|right)_foot_collision$": 1, ".*": 0},
    friction={"^(left|right)_foot_collision$": (0.6,)},
)

ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        BuiltinPositionActuatorCfg(
            target_names_expr=("JOINT_HIP_PITCH_.*", "JOINT_KNEE_PITCH_.*"),
            stiffness=200.0,
            damping=5.0,
            effort_limit=300.0,
            armature=0.01,
        ),
        BuiltinPositionActuatorCfg(
            target_names_expr=("JOINT_HIP_ROLL_.*", "JOINT_HIP_YAW_.*"),
            stiffness=150.0,
            damping=5.0,
            effort_limit=300.0,
            armature=0.01,
        ),
        BuiltinPositionActuatorCfg(
            target_names_expr=("JOINT_ANKLE_PITCH_.*", "JOINT_ANKLE_ROLL_.*"),
            stiffness=20.0,
            damping=2.0,
            effort_limit=20.0,
            armature=0.01,
        ),
        BuiltinPositionActuatorCfg(
            target_names_expr=("joint_.*a1",),
            stiffness=40.0,
            damping=10.0,
            effort_limit=300.0,
            armature=0.01,
        ),
    ),
    soft_joint_pos_limit_factor=0.9,
)


def _cfg(xml: Path, height: float, hip: float, knee: float, ankle: float) -> EntityCfg:
    return EntityCfg(
        init_state=EntityCfg.InitialStateCfg(
            pos=(0.0, 0.0, height),
            joint_pos={
                ".*": 0.0,
                "JOINT_HIP_PITCH_.*": hip,
                "JOINT_KNEE_PITCH_.*": knee,
                "JOINT_ANKLE_PITCH_.*": ankle,
            },
            joint_vel={".*": 0.0},
        ),
        collisions=(COLLISION,),
        spec_fn=partial(get_spec, xml),
        articulation=ARTICULATION,
    )


MAGICLAB_BOT_GEN1_CFG = _cfg(GEN1_XML, 0.92, -0.4, 0.8, -0.45)
MAGICLAB_BOT_Z1_CFG = _cfg(Z1_XML, 0.75, -0.35, 0.7, -0.35)
