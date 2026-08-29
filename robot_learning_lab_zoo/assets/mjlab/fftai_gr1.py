"""MJLab configurations for FFTAI GR1 humanoids."""

from functools import partial
from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

GR1T1_XML = ROBOTS_DIR / "fftai" / "gr1t1_description" / "xmls" / "GR1T1.xml"
GR1T2_XML = ROBOTS_DIR / "fftai" / "gr1t2_description" / "xmls" / "GR1T2.xml"
GR1_FOOT_BODY_NAMES = ("l_foot_roll", "r_foot_roll")
GR1_FOOT_SITE_NAMES = ("left_foot", "right_foot")
GR1_FOOT_GEOM_NAMES = ("left_foot_collision", "right_foot_collision")
GR1_ACTION_SCALE = 0.25


def get_spec(xml_path: Path) -> mujoco.MjSpec:
    """Load a GR1 model and add task-facing names and sensors."""
    spec = mujoco.MjSpec.from_file(str(xml_path))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, geom_name, site_name in zip(
        GR1_FOOT_BODY_NAMES, GR1_FOOT_GEOM_NAMES, GR1_FOOT_SITE_NAMES, strict=True
    ):
        body = spec.body(body_name)
        body.geoms[-1].name = geom_name
        body.add_site(name=site_name, size=[0.01])
    base = spec.body("base")
    base.add_site(name="imu", size=[0.01])
    for name, sensor_type in (
        ("imu_lin_vel", mujoco.mjtSensor.mjSENS_VELOCIMETER),
        ("imu_ang_vel", mujoco.mjtSensor.mjSENS_GYRO),
    ):
        spec.add_sensor(
            name=name,
            type=sensor_type,
            objtype=mujoco.mjtObj.mjOBJ_SITE,
            objname="imu",
        )
    return spec


GR1_INITIAL_STATE = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, 0.93),
    joint_pos={
        ".*": 0.0,
        ".*_hip_pitch": -0.2618,
        ".*_knee_pitch": 0.5236,
        ".*_ankle_pitch": -0.2618,
        "l_shoulder_roll": 0.2,
        "r_shoulder_roll": -0.2,
        ".*_elbow_pitch": -0.3,
    },
    joint_vel={".*": 0.0},
)

GR1_COLLISION = CollisionCfg(
    geom_names_expr=(".*_collision.*",),
    contype=1,
    conaffinity=1,
    condim={"^(left|right)_foot_collision$": 3, ".*": 1},
    priority={"^(left|right)_foot_collision$": 1, ".*": 0},
    friction={"^(left|right)_foot_collision$": (0.6,)},
)


def _actuator(
    pattern: str,
    stiffness: float,
    damping: float,
    effort: float,
    armature: float = 0.01,
) -> BuiltinPositionActuatorCfg:
    return BuiltinPositionActuatorCfg(
        target_names_expr=(pattern,),
        stiffness=stiffness,
        damping=damping,
        effort_limit=effort,
        armature=armature,
    )


GR1_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        _actuator(".*_hip_roll", 251.625, 14.72, 100.0, 0.1),
        _actuator(".*_hip_yaw", 362.5214, 10.0833, 82.5, 0.1),
        _actuator(".*_hip_pitch", 200.0, 11.0, 225.0, 0.1),
        _actuator(".*_knee_pitch", 200.0, 11.0, 225.0, 0.1),
        _actuator(".*_ankle_pitch", 10.9805, 0.5991, 15.0, 0.1),
        _actuator(".*_ankle_roll", 40.0, 2.0, 30.0, 0.1),
        _actuator(".*waist_yaw", 362.5214, 10.0833, 82.5, 0.1),
        _actuator(".*waist_pitch", 362.5214, 10.0833, 82.5, 0.1),
        _actuator(".*waist_roll", 362.5214, 10.0833, 82.5, 0.1),
        _actuator(".*head_yaw", 10.0, 1.0, 10.2, 0.01),
        _actuator(".*head_pitch", 10.0, 1.0, 3.95, 0.01),
        _actuator(".*head_roll", 10.0, 1.0, 3.95, 0.01),
        _actuator(".*_shoulder_pitch", 92.85, 2.575, 38.0, 0.05),
        _actuator(".*_shoulder_roll", 92.85, 2.575, 38.0, 0.05),
        _actuator(".*_shoulder_yaw", 112.06, 3.1, 30.0, 0.05),
        _actuator(".*_elbow_pitch", 112.06, 3.1, 30.0, 0.05),
        _actuator(".*_wrist_yaw", 10.0, 1.0, 10.2, 0.01),
        _actuator(".*_wrist_roll", 10.0, 1.0, 3.95, 0.01),
        _actuator(".*_wrist_pitch", 10.0, 1.0, 3.95, 0.01),
    ),
    soft_joint_pos_limit_factor=0.9,
)

FFTAI_GR1T1_CFG = EntityCfg(
    init_state=GR1_INITIAL_STATE,
    collisions=(GR1_COLLISION,),
    spec_fn=partial(get_spec, GR1T1_XML),
    articulation=GR1_ARTICULATION,
)

FFTAI_GR1T2_CFG = EntityCfg(
    init_state=GR1_INITIAL_STATE,
    collisions=(GR1_COLLISION,),
    spec_fn=partial(get_spec, GR1T2_XML),
    articulation=GR1_ARTICULATION,
)
