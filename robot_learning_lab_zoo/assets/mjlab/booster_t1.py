"""MJLab configuration for the Booster T1 humanoid."""

from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

BOOSTER_T1_XML: Path = ROBOTS_DIR / "booster" / "t1_description" / "xmls" / "robot.xml"
BOOSTER_T1_FOOT_BODY_NAMES = ("left_foot_link", "right_foot_link")
BOOSTER_T1_FOOT_SITE_NAMES = ("left_foot", "right_foot")
BOOSTER_T1_FOOT_GEOM_NAMES = ("left_foot_collision", "right_foot_collision")
BOOSTER_T1_ACTION_SCALE = 0.25


def get_spec() -> mujoco.MjSpec:
    """Load T1 and add stable collision, foot-site, and IMU names."""
    spec = mujoco.MjSpec.from_file(str(BOOSTER_T1_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, geom_name, site_name in zip(
        BOOSTER_T1_FOOT_BODY_NAMES,
        BOOSTER_T1_FOOT_GEOM_NAMES,
        BOOSTER_T1_FOOT_SITE_NAMES,
        strict=True,
    ):
        body = spec.body(body_name)
        body.geoms[-1].name = geom_name
        body.add_site(name=site_name, size=[0.01])
    trunk = spec.body("Trunk")
    trunk.add_site(name="imu", size=[0.01])
    spec.add_sensor(
        name="imu_lin_vel",
        type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname="imu",
    )
    spec.add_sensor(
        name="imu_ang_vel",
        type=mujoco.mjtSensor.mjSENS_GYRO,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname="imu",
    )
    return spec


BOOSTER_T1_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.72),
        joint_pos={
            ".*": 0.0,
            ".*_Shoulder_Pitch": 0.2,
            "Left_Shoulder_Roll": -1.35,
            "Right_Shoulder_Roll": 1.35,
            "Left_Elbow_Yaw": -0.5,
            "Right_Elbow_Yaw": 0.5,
            ".*_Hip_Pitch": -0.20,
            ".*_Knee_Pitch": 0.42,
            ".*_Ankle_Pitch": -0.23,
        },
        joint_vel={".*": 0.0},
    ),
    collisions=(
        CollisionCfg(
            geom_names_expr=(".*_collision.*",),
            contype=1,
            conaffinity=1,
            condim={"^(left|right)_foot_collision$": 3, ".*": 1},
            priority={"^(left|right)_foot_collision$": 1, ".*": 0},
            friction={"^(left|right)_foot_collision$": (0.6,)},
        ),
    ),
    spec_fn=get_spec,
    articulation=EntityArticulationInfoCfg(
        actuators=(
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_Hip_Pitch",),
                stiffness=200.0,
                damping=5.0,
                effort_limit=45.0,
                armature=0.01,
            ),
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_Hip_Roll", ".*_Hip_Yaw", "Waist"),
                stiffness=200.0,
                damping=5.0,
                effort_limit=30.0,
                armature=0.01,
            ),
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_Knee_Pitch",),
                stiffness=200.0,
                damping=5.0,
                effort_limit=60.0,
                armature=0.01,
            ),
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_Ankle_Pitch",),
                stiffness=50.0,
                damping=1.0,
                effort_limit=24.0,
                armature=0.01,
            ),
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_Ankle_Roll",),
                stiffness=50.0,
                damping=1.0,
                effort_limit=15.0,
                armature=0.01,
            ),
            BuiltinPositionActuatorCfg(
                target_names_expr=(".*_Shoulder_Pitch", ".*_Shoulder_Roll", ".*_Elbow_Pitch", ".*_Elbow_Yaw"),
                stiffness=40.0,
                damping=10.0,
                effort_limit=18.0,
                armature=0.01,
            ),
        ),
        soft_joint_pos_limit_factor=0.9,
    ),
)
