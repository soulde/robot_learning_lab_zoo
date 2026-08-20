"""MJLab configurations for RoboParty robots."""

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg
from robot_learning_lab_zoo import ROBOTS_DIR

XML = ROBOTS_DIR / "roboparty" / "atom01_description" / "xmls" / "atom01.xml"


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name: geom.name = f"{body.name}_collision{index}"
    for body_name, side in zip(("left_ankle_roll_link", "right_ankle_roll_link"), ("left", "right"), strict=True):
        body = spec.body(body_name); body.geoms[-1].name = f"{side}_foot_collision"
        body.add_site(name=f"{side}_foot", size=[0.01])
    base = spec.body("base_link"); base.add_site(name="imu", size=[0.01])
    spec.add_sensor(name="imu_lin_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_ang_vel", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec


COLLISION = CollisionCfg(geom_names_expr=(".*_collision.*",), contype=1, conaffinity=1,
    condim={"^(left|right)_foot_collision$": 3, ".*": 1}, priority={"^(left|right)_foot_collision$": 1, ".*": 0},
    friction={"^(left|right)_foot_collision$": (0.6,)})


def actuator(patterns, kp, kd, effort, armature=0.0):
    return BuiltinPositionActuatorCfg(target_names_expr=patterns, stiffness=kp, damping=kd,
                                      effort_limit=effort, armature=armature)

ROBOPARTY_ATOM01_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 0.7), joint_pos={".*": 0.0,
        ".*_thigh_pitch_joint": -0.2, ".*_knee_joint": 0.4, ".*_ankle_pitch_joint": -0.2,
        ".*_arm_pitch_joint": 0.1, "left_arm_roll_joint": 0.07, "right_arm_roll_joint": -0.07,
        ".*_elbow_pitch_joint": 1.0}, joint_vel={".*": 0.0}),
    collisions=(COLLISION,), spec_fn=get_spec,
    articulation=EntityArticulationInfoCfg(actuators=(
        actuator((".*_thigh_.*_joint",), 100.0, 3.0, 80.0, 0.01),
        actuator((".*_knee_joint", "torso_joint"), 150.0, 5.0, 80.0, 0.01),
        actuator((".*_ankle_.*_joint",), 40.0, 1.5, 18.0, 0.01),
        actuator((".*_arm_.*_joint",), 60.0, 2.0, 18.0, 0.01),
        actuator((".*_elbow_pitch_joint",), 40.0, 1.5, 18.0, 0.01),
        actuator((".*_elbow_yaw_joint",), 20.0, 1.0, 18.0, 0.01),
    ), soft_joint_pos_limit_factor=0.9),
)
