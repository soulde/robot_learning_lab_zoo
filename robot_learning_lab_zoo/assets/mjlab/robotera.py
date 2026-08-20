"""MJLab configurations for RobotEra robots."""

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg
from robot_learning_lab_zoo import ROBOTS_DIR

XML = ROBOTS_DIR / "robotera" / "xbot_description" / "xmls" / "robot.xml"


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name: geom.name = f"{body.name}_collision{index}"
    for body_name, side in zip(("left_ankle_roll_link", "right_ankle_roll_link"), ("left", "right"), strict=True):
        body = spec.body(body_name); body.geoms[-1].name = f"{side}_foot_collision"
        body.add_site(name=f"{side}_foot", size=[0.01])
    base = spec.body("base"); base.add_site(name="imu", size=[0.01])
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

ROBOTERA_XBOT_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 0.95), joint_pos={".*": 0.0}, joint_vel={".*": 0.0}),
    collisions=(COLLISION,), spec_fn=get_spec,
    articulation=EntityArticulationInfoCfg(actuators=(
        actuator((".*_leg_roll_joint", ".*_leg_yaw_joint"), 200.0, 10.0, 100.0, 0.01),
        actuator((".*_leg_pitch_joint", ".*_knee_joint"), 350.0, 10.0, 250.0, 0.01),
        actuator((".*_ankle_.*_joint",), 15.0, 10.0, 100.0, 0.01),
        actuator(("waist_.*",), 200.0, 10.0, 100.0, 0.01),
        actuator((".*_shoulder_.*_joint",), 100.0, 10.0, 80.0, 0.01),
        actuator((".*_arm_yaw_joint", ".*_elbow_.*_joint", ".*_wrist_.*_joint"), 100.0, 10.0, 50.0, 0.01),
    ), soft_joint_pos_limit_factor=0.9),
)
