"""MJLab configurations for OpenLoong robots."""

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg
from robot_learning_lab_zoo import ROBOTS_DIR

XML = ROBOTS_DIR / "openloong" / "loong_description" / "xmls" / "loong.xml"


def get_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name: geom.name = f"{body.name}_collision{index}"
    for body_name, side in zip(("Link_ankle_l_roll", "Link_ankle_r_roll"), ("left", "right"), strict=True):
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


def actuator(patterns, kp, kd, effort):
    return BuiltinPositionActuatorCfg(target_names_expr=patterns, stiffness=kp, damping=kd, effort_limit=effort)

OPENLOONG_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 1.2), joint_pos={".*": 0.0,
        "J_hip_.*_pitch": 0.2, "J_knee_.*_pitch": -0.5, "J_ankle_.*_pitch": 0.3}, joint_vel={".*": 0.0}),
    collisions=(COLLISION,), spec_fn=get_spec,
    articulation=EntityArticulationInfoCfg(actuators=(
        actuator(("J_hip_.*_roll", "J_hip_.*_pitch"), 400.0, 2.0, 396.0),
        actuator(("J_hip_.*_yaw",), 200.0, 2.0, 160.0),
        actuator(("J_knee_.*_pitch",), 400.0, 4.0, 396.0),
        actuator(("J_ankle_.*_pitch", "J_ankle_.*_roll"), 120.0, 0.5, 58.5),
    ), soft_joint_pos_limit_factor=0.9),
)
