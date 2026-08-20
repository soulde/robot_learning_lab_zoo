"""MJLab configurations for Zsibot robots."""

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg, BuiltinVelocityActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg
from robot_learning_lab_zoo import ROBOTS_DIR

ZSL1W_XML = ROBOTS_DIR / "zsibot" / "zsl1w_description" / "xmls" / "zsl1w.xml"
ZSL1W_WHEEL_BODIES = ("FR_FOOT_LINK", "FL_FOOT_LINK", "RR_FOOT_LINK", "RL_FOOT_LINK")
ZSL1W_WHEEL_GEOMS = tuple(f"{name}_collision" for name in ZSL1W_WHEEL_BODIES)


def get_zsl1w_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(ZSL1W_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name: geom.name = f"{body.name}_collision{index}"
    for body_name in ZSL1W_WHEEL_BODIES:
        body = spec.body(body_name); body.geoms[-1].name = f"{body_name}_collision"
        body.add_site(name=f"{body_name}_site", size=[0.01])
    base = spec.body("BASE_LINK"); base.add_site(name="imu", size=[0.01])
    spec.add_sensor(name="imu_lin_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_ang_vel", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec


ZSIBOT_ZSL1W_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 0.4),
        joint_pos={".*": 0.0, ".*_HIP_JOINT": 0.8, ".*_KNEE_JOINT": -1.5}, joint_vel={".*": 0.0}),
    collisions=(CollisionCfg(geom_names_expr=(".*_collision.*",), contype=1, conaffinity=1,
        condim={".*_collision": 3, ".*": 1}, priority={".*_collision": 1, ".*": 0},
        friction={name: (0.6,) for name in ZSL1W_WHEEL_GEOMS}),),
    spec_fn=get_zsl1w_spec,
    articulation=EntityArticulationInfoCfg(actuators=(
        BuiltinPositionActuatorCfg(target_names_expr=(".*_(ABAD|HIP|KNEE)_JOINT",),
            stiffness=20.0, damping=0.7, effort_limit=28.0),
        BuiltinVelocityActuatorCfg(target_names_expr=(".*_FOOT_JOINT",), damping=0.7, effort_limit=28.0),
    ), soft_joint_pos_limit_factor=0.9),
)
