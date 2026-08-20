"""MJLab configurations for DeepRobotics robots."""

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg, BuiltinVelocityActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg
from robot_learning_lab_zoo import ROBOTS_DIR

M20_XML = ROBOTS_DIR / "deeprobotics" / "m20_description" / "xmls" / "m20.xml"
M20_WHEEL_BODIES = tuple(f"{leg}_wheel" for leg in ("fl", "fr", "hl", "hr"))
M20_WHEEL_GEOMS = tuple(f"{name}_collision" for name in M20_WHEEL_BODIES)


def get_m20_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(M20_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name: geom.name = f"{body.name}_collision{index}"
    for body_name in M20_WHEEL_BODIES:
        body = spec.body(body_name); body.geoms[-1].name = f"{body_name}_collision"
        body.add_site(name=f"{body_name}_site", size=[0.01])
    base = spec.body("base_link"); base.add_site(name="imu", size=[0.01])
    spec.add_sensor(name="imu_lin_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_ang_vel", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec


DEEPROBOTICS_M20_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 0.52), joint_pos={".*": 0.0,
        "(fl|fr)_hipy_joint": -0.6, "(hl|hr)_hipy_joint": 0.6,
        "(fl|fr)_knee_joint": 1.0, "(hl|hr)_knee_joint": -1.0}, joint_vel={".*": 0.0}),
    collisions=(CollisionCfg(geom_names_expr=(".*_collision.*",), contype=1, conaffinity=1,
        condim={".*_collision": 3, ".*": 1}, priority={".*_collision": 1, ".*": 0},
        friction={name: (0.6,) for name in M20_WHEEL_GEOMS}),),
    spec_fn=get_m20_spec,
    articulation=EntityArticulationInfoCfg(actuators=(
        BuiltinPositionActuatorCfg(target_names_expr=(".*_(hipx|hipy|knee)_joint",),
            stiffness=80.0, damping=2.0, effort_limit=76.4),
        BuiltinVelocityActuatorCfg(target_names_expr=(".*_wheel_joint",), damping=0.6, effort_limit=21.6),
    ), soft_joint_pos_limit_factor=0.9),
)
