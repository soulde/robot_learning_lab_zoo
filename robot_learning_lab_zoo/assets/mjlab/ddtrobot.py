"""MJLab configurations for DDTRobot robots."""

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg, BuiltinVelocityActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg
from robot_learning_lab_zoo import ROBOTS_DIR

TITA_XML = ROBOTS_DIR / "ddt" / "tita_description" / "xmls" / "tita.xml"
TITA_WHEEL_BODIES = ("right_leg_4", "left_leg_4")
TITA_WHEEL_GEOMS = tuple(f"{name}_collision" for name in TITA_WHEEL_BODIES)


def get_tita_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(TITA_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name: geom.name = f"{body.name}_collision{index}"
    for body_name in TITA_WHEEL_BODIES:
        body = spec.body(body_name); body.geoms[-1].name = f"{body_name}_collision"
        body.add_site(name=f"{body_name}_site", size=[0.01])
    base = spec.body("base_link"); base.add_site(name="imu", size=[0.01])
    spec.add_sensor(name="imu_lin_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_ang_vel", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec


DDTROBOT_TITA_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 0.3),
        joint_pos={".*": 0.0, ".*_leg_2": 0.8, ".*_leg_3": -1.5}, joint_vel={".*": 0.0}),
    collisions=(CollisionCfg(geom_names_expr=(".*_collision.*",), contype=1, conaffinity=1,
        condim={".*_collision": 3, ".*": 1}, priority={".*_collision": 1, ".*": 0},
        friction={name: (0.6,) for name in TITA_WHEEL_GEOMS}),),
    spec_fn=get_tita_spec,
    articulation=EntityArticulationInfoCfg(actuators=(
        BuiltinPositionActuatorCfg(target_names_expr=(".*_leg_[123]",),
            stiffness=40.0, damping=1.0, effort_limit=60.0),
        BuiltinVelocityActuatorCfg(target_names_expr=(".*_leg_4",), damping=1.0, effort_limit=15.0),
    ), soft_joint_pos_limit_factor=0.9),
)
