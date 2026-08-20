"""MJLab configurations for MagicLab robots."""

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg, BuiltinVelocityActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg
from robot_learning_lab_zoo import ROBOTS_DIR

MAGICDOG_W_XML = ROBOTS_DIR / "magiclab" / "magicdog_w" / "xmls" / "magicdog_w.xml"
MAGICDOG_W_WHEEL_BODIES = tuple(f"{leg}_wheel" for leg in ("FR", "FL", "RR", "RL"))
MAGICDOG_W_WHEEL_GEOMS = tuple(f"{name}_collision" for name in MAGICDOG_W_WHEEL_BODIES)


def get_magicdog_w_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(MAGICDOG_W_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name: geom.name = f"{body.name}_collision{index}"
    for body_name in MAGICDOG_W_WHEEL_BODIES:
        body = spec.body(body_name); body.geoms[-1].name = f"{body_name}_collision"
        body.add_site(name=f"{body_name}_site", size=[0.01])
    base = spec.body("base"); base.add_site(name="imu", size=[0.01])
    spec.add_sensor(name="imu_lin_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_ang_vel", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec


MAGICLAB_MAGICDOG_W_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(pos=(0.0, 0.0, 0.5),
        joint_pos={".*": 0.0, ".*_thigh_joint": 1.0, ".*_calf_joint": -1.8}, joint_vel={".*": 0.0}),
    collisions=(CollisionCfg(geom_names_expr=(".*_collision.*",), contype=1, conaffinity=1,
        condim={".*_collision": 3, ".*": 1}, priority={".*_collision": 1, ".*": 0},
        friction={name: (0.6,) for name in MAGICDOG_W_WHEEL_GEOMS}),),
    spec_fn=get_magicdog_w_spec,
    articulation=EntityArticulationInfoCfg(actuators=(
        BuiltinPositionActuatorCfg(target_names_expr=(".*_(hip|thigh|calf)_joint",),
            stiffness=30.0, damping=1.0, effort_limit=37.5),
        BuiltinVelocityActuatorCfg(target_names_expr=(".*_wheel_joint",), damping=0.2, effort_limit=15.0),
    ), soft_joint_pos_limit_factor=0.9),
)
