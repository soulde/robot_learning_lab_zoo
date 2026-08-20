"""MJLab configurations for Unitree wheeled quadrupeds."""

from functools import partial
from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg, BuiltinVelocityActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

B2W_XML = ROBOTS_DIR / "unitree" / "b2w_description" / "xmls" / "b2w_description.xml"
GO2W_XML = ROBOTS_DIR / "unitree" / "go2w_description" / "xmls" / "go2w_description.xml"
LEG_JOINT_NAMES = tuple(
    f"{leg}_{joint}_joint" for leg in ("FR", "FL", "RR", "RL") for joint in ("hip", "thigh", "calf")
)
WHEEL_JOINT_NAMES = tuple(f"{leg}_foot_joint" for leg in ("FR", "FL", "RR", "RL"))
WHEEL_BODY_NAMES = tuple(f"{leg}_foot" for leg in ("FR", "FL", "RR", "RL"))
WHEEL_SITE_NAMES = tuple(f"{leg}_wheel" for leg in ("FR", "FL", "RR", "RL"))
WHEEL_GEOM_NAMES = tuple(f"{leg}_wheel_collision" for leg in ("FR", "FL", "RR", "RL"))


def get_spec(xml_path: Path, base_name: str) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(xml_path))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, site_name, geom_name in zip(WHEEL_BODY_NAMES, WHEEL_SITE_NAMES, WHEEL_GEOM_NAMES, strict=True):
        body = spec.body(body_name)
        body.geoms[-1].name = geom_name
        body.add_site(name=site_name, size=[0.01])
    base = spec.body(base_name)
    base.add_site(name="imu", size=[0.01])
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
    condim={".*_wheel_collision": 3, ".*": 1},
    priority={".*_wheel_collision": 1, ".*": 0},
    friction={".*_wheel_collision": (0.6,)},
)


def make_cfg(
    xml: Path, base: str, height: float, kp: float, kd: float, leg_effort: float, wheel_effort: float, wheel_kd: float
):
    return EntityCfg(
        init_state=EntityCfg.InitialStateCfg(
            pos=(0.0, 0.0, height),
            joint_pos={".*": 0.0, ".*_thigh_joint": 0.8, ".*_calf_joint": -1.5},
            joint_vel={".*": 0.0},
        ),
        collisions=(COLLISION,),
        spec_fn=partial(get_spec, xml, base),
        articulation=EntityArticulationInfoCfg(
            actuators=(
                BuiltinPositionActuatorCfg(
                    target_names_expr=(".*_hip_joint", ".*_thigh_joint"),
                    stiffness=kp,
                    damping=kd,
                    effort_limit=leg_effort,
                ),
                BuiltinPositionActuatorCfg(
                    target_names_expr=(".*_calf_joint",),
                    stiffness=kp,
                    damping=kd,
                    effort_limit=320.0 if xml == B2W_XML else leg_effort,
                ),
                BuiltinVelocityActuatorCfg(
                    target_names_expr=(".*_foot_joint",),
                    damping=wheel_kd,
                    effort_limit=wheel_effort,
                ),
            ),
            soft_joint_pos_limit_factor=0.9,
        ),
    )


UNITREE_B2W_CFG = make_cfg(B2W_XML, "base_link", 0.65, 160.0, 5.0, 200.0, 20.0, 1.0)
UNITREE_GO2W_CFG = make_cfg(GO2W_XML, "base", 0.45, 25.0, 0.5, 23.5, 23.5, 0.5)
