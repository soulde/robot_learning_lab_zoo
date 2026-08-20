"""MJLab configurations for Unitree robots."""

from copy import deepcopy
from pathlib import Path

import mujoco
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.actuator import (
    ElectricActuator,
    reflected_inertia_from_two_stage_planetary,
)
from mjlab.utils.spec_config import CollisionCfg

from robot_learning_lab_zoo import ROBOTS_DIR

##
# MJCF assets.
##

G1_XML: Path = ROBOTS_DIR / "unitree" / "g1_description" / "xmls" / "g1_29dof_rev_1_0.xml"
G1_DEX3_XML: Path = ROBOTS_DIR / "unitree" / "g1_description" / "xmls" / "g1_29dof_with_hand_rev_1_0.xml"
G1_DEX3_BACKPACK_XML: Path = (
    ROBOTS_DIR / "unitree" / "g1_description" / "xmls" / "g1_29dof_with_hand_backpack_1kg.xml"
)
assert G1_XML.exists()
assert G1_DEX3_XML.exists()
assert G1_DEX3_BACKPACK_XML.exists()
G1_FOOT_SITE_NAMES = ("left_foot", "right_foot")
G1_FOOT_BODY_NAMES = ("left_ankle_roll_link", "right_ankle_roll_link")
G1_FOOT_GEOM_NAMES = tuple(f"{side}_foot{index}_collision" for side in ("left", "right") for index in range(1, 5))


def _get_g1_spec(xml_path: Path) -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(xml_path))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for side, body_name, site_name in zip(("left", "right"), G1_FOOT_BODY_NAMES, G1_FOOT_SITE_NAMES, strict=True):
        body = spec.body(body_name)
        for index, geom in enumerate(body.geoms[-4:], start=1):
            geom.name = f"{side}_foot{index}_collision"
        body.add_site(name=site_name, pos=[0.035, 0.0, -0.03], size=[0.01])
    pelvis = spec.body("pelvis")
    pelvis.add_site(name="imu", size=[0.01])
    spec.add_sensor(name="imu_lin_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_ang_vel", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec


def get_spec() -> mujoco.MjSpec:
    return _get_g1_spec(G1_XML)


def get_g1_dex3_spec() -> mujoco.MjSpec:
    return _get_g1_spec(G1_DEX3_XML)


def get_g1_dex3_backpack_spec() -> mujoco.MjSpec:
    return _get_g1_spec(G1_DEX3_BACKPACK_XML)


##
# Actuator config.
##

# Motor specs (from Unitree).
ROTOR_INERTIAS_5020 = (
    0.139e-4,
    0.017e-4,
    0.169e-4,
)
GEARS_5020 = (
    1,
    1 + (46 / 18),
    1 + (56 / 16),
)
ARMATURE_5020 = reflected_inertia_from_two_stage_planetary(ROTOR_INERTIAS_5020, GEARS_5020)

ROTOR_INERTIAS_7520_14 = (
    0.489e-4,
    0.098e-4,
    0.533e-4,
)
GEARS_7520_14 = (
    1,
    4.5,
    1 + (48 / 22),
)
ARMATURE_7520_14 = reflected_inertia_from_two_stage_planetary(ROTOR_INERTIAS_7520_14, GEARS_7520_14)

ROTOR_INERTIAS_7520_22 = (
    0.489e-4,
    0.109e-4,
    0.738e-4,
)
GEARS_7520_22 = (
    1,
    4.5,
    5,
)
ARMATURE_7520_22 = reflected_inertia_from_two_stage_planetary(ROTOR_INERTIAS_7520_22, GEARS_7520_22)

ROTOR_INERTIAS_4010 = (
    0.068e-4,
    0.0,
    0.0,
)
GEARS_4010 = (
    1,
    5,
    5,
)
ARMATURE_4010 = reflected_inertia_from_two_stage_planetary(ROTOR_INERTIAS_4010, GEARS_4010)

ACTUATOR_5020 = ElectricActuator(
    reflected_inertia=ARMATURE_5020,
    velocity_limit=37.0,
    effort_limit=25.0,
)
ACTUATOR_7520_14 = ElectricActuator(
    reflected_inertia=ARMATURE_7520_14,
    velocity_limit=32.0,
    effort_limit=88.0,
)
ACTUATOR_7520_22 = ElectricActuator(
    reflected_inertia=ARMATURE_7520_22,
    velocity_limit=20.0,
    effort_limit=139.0,
)
ACTUATOR_4010 = ElectricActuator(
    reflected_inertia=ARMATURE_4010,
    velocity_limit=22.0,
    effort_limit=5.0,
)

NATURAL_FREQ = 10 * 2.0 * 3.1415926535  # 10Hz
DAMPING_RATIO = 2.0

STIFFNESS_5020 = ARMATURE_5020 * NATURAL_FREQ**2
STIFFNESS_7520_14 = ARMATURE_7520_14 * NATURAL_FREQ**2
STIFFNESS_7520_22 = ARMATURE_7520_22 * NATURAL_FREQ**2
STIFFNESS_4010 = ARMATURE_4010 * NATURAL_FREQ**2

DAMPING_5020 = 2.0 * DAMPING_RATIO * ARMATURE_5020 * NATURAL_FREQ
DAMPING_7520_14 = 2.0 * DAMPING_RATIO * ARMATURE_7520_14 * NATURAL_FREQ
DAMPING_7520_22 = 2.0 * DAMPING_RATIO * ARMATURE_7520_22 * NATURAL_FREQ
DAMPING_4010 = 2.0 * DAMPING_RATIO * ARMATURE_4010 * NATURAL_FREQ

G1_ACTUATOR_5020 = BuiltinPositionActuatorCfg(
    target_names_expr=(
        ".*_elbow_joint",
        ".*_shoulder_pitch_joint",
        ".*_shoulder_roll_joint",
        ".*_shoulder_yaw_joint",
        ".*_wrist_roll_joint",
    ),
    stiffness=STIFFNESS_5020,
    damping=DAMPING_5020,
    effort_limit=ACTUATOR_5020.effort_limit,
    armature=ACTUATOR_5020.reflected_inertia,
)
G1_ACTUATOR_7520_14 = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_hip_pitch_joint", ".*_hip_yaw_joint", "waist_yaw_joint"),
    stiffness=STIFFNESS_7520_14,
    damping=DAMPING_7520_14,
    effort_limit=ACTUATOR_7520_14.effort_limit,
    armature=ACTUATOR_7520_14.reflected_inertia,
)
G1_ACTUATOR_7520_22 = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_hip_roll_joint", ".*_knee_joint"),
    stiffness=STIFFNESS_7520_22,
    damping=DAMPING_7520_22,
    effort_limit=ACTUATOR_7520_22.effort_limit,
    armature=ACTUATOR_7520_22.reflected_inertia,
)
G1_ACTUATOR_4010 = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_wrist_pitch_joint", ".*_wrist_yaw_joint"),
    stiffness=STIFFNESS_4010,
    damping=DAMPING_4010,
    effort_limit=ACTUATOR_4010.effort_limit,
    armature=ACTUATOR_4010.reflected_inertia,
)

# Waist pitch/roll and ankles are 4-bar linkages with 2 5020 actuators.
# Due to the parallel linkage, the effective armature at the ankle and waist joints
# is configuration dependent. Since the exact geometry of the linkage is unknown, we
# assume a nominal 1:1 gear ratio. Under this assumption, the joint armature in the
# nominal configuration is approximated as the sum of the 2 actuators' armatures.
G1_ACTUATOR_WAIST = BuiltinPositionActuatorCfg(
    target_names_expr=("waist_pitch_joint", "waist_roll_joint"),
    stiffness=STIFFNESS_5020 * 2,
    damping=DAMPING_5020 * 2,
    effort_limit=ACTUATOR_5020.effort_limit * 2,
    armature=ACTUATOR_5020.reflected_inertia * 2,
)
G1_ACTUATOR_ANKLE = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_ankle_pitch_joint", ".*_ankle_roll_joint"),
    stiffness=STIFFNESS_5020 * 2,
    damping=DAMPING_5020 * 2,
    effort_limit=ACTUATOR_5020.effort_limit * 2,
    armature=ACTUATOR_5020.reflected_inertia * 2,
)

G1_DEX3_THUMB_ROOT_ACTUATOR = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_hand_thumb_0_joint",),
    stiffness=20.0,
    damping=0.5,
    effort_limit=2.45,
)
G1_DEX3_FINGER_ACTUATOR = BuiltinPositionActuatorCfg(
    target_names_expr=(".*_hand_(?!thumb_0).*_joint",),
    stiffness=20.0,
    damping=0.5,
    effort_limit=1.4,
)

##
# Keyframe config.
##

HOME_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0, 0, 0.783675),
    joint_pos={
        ".*_hip_pitch_joint": -0.1,
        ".*_knee_joint": 0.3,
        ".*_ankle_pitch_joint": -0.2,
        ".*_shoulder_pitch_joint": 0.2,
        ".*_elbow_joint": 1.28,
        "left_shoulder_roll_joint": 0.2,
        "right_shoulder_roll_joint": -0.2,
    },
    joint_vel={".*": 0.0},
)

KNEES_BENT_KEYFRAME = EntityCfg.InitialStateCfg(
    pos=(0, 0, 0.76),
    joint_pos={
        ".*_hip_pitch_joint": -0.312,
        ".*_knee_joint": 0.669,
        ".*_ankle_pitch_joint": -0.363,
        ".*_elbow_joint": 0.6,
        "left_shoulder_roll_joint": 0.2,
        "left_shoulder_pitch_joint": 0.2,
        "right_shoulder_roll_joint": -0.2,
        "right_shoulder_pitch_joint": 0.2,
    },
    joint_vel={".*": 0.0},
)

##
# Collision config.
##

# This enables all collisions, including self collisions.
# Self-collisions are given condim=1 while foot collisions
# are given condim=3.
FULL_COLLISION = CollisionCfg(
    geom_names_expr=(".*_collision",),
    contype=1,
    conaffinity=1,
    condim={r"^(left|right)_foot[1-4]_collision$": 3, ".*_collision": 1},
    priority={r"^(left|right)_foot[1-4]_collision$": 1, ".*": 0},
    friction={r"^(left|right)_foot[1-4]_collision$": (0.6,)},
)

FULL_COLLISION_WITHOUT_SELF = CollisionCfg(
    geom_names_expr=(".*_collision",),
    contype=0,
    conaffinity=1,
    condim={r"^(left|right)_foot[1-4]_collision$": 3, ".*_collision": 1},
    priority={r"^(left|right)_foot[1-4]_collision$": 1, ".*": 0},
    friction={r"^(left|right)_foot[1-4]_collision$": (0.6,)},
)

# This disables all collisions except the feet.
# Feet get condim=3, all other geoms are disabled.
FEET_ONLY_COLLISION = CollisionCfg(
    geom_names_expr=(r"^(left|right)_foot[1-4]_collision$",),
    contype=0,
    conaffinity=1,
    condim=3,
    priority=1,
    friction=(0.6,),
)

##
# Final config.
##

G1_ARTICULATION = EntityArticulationInfoCfg(
    actuators=(
        G1_ACTUATOR_5020,
        G1_ACTUATOR_7520_14,
        G1_ACTUATOR_7520_22,
        G1_ACTUATOR_4010,
        G1_ACTUATOR_WAIST,
        G1_ACTUATOR_ANKLE,
    ),
    soft_joint_pos_limit_factor=0.9,
)


UNITREE_G1_29DOF_CFG = EntityCfg(
    init_state=KNEES_BENT_KEYFRAME,
    collisions=(FULL_COLLISION,),
    spec_fn=get_spec,
    articulation=G1_ARTICULATION,
)

G1_DEX3_ARTICULATION = EntityArticulationInfoCfg(
    actuators=G1_ARTICULATION.actuators + (G1_DEX3_THUMB_ROOT_ACTUATOR, G1_DEX3_FINGER_ACTUATOR),
    soft_joint_pos_limit_factor=0.9,
)

UNITREE_G1_29DOF_DEX3_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(
        pos=KNEES_BENT_KEYFRAME.pos,
        joint_pos={
            **KNEES_BENT_KEYFRAME.joint_pos,
            ".*_hand_thumb_0_joint": 0.0,
            ".*_hand_thumb_[12]_joint": 0.2,
            ".*_hand_(index|middle)_[01]_joint": -0.2,
        },
        joint_vel={".*": 0.0},
    ),
    collisions=(FULL_COLLISION,),
    spec_fn=get_g1_dex3_spec,
    articulation=G1_DEX3_ARTICULATION,
)

UNITREE_G1_29DOF_DEX3_BACKPACK_CFG = deepcopy(UNITREE_G1_29DOF_DEX3_CFG)
UNITREE_G1_29DOF_DEX3_BACKPACK_CFG.spec_fn = get_g1_dex3_backpack_spec


H1_XML = ROBOTS_DIR / "unitree" / "h1_description" / "xmls" / "h1.xml"


def get_h1_spec() -> mujoco.MjSpec:
    spec = mujoco.MjSpec.from_file(str(H1_XML))
    for body in spec.bodies:
        for index, geom in enumerate(body.geoms):
            if not geom.name:
                geom.name = f"{body.name}_collision{index}"
    for body_name, side in zip(("left_ankle_link", "right_ankle_link"), ("left", "right"), strict=True):
        body = spec.body(body_name)
        body.geoms[-1].name = f"{side}_foot_collision"
        body.add_site(name=f"{side}_foot", size=[0.01])
    torso = spec.body("torso_link")
    torso.add_site(name="imu", size=[0.01])
    spec.add_sensor(name="imu_lin_vel", type=mujoco.mjtSensor.mjSENS_VELOCIMETER,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.add_sensor(name="imu_ang_vel", type=mujoco.mjtSensor.mjSENS_GYRO,
                    objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    return spec


H1_COLLISION = CollisionCfg(
    geom_names_expr=(".*_collision.*",), contype=1, conaffinity=1,
    condim={"^(left|right)_foot_collision$": 3, ".*": 1},
    priority={"^(left|right)_foot_collision$": 1, ".*": 0},
    friction={"^(left|right)_foot_collision$": (0.6,)},
)

UNITREE_H1_CFG = EntityCfg(
    init_state=EntityCfg.InitialStateCfg(
        pos=(0.0, 0.0, 1.05),
        joint_pos={".*": 0.0, ".*_hip_pitch_joint": -0.28, ".*_knee_joint": 0.79,
                   ".*_ankle_joint": -0.52, ".*_shoulder_pitch_joint": 0.28, ".*_elbow_joint": 0.52},
        joint_vel={".*": 0.0},
    ),
    collisions=(H1_COLLISION,), spec_fn=get_h1_spec,
    articulation=EntityArticulationInfoCfg(
        actuators=(
            BuiltinPositionActuatorCfg(target_names_expr=(".*_hip_yaw_joint", ".*_hip_roll_joint"),
                                       stiffness=150.0, damping=5.0, effort_limit=300.0),
            BuiltinPositionActuatorCfg(target_names_expr=(".*_hip_pitch_joint", ".*_knee_joint", "torso_joint"),
                                       stiffness=200.0, damping=5.0, effort_limit=300.0),
            BuiltinPositionActuatorCfg(target_names_expr=(".*_ankle_joint",), stiffness=20.0,
                                       damping=4.0, effort_limit=100.0),
            BuiltinPositionActuatorCfg(target_names_expr=(".*_shoulder_.*_joint", ".*_elbow_joint"),
                                       stiffness=40.0, damping=10.0, effort_limit=300.0),
        ), soft_joint_pos_limit_factor=0.9,
    ),
)
"""Unitree G1 configuration for MJLab.

Use ``deepcopy(UNITREE_G1_29DOF_CFG)`` before modifying the configuration.
"""


G1_ACTION_SCALE: dict[str, float] = {}
for a in G1_ARTICULATION.actuators:
    assert isinstance(a, BuiltinPositionActuatorCfg)
    e = a.effort_limit
    s = a.stiffness
    names = a.target_names_expr
    assert e is not None
    for n in names:
        G1_ACTION_SCALE[n] = 0.25 * e / s

G1_DEX3_ACTION_SCALE = dict(G1_ACTION_SCALE)
for actuator in (G1_DEX3_THUMB_ROOT_ACTUATOR, G1_DEX3_FINGER_ACTUATOR):
    assert actuator.effort_limit is not None
    for name in actuator.target_names_expr:
        G1_DEX3_ACTION_SCALE[name] = 0.25 * actuator.effort_limit / actuator.stiffness

G1_DEX3_BACKPACK_ACTION_SCALE = dict(G1_DEX3_ACTION_SCALE)


if __name__ == "__main__":
    import mujoco.viewer as viewer
    from mjlab.entity.entity import Entity

    robot = Entity(deepcopy(UNITREE_G1_29DOF_CFG))

    viewer.launch(robot.spec.compile())
