from pathlib import Path
from xml.etree import ElementTree

import pytest

ZOO_ROOT = Path(__file__).resolve().parents[1]
G1_URDF_ROOT = ZOO_ROOT / "robots" / "unitree" / "g1_description" / "urdf"
G1_URDF = G1_URDF_ROOT / "g1_29dof_rev_1_0.urdf"
G1_DEX3_URDF = G1_URDF_ROOT / "g1_29dof_with_hand_rev_1_0.urdf"
DEX3_JOINT_SUFFIXES = {
    "thumb_0_joint",
    "thumb_1_joint",
    "thumb_2_joint",
    "index_0_joint",
    "index_1_joint",
    "middle_0_joint",
    "middle_1_joint",
}


def _actuated_joint_names(path: Path) -> list[str]:
    root = ElementTree.parse(path).getroot()
    return [
        str(joint.get("name"))
        for joint in root.findall("joint")
        if joint.get("type") in {"continuous", "revolute"}
    ]


def test_g1_urdfs_keep_the_existing_and_dex3_joint_sets_distinct() -> None:
    """Removing or mixing either model's hand joints must break this contract."""
    body_joints = _actuated_joint_names(G1_URDF)
    dex3_joints = _actuated_joint_names(G1_DEX3_URDF)
    left_hand_joints = {name.removeprefix("left_hand_") for name in dex3_joints if name.startswith("left_hand_")}
    right_hand_joints = {
        name.removeprefix("right_hand_") for name in dex3_joints if name.startswith("right_hand_")
    }

    assert len(body_joints) == 29
    assert all("_hand_" not in name for name in body_joints)
    assert len(dex3_joints) == 43
    assert left_hand_joints == DEX3_JOINT_SUFFIXES
    assert right_hand_joints == DEX3_JOINT_SUFFIXES


def test_mjlab_exports_a_distinct_g1_dex3_entity() -> None:
    """Pointing both public variants at one entity must break this contract."""
    pytest.importorskip("mjlab", exc_type=ImportError)
    from robot_learning_lab_zoo.assets.mjlab import UNITREE_G1_29DOF_CFG, UNITREE_G1_29DOF_DEX3_CFG

    assert UNITREE_G1_29DOF_DEX3_CFG is not UNITREE_G1_29DOF_CFG
    body_model = UNITREE_G1_29DOF_CFG.spec_fn().compile()
    dex3_model = UNITREE_G1_29DOF_DEX3_CFG.spec_fn().compile()
    assert body_model.njnt == 30  # free root plus 29 actuated joints
    assert dex3_model.njnt == 44  # free root plus 43 actuated joints


def test_isaaclab_dex3_configuration_adds_only_hand_actuation() -> None:
    """Dropping hand actuation or adding it to the old model must fail."""
    pytest.importorskip("isaaclab", exc_type=ImportError)
    from robot_learning_lab_zoo.assets.isaaclab.unitree import (
        UNITREE_G1_29DOF_ACTION_SCALE,
        UNITREE_G1_29DOF_CFG,
        UNITREE_G1_29DOF_DEX3_ACTION_SCALE,
        UNITREE_G1_29DOF_DEX3_CFG,
    )

    assert UNITREE_G1_29DOF_DEX3_CFG is not UNITREE_G1_29DOF_CFG
    assert UNITREE_G1_29DOF_CFG.spawn.asset_path.endswith("g1_29dof_rev_1_0.urdf")
    assert UNITREE_G1_29DOF_DEX3_CFG.spawn.asset_path.endswith("g1_29dof_with_hand_rev_1_0.urdf")
    assert "hands" not in UNITREE_G1_29DOF_CFG.actuators
    assert UNITREE_G1_29DOF_DEX3_CFG.actuators["hands"].joint_names_expr == [".*_hand_.*_joint"]
    assert all("_hand_" not in name for name in UNITREE_G1_29DOF_ACTION_SCALE)
    assert ".*_hand_thumb_0_joint" in UNITREE_G1_29DOF_DEX3_ACTION_SCALE
    assert ".*_hand_(?!thumb_0).*_joint" in UNITREE_G1_29DOF_DEX3_ACTION_SCALE
