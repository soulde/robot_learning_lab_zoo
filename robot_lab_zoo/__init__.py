"""Robot descriptions and simulation assets for Isaac Lab and MuJoCo."""

from pathlib import Path


ROBOT_LAB_ZOO_DIR = Path(__file__).resolve().parents[1]
"""Root directory of the Robot Lab Zoo extension."""

ROBOTS_DIR = ROBOT_LAB_ZOO_DIR / "robots"
"""Directory containing all robot descriptions."""

CONFIGS_DIR = Path(__file__).resolve().parent / "configs"
"""Directory containing runtime configuration files."""

DR02_INIT_CONTROL_CONFIG = CONFIGS_DIR / "dr02_init_control.json"

DR02_STANDARD_DIR = ROBOTS_DIR / "deeprobotics" / "dr02_standard_description"
"""Directory containing the DR02 Standard model."""

DR02_PRO_DIR = ROBOTS_DIR / "deeprobotics" / "dr02_pro_description"
"""Directory containing the DR02 Pro model."""

DR02_STANDARD_URDF = DR02_STANDARD_DIR / "urdf" / "dr02_std.urdf"
DR02_PRO_URDF = DR02_PRO_DIR / "urdf" / "dr02_pro.urdf"

DR02_STANDARD_MJCF = DR02_STANDARD_DIR / "mjcf" / "dr02.xml"
DR02_PRO_POSITION_MJCF = DR02_PRO_DIR / "mjcf" / "dr02_pos.xml"
DR02_PRO_TORQUE_MJCF = DR02_PRO_DIR / "mjcf" / "dr02_torque.xml"

from .assets.deeprobotics import (  # noqa: E402, F401
    DEEPROBOTICS_DR02_PRO_CFG as DR02_PRO_CFG,
    DEEPROBOTICS_DR02_STANDARD_CFG as DR02_STANDARD_CFG,
)
