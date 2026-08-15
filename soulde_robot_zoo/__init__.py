"""Robot descriptions and simulation assets for Isaac Lab and MuJoCo."""

from pathlib import Path


SOULDE_ROBOT_ZOO_DIR = Path(__file__).resolve().parents[1]
"""Root directory of the Soulde Robot Zoo extension."""

ROBOTS_DIR = SOULDE_ROBOT_ZOO_DIR / "robots"
"""Directory containing all robot descriptions."""

CONFIGS_DIR = Path(__file__).resolve().parent / "configs"
"""Directory containing runtime configuration files."""

DR02_INIT_CONTROL_CONFIG = CONFIGS_DIR / "dr02_init_control.json"

DR02_STANDARD_DIR = ROBOTS_DIR / "dr02_standard"
"""Directory containing the DR02 Standard model."""

DR02_PRO_DIR = ROBOTS_DIR / "dr02_pro"
"""Directory containing the DR02 Pro model."""

DR02_STANDARD_URDF = DR02_STANDARD_DIR / "urdf" / "dr02_std.urdf"
DR02_PRO_URDF = DR02_PRO_DIR / "urdf" / "DR02-pro.urdf"

DR02_STANDARD_MJCF = DR02_STANDARD_DIR / "mjcf" / "dr02.xml"
DR02_PRO_POSITION_MJCF = DR02_PRO_DIR / "mjcf" / "dr02_pos.xml"
DR02_PRO_TORQUE_MJCF = DR02_PRO_DIR / "mjcf" / "dr02_torque.xml"

from .dr02 import DR02_PRO_CFG, DR02_STANDARD_CFG  # noqa: E402, F401
