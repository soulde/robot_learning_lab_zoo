from pathlib import Path
import re


ASSET_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ASSET_ROOT / "soulde_robot_zoo"
ROBOTS_ROOT = ASSET_ROOT / "robots"


def _assert_complete_urdf(robot_dir: Path, urdf_name: str) -> None:
    urdf_path = robot_dir / "urdf" / urdf_name

    assert urdf_path.is_file()
    urdf = urdf_path.read_text(encoding="utf-8")
    mesh_filenames = re.findall(r'<mesh\s+filename="([^"]+)"', urdf)

    assert mesh_filenames
    for filename in mesh_filenames:
        assert (urdf_path.parent / filename).resolve().is_file(), filename


def test_asset_package_owns_complete_dr02_models() -> None:
    assert (PACKAGE_ROOT / "__init__.py").is_file()
    assert (PACKAGE_ROOT / "dr02.py").is_file()
    assert (PACKAGE_ROOT / "mujoco_gui.py").is_file()
    assert (PACKAGE_ROOT / "configs" / "dr02_init_control.json").is_file()
    _assert_complete_urdf(ROBOTS_ROOT / "dr02_standard", "dr02_std.urdf")
    _assert_complete_urdf(ROBOTS_ROOT / "dr02_pro", "DR02-pro.urdf")


def test_asset_package_does_not_depend_on_training_package() -> None:
    python_sources = list(PACKAGE_ROOT.glob("*.py"))

    assert python_sources
    for source in python_sources:
        contents = source.read_text(encoding="utf-8")
        assert "robot_lab" not in contents, source
        assert "chocolate_training" not in contents, source
