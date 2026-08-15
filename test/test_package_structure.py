from pathlib import Path
import re


ASSET_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = ASSET_ROOT / "robot_lab_zoo"
ROBOTS_ROOT = ASSET_ROOT / "robots"

EXPECTED_MANUFACTURERS = {
    "agibot",
    "booster",
    "ddt",
    "deeprobotics",
    "fftai",
    "magiclab",
    "openloong",
    "roboparty",
    "robotera",
    "unitree",
    "zsibot",
}

EXPECTED_ASSET_MODULES = {
    "agibot.py",
    "booster.py",
    "ddtrobot.py",
    "deeprobotics.py",
    "fftai.py",
    "magiclab.py",
    "openloong.py",
    "roboparty.py",
    "robotera.py",
    "unitree.py",
    "zsibot.py",
}


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
    assert (PACKAGE_ROOT / "assets" / "deeprobotics.py").is_file()
    assert (PACKAGE_ROOT / "mujoco_gui.py").is_file()
    assert (PACKAGE_ROOT / "configs" / "dr02_init_control.json").is_file()
    _assert_complete_urdf(ROBOTS_ROOT / "deeprobotics" / "dr02_standard_description", "dr02_std.urdf")
    _assert_complete_urdf(ROBOTS_ROOT / "deeprobotics" / "dr02_pro_description", "dr02_pro.urdf")


def test_zoo_contains_all_manufacturers_and_asset_modules() -> None:
    manufacturers = {path.name for path in ROBOTS_ROOT.iterdir() if path.is_dir()}
    asset_modules = {path.name for path in (PACKAGE_ROOT / "assets").glob("*.py") if path.name != "__init__.py"}

    assert manufacturers == EXPECTED_MANUFACTURERS
    assert asset_modules == EXPECTED_ASSET_MODULES


def test_asset_package_does_not_depend_on_training_package() -> None:
    python_sources = list(PACKAGE_ROOT.rglob("*.py"))

    assert python_sources
    for source in python_sources:
        contents = source.read_text(encoding="utf-8")
        assert "from robot_lab." not in contents, source
        assert "import robot_lab." not in contents, source
        assert "chocolate_training" not in contents, source
