"""Installation script for the ``robot_lab_zoo`` extension."""

from pathlib import Path

import toml
from setuptools import setup


EXTENSION_ROOT = Path(__file__).resolve().parent
EXTENSION_DATA = toml.load(EXTENSION_ROOT / "config" / "extension.toml")

setup(
    name="robot_lab_zoo",
    version=EXTENSION_DATA["package"]["version"],
    author=EXTENSION_DATA["package"]["author"],
    maintainer=EXTENSION_DATA["package"]["maintainer"],
    description=EXTENSION_DATA["package"]["description"],
    keywords=EXTENSION_DATA["package"]["keywords"],
    packages=["robot_lab_zoo", "robot_lab_zoo.assets"],
    package_data={"robot_lab_zoo": ["configs/*.json"]},
    install_requires=["mujoco>=3.10.0", "numpy>=1.23"],
    entry_points={"console_scripts": ["mujoco-gui=robot_lab_zoo.mujoco_gui:main"]},
    include_package_data=True,
    python_requires=">=3.10",
    zip_safe=False,
)
