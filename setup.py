"""Installation script for the ``soulde_robot_zoo`` extension."""

from pathlib import Path

import toml
from setuptools import setup


EXTENSION_ROOT = Path(__file__).resolve().parent
EXTENSION_DATA = toml.load(EXTENSION_ROOT / "config" / "extension.toml")

setup(
    name="soulde_robot_zoo",
    version=EXTENSION_DATA["package"]["version"],
    author=EXTENSION_DATA["package"]["author"],
    maintainer=EXTENSION_DATA["package"]["maintainer"],
    description=EXTENSION_DATA["package"]["description"],
    keywords=EXTENSION_DATA["package"]["keywords"],
    packages=["soulde_robot_zoo"],
    package_data={"soulde_robot_zoo": ["configs/*.json"]},
    install_requires=["mujoco>=3.10.0", "numpy>=1.23"],
    entry_points={"console_scripts": ["mujoco-gui=soulde_robot_zoo.mujoco_gui:main"]},
    include_package_data=True,
    python_requires=">=3.10",
    zip_safe=False,
)
