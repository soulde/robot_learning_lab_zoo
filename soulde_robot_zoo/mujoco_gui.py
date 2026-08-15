"""Run a MuJoCo model in a GUI loop."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run a MuJoCo XML/MJCF model with GUI.")
    parser.add_argument("model", type=Path, help="Path to the MuJoCo XML/MJCF model.")
    parser.add_argument(
        "--config",
        type=Path,
        help="Path to a JSON config with initial actuator controls.",
    )
    parser.add_argument(
        "--realtime",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Sleep between steps to approximately match the model timestep.",
    )
    return parser.parse_args()


def load_init_ctrl(model: mujoco.MjModel, config_path: Path | None) -> np.ndarray:
    init_ctrl = np.zeros(model.nu)
    if config_path is None:
        return init_ctrl

    resolved_config_path = config_path.expanduser().resolve()
    if not resolved_config_path.is_file():
        raise FileNotFoundError(f"Config file not found: {resolved_config_path}")

    with resolved_config_path.open("r", encoding="utf-8") as file:
        config = json.load(file)

    ctrl_config = config.get("ctrl", {})
    if not isinstance(ctrl_config, dict):
        raise ValueError("Config field 'ctrl' must be an object mapping actuator names to values.")

    for actuator_name, ctrl_value in ctrl_config.items():
        actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        if actuator_id < 0:
            raise ValueError(f"Unknown actuator in config: {actuator_name}")
        init_ctrl[actuator_id] = float(ctrl_value)

    return init_ctrl


def main() -> None:
    args = parse_args()
    model_path = args.model.expanduser().resolve()
    if not model_path.is_file():
        raise FileNotFoundError(f"Model file not found: {model_path}")

    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    init_ctrl = load_init_ctrl(model, args.config)
    data.ctrl[:] = init_ctrl

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()
            data.ctrl[:] = init_ctrl
            mujoco.mj_step(model, data)
            viewer.sync()

            if args.realtime:
                sleep_time = model.opt.timestep - (time.time() - step_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)


if __name__ == "__main__":
    main()
