# Soulde Robot Zoo

Robot descriptions and simulation assets shared by Isaac Lab and MuJoCo projects.

Models are grouped by manufacturer, while Isaac Lab configurations live in the
matching module under `soulde_robot_zoo/assets/`:

```text
robots/<manufacturer>/<model>/{urdf,meshes,mjcf}
soulde_robot_zoo/assets/<manufacturer>.py
```

The zoo currently contains 24 models from AgiBot, Booster, DDT, Deep Robotics,
Fourier, MagicLab, OpenLoong, RoboParty, RobotEra, Unitree, and Zsibot.

The package exposes stable paths for each model:

```python
from soulde_robot_zoo import DR02_PRO_DIR, DR02_STANDARD_DIR
```

Use the bundled MuJoCo viewer with:

```bash
mujoco-gui robots/deeprobotics/dr02_standard_description/mjcf/flat_scene.xml \
  --config soulde_robot_zoo/configs/dr02_init_control.json
```
