# Robot Learning Lab Zoo

Robot descriptions and simulation assets shared by Isaac Lab and MuJoCo projects.

Models are grouped by manufacturer, while backend configurations live under
`robot_learning_lab_zoo/assets/{isaaclab,mjlab}/`:

```text
robots/<manufacturer>/<model>/{urdf,meshes,mjcf}
robot_learning_lab_zoo/assets/isaaclab/<manufacturer>.py
robot_learning_lab_zoo/assets/mjlab/<manufacturer>.py
```

The zoo currently contains 24 models from AgiBot, Booster, DDT, Deep Robotics,
Fourier, MagicLab, OpenLoong, RoboParty, RobotEra, Unitree, and Zsibot.

Configuration objects are exported as module variables. Make a deep copy before
customizing an MJLab configuration:

```python
from copy import deepcopy

from robot_learning_lab_zoo.assets.mjlab.unitree import UNITREE_G1_29DOF_CFG

robot_cfg = deepcopy(UNITREE_G1_29DOF_CFG)
```

The package exposes stable paths for each model:

```python
from robot_learning_lab_zoo import DR02_PRO_DIR, DR02_STANDARD_DIR
```

Use the bundled MuJoCo viewer with:

```bash
mujoco-gui robots/deeprobotics/dr02_standard_description/mjcf/flat_scene.xml \
  --config robot_learning_lab_zoo/configs/dr02_init_control.json
```
