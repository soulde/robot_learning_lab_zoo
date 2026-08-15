# Soulde Robot Zoo

Robot descriptions and simulation assets shared by Isaac Lab and MuJoCo projects.

The package exposes stable paths for each model:

```python
from soulde_robot_zoo import DR02_PRO_DIR, DR02_STANDARD_DIR
```

Use the bundled MuJoCo viewer with:

```bash
mujoco-gui robots/dr02_standard/mjcf/flat_scene.xml \
  --config configs/dr02_init_control.json
```
