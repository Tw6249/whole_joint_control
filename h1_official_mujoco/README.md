# Unitree H1 MuJoCo Model

This directory contains the official Unitree H1 MJCF model copied from:

https://github.com/unitreerobotics/unitree_mujoco/tree/main/unitree_robots/h1

Downloaded on 2026-05-11 from `unitreerobotics/unitree_mujoco` main branch at commit:

`517e161b4a89d1a62831357314d8aa6d90d9c18d`

The upstream repository describes `unitree_robots` as MJCF description files for robots supported by Unitree SDK2, and notes that motor numbering corresponds to the real robot hardware. This makes it a better baseline for full-body H1 simulation than the local `h1_single_knee` model, which is intentionally reduced to one joint.

## Files

- `h1.xml`: full-body H1 robot model.
- `scene.xml`: flat-ground MuJoCo scene including `h1.xml`.
- `scene_terrain.xml`: terrain scene including `h1.xml`.
- `assets/`: STL mesh files required by `h1.xml`.
- `UNITREE_MUJOCO_LICENSE`: upstream BSD-3-Clause license.

## Smoke Test

The model has been checked locally with Python MuJoCo:

```powershell
@'
import mujoco
for name in ["h1.xml", "scene.xml", "scene_terrain.xml"]:
    model = mujoco.MjModel.from_xml_path(f"eid_control_package/h1_official_mujoco/{name}")
    print(name, model.nq, model.nv, model.nu)
'@ | python -
```

Expected compiled sizes:

- `h1.xml`: `nq=27`, `nv=26`, `nu=20`
- `scene.xml`: `nq=27`, `nv=26`, `nu=20`
- `scene_terrain.xml`: `nq=27`, `nv=26`, `nu=20`
