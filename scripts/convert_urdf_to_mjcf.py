"""Convert robot zoo URDF files to self-contained MJCF XML files."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import trimesh


REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOTS_DIR = REPO_ROOT / "robots"


def resolve_mesh_path(urdf_path: Path, filename: str) -> Path:
  if filename.startswith("package://"):
    relative_path = Path(filename.removeprefix("package://"))
    return urdf_path.parent.parent / Path(*relative_path.parts[1:])

  if filename.startswith("$(find "):
    relative_path = Path(filename.split(")/", maxsplit=1)[1])
    return urdf_path.parent.parent / relative_path

  mesh_path = Path(filename)
  if mesh_path.is_absolute():
    return mesh_path
  relative_to_urdf = (urdf_path.parent / mesh_path).resolve()
  if relative_to_urdf.exists():
    return relative_to_urdf
  return (urdf_path.parent.parent / mesh_path).resolve()


def convert_urdf(urdf_path: Path) -> Path:
  urdf_root = ET.parse(urdf_path).getroot()
  assets: dict[str, bytes] = {}
  mesh_names: dict[str, str] = {}

  for visual in urdf_root.findall(".//visual"):
    materials = visual.findall("material")
    for material in materials[1:]:
      visual.remove(material)

  for inertial in urdf_root.findall(".//inertial"):
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
      continue
    if float(mass.get("value", "0")) >= 1e-6:
      continue
    mass.set("value", "1e-6")
    for name in ("ixx", "iyy", "izz"):
      inertia.set(name, "1e-8")
    for name in ("ixy", "ixz", "iyz"):
      inertia.set(name, "0")

  for index, mesh in enumerate(urdf_root.findall(".//mesh")):
    filename = mesh.get("filename")
    if filename is None:
      continue

    mesh_path = resolve_mesh_path(urdf_path, filename)
    if not mesh_path.is_file():
      raise FileNotFoundError(f"Mesh referenced by {urdf_path} does not exist: {mesh_path}")

    model_dir = urdf_path.parent.parent
    if mesh_path.suffix.lower() == ".dae":
      converted_dir = model_dir / "meshes_mjlab"
      converted_dir.mkdir(exist_ok=True)
      converted_path = converted_dir / f"{mesh_path.stem}.obj"
      if not converted_path.exists():
        trimesh.load(mesh_path, force="scene").export(converted_path)
      mesh_path = converted_path

    asset_name = f"mesh_{index}{mesh_path.suffix}"
    assets[asset_name] = mesh_path.read_bytes()
    mesh_names[asset_name] = mesh_path.relative_to(model_dir).as_posix()
    mesh.set("filename", asset_name)

  spec = mujoco.MjSpec.from_string(ET.tostring(urdf_root, encoding="unicode"), assets=assets)
  spec.compiler.balanceinertia = True
  spec.compiler.boundmass = 1e-6
  spec.compiler.boundinertia = 1e-8
  root_body = spec.worldbody.first_body()
  if root_body is None:
    raise ValueError(f"URDF has no root body: {urdf_path}")
  root_body.add_freejoint()
  spec.compile()

  output_dir = urdf_path.parent.parent / "xmls"
  output_dir.mkdir(exist_ok=True)
  output_path = output_dir / f"{urdf_path.stem}.xml"
  spec.to_file(str(output_path))

  mjcf_tree = ET.parse(output_path)
  mjcf_root = mjcf_tree.getroot()
  compiler = mjcf_root.find("compiler")
  if compiler is None:
    compiler = ET.SubElement(mjcf_root, "compiler")
  compiler.set("meshdir", "..")

  for mesh in mjcf_root.findall("./asset/mesh"):
    filename = mesh.get("file")
    if filename in mesh_names:
      mesh.set("file", mesh_names[filename])

  ET.indent(mjcf_tree, space="  ")
  mjcf_tree.write(output_path, encoding="utf-8", xml_declaration=True)
  mujoco.MjSpec.from_file(str(output_path)).compile()
  return output_path


def main() -> None:
  urdf_paths = sorted(ROBOTS_DIR.glob("*/*/urdf/*.urdf"))
  skipped: list[tuple[Path, Exception]] = []
  for urdf_path in urdf_paths:
    try:
      output_path = convert_urdf(urdf_path)
    except (FileNotFoundError, ValueError) as error:
      skipped.append((urdf_path, error))
      print(f"SKIP {urdf_path.relative_to(REPO_ROOT)}: {error}")
      continue
    print(f"{urdf_path.relative_to(REPO_ROOT)} -> {output_path.relative_to(REPO_ROOT)}")

  if skipped:
    print(f"Skipped {len(skipped)} URDF file(s) with incomplete source assets.")


if __name__ == "__main__":
  main()
