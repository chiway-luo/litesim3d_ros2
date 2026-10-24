"""Geometric acceptance: rotated wheels touch the floor and every assembly has support."""
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import numpy as np
import pytest
import xacro
import yaml


def pose(xyz="0 0 0", rpy="0 0 0"):
    r, p, y = map(float, rpy.split())
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    result = np.eye(4)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                      [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    result[:3, 3] = list(map(float, xyz.split()))
    return result


def origin(element):
    value = element.find("origin")
    return np.eye(4) if value is None else pose(value.get("xyz", "0 0 0"), value.get("rpy", "0 0 0"))


def bounds(element, transform):
    world = transform @ origin(element)
    shape = element.find("geometry")[0]
    if shape.tag == "box":
        extents = np.abs(world[:3, :3]) @ (np.array(list(map(float, shape.get("size").split()))) / 2)
    elif shape.tag == "mesh":
        points = mesh_points(shape)
        points = points @ world[:3, :3].T + world[:3, 3]
        return points.min(axis=0), points.max(axis=0)
    else:
        assert shape.tag == "cylinder"
        axis = world[:3, 2]
        extents = (float(shape.get("radius")) * np.sqrt(np.maximum(0, 1-axis**2)) +
                   abs(axis) * float(shape.get("length")) / 2)
    return world[:3, 3] - extents, world[:3, 3] + extents


def mesh_points(mesh):
    package, relative = mesh.get("filename").removeprefix("package://").split("/", 1)
    path = Path(get_package_share_directory(package)) / relative
    if path.suffix == ".dae":
        ns = {"c": "http://www.collada.org/2005/11/COLLADASchema"}
        root = ET.parse(path).getroot()
        assert root.findtext("c:asset/c:up_axis", namespaces=ns) == "Z_UP"
        assert root.find("c:asset/c:unit", ns).get("meter") == "1"
        arrays = [np.fromstring(a.text, sep=" ").reshape(-1, 3)
                  for a in root.findall(".//c:float_array", ns) if a.get("id").endswith("_positions_array")]
        points = np.concatenate(arrays)
    else:
        assert path.suffix == ".stl"
        raw = path.read_bytes()
        count = struct.unpack_from("<I", raw, 80)[0]
        assert len(raw) == 84 + 50*count, "RViz requires binary STL"
        points = np.array([struct.unpack_from("<12fH", raw, 84+50*i)[3:12]
                           for i in range(count)]).reshape(-1, 3)
    return points * np.fromstring(mesh.get("scale", "1 1 1"), sep=" ")


def model(spin=0):
    share = Path(get_package_share_directory("dual_mid360_description"))
    root = ET.fromstring(xacro.process_file(str(share / "urdf/rover.urdf.xacro")).toxml())
    transforms = {"base_link": np.eye(4)}
    pending = list(root.findall("joint"))
    while pending:
        ready = [j for j in pending if j.find("parent").get("link") in transforms]
        assert ready, "URDF is not a connected tree"
        for joint in ready:
            transform = transforms[joint.find("parent").get("link")] @ origin(joint)
            if joint.get("type") == "continuous":
                assert joint.find("axis").get("xyz") == "0 1 0"
                transform = transform @ pose(rpy=f"0 {spin} 0")
            transforms[joint.find("child").get("link")] = transform
            pending.remove(joint)
    return root, transforms


@pytest.mark.parametrize("spin", [0, 0.7, math.pi/2, math.pi])
def test_rotating_wheels_remain_on_floor(spin):
    root, transforms = model(spin)
    sim = Path(get_package_share_directory("dual_mid360_bringup"))
    runtime = yaml.safe_load((sim / "config/runtime.yaml").read_text())
    assert runtime["terrain"]["enabled"] and runtime["terrain"]["clearance_m"] == 0
    for name in runtime["robot"]["support_links"]:
        link = root.find(f"./link[@name='{name}']")
        for element in [link.find("visual"), link.find("collision")]:
            lo, hi = bounds(element, transforms[name])
            assert abs(lo[2]) < 1e-9, (name, lo)
            assert hi[2] == pytest.approx(0.3)


def test_description_mounts_are_connected_and_have_no_plugins():
    root, transforms = model()
    assert root.find("litesim3d") is None
    solids = {link.get("name"): bounds(link.find("collision"), transforms[link.get("name")])
              for link in root.findall("link") if link.find("collision") is not None}
    angle = math.radians(15)
    for side, sign in [("front", 1), ("rear", -1)]:
        lidar = transforms[side + "_mid360_link"]
        assert lidar[:3, 3] == pytest.approx([sign * 0.33423504748, 0, 0.57652338007])
        assert lidar[:3, 0] == pytest.approx([sign * math.cos(angle), 0, -math.sin(angle)])
        stack = ["chassis_link", "deck_link", side + "_mount_foot",
                 side + "_mount_post", side + "_mount_adapter", side + "_mount_wedge"]
        for lower, upper in zip(stack, stack[1:]):
            lo_a, hi_a = solids[lower]
            lo_b, hi_b = solids[upper]
            assert hi_a[2] == pytest.approx(lo_b[2]), (lower, upper)
            assert np.all(np.minimum(hi_a[:2], hi_b[:2]) > np.maximum(lo_a[:2], lo_b[:2]))
    for name, (lo, _) in solids.items():
        assert lo[2] >= -1e-9, name


def test_real_cad_and_tilted_mounting_faces_meet():
    root, transforms = model()
    for side in ("front", "rear"):
        plate = transforms[side + "_mount_plate"]
        lidar = root.find(f"./link[@name='{side}_mid360_link']")
        assert len(lidar.findall("visual")) == 1
        mesh = lidar.find("visual/geometry/mesh")
        assert mesh.get("filename").endswith("/livox_mid360.dae")
        points = mesh_points(mesh)
        # Native metric CAD, including the actual protruding connector on -X.
        assert points[:, 2].min() == pytest.approx(-0.047, abs=2e-6)
        assert points[:, 2].max() == pytest.approx(0.013, abs=2e-4)
        assert points[:, 0].min() < -0.040
        assert np.ptp(points[:, 1]) == pytest.approx(0.065, abs=5e-4)
        relative = np.linalg.inv(plate) @ transforms[side + "_mid360_link"]
        mounted = points @ relative[:3, :3].T + relative[:3, 3]
        # Actual CAD underside, not just its collision box, touches plate top.
        assert mounted[:, 2].min() == pytest.approx(0.004, abs=2e-6)
        contacts = mounted[np.abs(mounted[:, 2]-0.004) < 2e-6]
        assert len(contacts) > 10
        assert np.ptp(contacts[:, 0]) > 0.06 and np.ptp(contacts[:, 1]) > 0.06
        wedge = root.find(f"./link[@name='{side}_mount_wedge']/visual/geometry/mesh")
        vertices = mesh_points(wedge)
        upper = vertices[np.isclose(vertices[:, 2], (0.05-vertices[:, 0])*math.tan(math.radians(15)))]
        relative = np.linalg.inv(plate) @ transforms[side + "_mount_wedge"]
        upper = upper @ relative[:3, :3].T + relative[:3, 3]
        assert len(upper) >= 4
        assert upper[:, 2] == pytest.approx(np.full(len(upper), -0.004), abs=1e-9)
        assert np.all(np.abs(upper[:, :2]) < 0.07), "wedge must fit under plate"


@pytest.mark.parametrize("world_name", ["workshop.sdf", "minimal.sdf"])
def test_workshop_has_no_floating_solids(world_name):
    share = Path(get_package_share_directory("dual_mid360_bringup"))
    world = ET.parse(share / "worlds" / world_name).getroot().find("world")
    solids = {}
    for item in world.findall("model"):
        if item.findtext("static") != "true":
            assert item.get("name") == "dual_mid360"
            assert item.findtext("pose") == "0 0 0 0 0 0"
            continue
        mpose = np.array(list(map(float, item.findtext("pose").split())))
        assert np.all(mpose[3:] == 0)
        for visual in item.findall("link/visual"):
            xyzrpy = np.array(list(map(float, visual.findtext("pose", "0 0 0 0 0 0").split())))
            assert np.all(xyzrpy[3:] == 0)
            size = np.array(list(map(float, visual.findtext("geometry/box/size").split())))
            center = mpose[:3] + xyzrpy[:3]
            solids[item.get("name") + "/" + visual.get("name")] = (center-size/2, center+size/2)
    floor = solids.pop("ground/floor")
    assert floor[1][2] == pytest.approx(0)
    supported = {"ground/floor": floor}
    pending = dict(solids)
    while pending:
        added = []
        for name, (lo, hi) in pending.items():
            assert lo[2] >= -1e-9, name
            # A part can rest on or join another part, but cannot hover above it.
            for low, high in supported.values():
                overlap = np.minimum(hi, high) - np.maximum(lo, low)
                if np.all(overlap >= -1e-9):
                    added.append(name)
                    break
        assert added, f"floating geometry: {list(pending)}"
        for name in added:
            supported[name] = pending.pop(name)
