#!/usr/bin/env python3
"""Tessellate the recorded MID-360 CAD; OCP is only needed to regenerate the mesh.

Run with cadquery-ocp 8.0.1.1.0 installed in a separate conversion environment.
Runtime / colcon builds use the bundled DAE and do not import OCP.
"""
import gzip
import hashlib
import json
from pathlib import Path
import struct
import tempfile
import xml.etree.ElementTree as ET

from OCP.BRep import BRep_Tool
from OCP.BRepLib import BRepLib_ToolTriangulatedShape
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.IFSelect import IFSelect_RetDone
from OCP.STEPControl import STEPControl_Reader
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopExp import TopExp_Explorer
from OCP.TopLoc import TopLoc_Location
from OCP.TopoDS import TopoDS


ROOT = Path(__file__).resolve().parents[1]
SOURCE_SHA256 = "c28576589f443c77cec7a8574d9afb9632ff83e3e876c68b5eb5b8f88aaf9d45"
NS = "http://www.collada.org/2005/11/COLLADASchema"
ET.register_namespace("", NS)


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, "{" + NS + "}" + tag, attrs)
    node.text = text
    return node


def cad_to_scan(point):
    # CAD: mm, +Y up, connector on +X, base at Y=-25.9171.
    # Livox: m, +Z up, +X away from connector, O 47 mm above base.
    x, y, z = point
    return (-x / 1000, z / 1000, (y - 21.0829) / 1000)


def main():
    raw = gzip.decompress((ROOT / "meshes/source/mid-360-asm.stp.gz").read_bytes())
    assert hashlib.sha256(raw).hexdigest() == SOURCE_SHA256, "Unexpected source CAD"
    with tempfile.TemporaryDirectory(prefix="mid360-step-") as directory:
        source = Path(directory) / "mid360.stp"
        source.write_bytes(raw)
        reader = STEPControl_Reader()
        assert reader.ReadFile(str(source)) == IFSelect_RetDone
        reader.TransferRoots()
        shape = reader.OneShape()
    BRepMesh_IncrementalMesh(shape, 0.15, False, 0.35, True).Perform()
    groups = {name: {"positions": [], "normals": [], "triangles": []}
              for name in ("housing", "optical_window", "connector")}
    explorer = TopExp_Explorer(shape, TopAbs_FACE)
    face_count = 0
    discarded = 0
    while explorer.More():
        face = TopoDS.Face(explorer.Current())
        location = TopLoc_Location()
        mesh = BRep_Tool.Triangulation_s(face, location)
        assert mesh is not None, "Untessellated CAD face"
        transform = location.Transformation()
        points = [mesh.Node(i).Transformed(transform).Coord() for i in range(1, mesh.NbNodes()+1)]
        # Retain every CAD face. These display materials replace the CAD's engineering colors.
        kind = "optical_window" if max(p[1] for p in points) > 14.5 else "housing"
        if min(p[0] for p in points) > 32.3 and max(p[1] for p in points) < 0:
            kind = "connector"
        group = groups[kind]
        offset = len(group["positions"])
        group["positions"].extend(cad_to_scan(p) for p in points)
        # Assimp reads coordinates as float32. Remove only numerical slivers
        # that would violate the simulator's existing 1e-12 m² cross-product check.
        quantized = [struct.unpack("fff", struct.pack("fff", *(float(f"{v:.9g}") for v in cad_to_scan(p))))
                     for p in points]
        BRepLib_ToolTriangulatedShape.ComputeNormals_s(face, mesh)
        flip = -1 if face.Orientation() == TopAbs_REVERSED else 1
        for i in range(1, mesh.NbNodes()+1):
            nx, ny, nz = mesh.Normal(i).Transformed(transform).Coord()
            group["normals"].append((-nx*flip, nz*flip, ny*flip))
        for i in range(1, mesh.NbTriangles()+1):
            triangle = mesh.Triangle(i)
            indices = [triangle.Value(j)-1+offset for j in (1, 2, 3)]
            a, b, c = [quantized[j-offset] for j in indices]
            u, v = [b[j]-a[j] for j in range(3)], [c[j]-a[j] for j in range(3)]
            cross = (u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0])
            if sum(n*n for n in cross) <= (1.01e-12)**2:
                discarded += 1
                continue
            if flip < 0:
                indices.reverse()
            group["triangles"].extend(indices)
        face_count += 1
        explorer.Next()

    document = ET.Element("{" + NS + "}COLLADA", version="1.4.1")
    asset = element(document, "asset")
    contributor = element(asset, "contributor")
    element(contributor, "authoring_tool", "convert_mid360_step.py / Open CASCADE")
    element(asset, "created", "2026-10-23T00:00:00Z")
    element(asset, "modified", "2026-10-23T00:00:00Z")
    element(asset, "unit", name="meter", meter="1")
    element(asset, "up_axis", "Z_UP")
    effects, materials = element(document, "library_effects"), element(document, "library_materials")
    library = element(document, "library_geometries")
    scenes = element(document, "library_visual_scenes")
    scene = element(scenes, "visual_scene", id="mid360_scene")
    colors = {"housing": "0.72 0.74 0.76 1", "optical_window": "0.025 0.032 0.037 1",
              "connector": "0.20 0.22 0.24 1"}
    for name, data in groups.items():
        effect = element(effects, "effect", id=name + "_fx")
        technique = element(element(effect, "profile_COMMON"), "technique", sid="common")
        phong = element(technique, "phong")
        element(element(phong, "ambient"), "color", colors[name])
        element(element(phong, "diffuse"), "color", colors[name])
        element(element(phong, "specular"), "color", "0.25 0.25 0.25 1")
        element(element(phong, "shininess"), "float", "60" if name == "optical_window" else "20")
        material = element(materials, "material", id=name + "_mat", name=name)
        element(material, "instance_effect", url="#" + name + "_fx")
        geometry = element(library, "geometry", id=name + "_geo", name=name)
        mesh = element(geometry, "mesh")
        for kind in ("positions", "normals"):
            sid = name + "_" + kind
            source = element(mesh, "source", id=sid)
            values = data[kind]
            element(source, "float_array", " ".join(f"{v:.9g}" for p in values for v in p),
                    id=sid + "_array", count=str(len(values)*3))
            accessor = element(element(source, "technique_common"), "accessor",
                               source="#"+sid+"_array", count=str(len(values)), stride="3")
            for axis in "XYZ":
                element(accessor, "param", name=axis, type="float")
        vertices = element(mesh, "vertices", id=name + "_vertices")
        element(vertices, "input", semantic="POSITION", source="#"+name+"_positions")
        triangles = element(mesh, "triangles", count=str(len(data["triangles"])//3), material=name)
        element(triangles, "input", semantic="VERTEX", source="#"+name+"_vertices", offset="0")
        element(triangles, "input", semantic="NORMAL", source="#"+name+"_normals", offset="0")
        element(triangles, "p", " ".join(map(str, data["triangles"])))
        node = element(scene, "node", id=name)
        instance = element(node, "instance_geometry", url="#"+name+"_geo")
        binding = element(element(instance, "bind_material"), "technique_common")
        element(binding, "instance_material", symbol=name, target="#"+name+"_mat")
    element(element(document, "scene"), "instance_visual_scene", url="#mid360_scene")
    ET.indent(document)
    output = ROOT / "meshes/livox_mid360.dae"
    ET.ElementTree(document).write(output, encoding="utf-8", xml_declaration=True)
    points = [p for g in groups.values() for p in g["positions"]]
    report = {
        "source_sha256": SOURCE_SHA256,
        "mesh_sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "cad_faces": face_count,
        "discarded_degenerate_triangles": discarded,
        "triangles": sum(len(g["triangles"])//3 for g in groups.values()),
        "bounds_m": [[min(p[i] for p in points) for i in range(3)],
                     [max(p[i] for p in points) for i in range(3)]],
        "scan_origin_above_mount_m": 0.047,
        "cad_to_scan": "(-X, Z, Y-21.0829) / 1000",
        "linear_deflection_mm": 0.15,
        "angular_deflection_rad": 0.35,
        "materials": {k: len(v["triangles"])//3 for k, v in groups.items()},
    }
    (ROOT / "meshes/mid360_mesh.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
