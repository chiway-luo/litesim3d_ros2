#!/usr/bin/env python3
"""Write the eight-triangle unit wedge as binary STL (supported by RViz)."""
from pathlib import Path
import struct


vertices = [(-.5, -.5, 0), (.5, -.5, 0), (.5, .5, 0), (-.5, .5, 0),
            (-.5, -.5, 1), (-.5, .5, 1)]
faces = [(0, 2, 1), (0, 3, 2), (0, 4, 5), (0, 5, 3),
         (0, 1, 4), (3, 5, 2), (4, 1, 2), (4, 2, 5)]
data = bytearray(b"LiteSim tilt wedge; Apache-2.0".ljust(80, b"\0"))
data.extend(struct.pack("<I", len(faces)))
for face in faces:
    a, b, c = [vertices[i] for i in face]
    u, v = [b[i]-a[i] for i in range(3)], [c[i]-a[i] for i in range(3)]
    normal = [u[1]*v[2]-u[2]*v[1], u[2]*v[0]-u[0]*v[2], u[0]*v[1]-u[1]*v[0]]
    length = sum(x*x for x in normal)**.5
    data.extend(struct.pack("<12fH", *[x/length for x in normal], *a, *b, *c, 0))
(Path(__file__).resolve().parents[1] / "meshes/unit_wedge.stl").write_bytes(data)
