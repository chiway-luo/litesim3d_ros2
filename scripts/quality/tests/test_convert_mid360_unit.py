"""CAD conversion logic units with an explicitly synthetic OCP interface fixture.

These do not claim to tessellate the real CAD or replace missing tracked assets.
The converter's coordinate/material/winding/report logic runs on four tiny faces.
"""
import gzip
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import runpy
import struct
import sys
import types
from types import SimpleNamespace as NS
from unittest.mock import Mock
import xml.etree.ElementTree as ET

import pytest


SOURCE = Path(__file__).resolve().parents[3] / 'examples/dual_mid360/dual_mid360_description/tools/convert_mid360_step.py'


class Point:
    def __init__(self, values):
        self.values = values
    def Transformed(self, transform):
        assert transform == 'identity-transform'
        return self
    def Coord(self):
        return self.values


class Mesh:
    def __init__(self, points, triangles):
        self.points, self.triangles = points, triangles
    def NbNodes(self):
        return len(self.points)
    def Node(self, i):
        return Point(self.points[i - 1])
    def Normal(self, i):
        assert 1 <= i <= len(self.points)
        return Point((1., 2., 3.))
    def NbTriangles(self):
        return len(self.triangles)
    def Triangle(self, i):
        return NS(Value=lambda j: self.triangles[i - 1][j - 1])


@pytest.fixture
def conversion(monkeypatch, tmp_path):
    faces = [
        NS(mesh=Mesh([(0., 0., 0.), (1., 0., 0.), (0., 1., 0.)], [(1, 2, 3), (1, 1, 2)]), Orientation=lambda: 'forward'),
        NS(mesh=Mesh([(0., 0., 1.), (1., 0., 1.), (0., 1., 1.)], [(1, 2, 3)]), Orientation=lambda: 'forward'),
        NS(mesh=Mesh([(0., 20., 0.), (1., 20., 0.), (0., 20., 1.)], [(1, 2, 3)]), Orientation=lambda: 'reversed'),
        NS(mesh=Mesh([(40., -3., 0.), (42., -3., 0.), (40., -1., 0.)], [(1, 2, 3)]), Orientation=lambda: 'forward')]
    class Explorer:
        def __init__(self, shape, kind):
            assert shape is faces and kind == 'face'
            self.index = 0
        def More(self):
            return self.index < len(faces)
        def Current(self):
            return faces[self.index]
        def Next(self):
            self.index += 1
    reader = Mock()
    reader.ReadFile.return_value = 'done'
    reader.OneShape.return_value = faces
    tessellator = Mock()
    triangulation = Mock(side_effect=lambda face, location: face.mesh)
    normals = Mock()
    symbols = {
        'OCP.BRep': {'BRep_Tool': NS(Triangulation_s=triangulation)},
        'OCP.BRepLib': {'BRepLib_ToolTriangulatedShape': NS(ComputeNormals_s=normals)},
        'OCP.BRepMesh': {'BRepMesh_IncrementalMesh': tessellator},
        'OCP.IFSelect': {'IFSelect_RetDone': 'done'},
        'OCP.STEPControl': {'STEPControl_Reader': Mock(return_value=reader)},
        'OCP.TopAbs': {'TopAbs_FACE': 'face', 'TopAbs_REVERSED': 'reversed'},
        'OCP.TopExp': {'TopExp_Explorer': Explorer},
        'OCP.TopLoc': {'TopLoc_Location': lambda: NS(Transformation=lambda: 'identity-transform')},
        'OCP.TopoDS': {'TopoDS': NS(Face=lambda face: face)},
    }
    monkeypatch.setitem(sys.modules, 'OCP', types.ModuleType('OCP'))
    for name, attributes in symbols.items():
        fake = types.ModuleType(name)
        vars(fake).update(attributes)
        monkeypatch.setitem(sys.modules, name, fake)
    spec = importlib.util.spec_from_file_location('convert_mid360_unit', SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.ROOT = tmp_path
    raw = b'explicit synthetic CAD unit fixture, not a STEP model'
    module.SOURCE_SHA256 = hashlib.sha256(raw).hexdigest()
    source = tmp_path / 'meshes/source/mid-360-asm.stp.gz'
    source.parent.mkdir(parents=True)
    source.write_bytes(gzip.compress(raw))
    return NS(module=module, faces=faces, reader=reader, tessellator=tessellator,
              triangulation=triangulation, normals=normals, raw=raw, source=source)


def test_cad_to_scan_axes_units_and_origin(conversion):
    module = conversion.module
    assert module.cad_to_scan((1000., 21.0829, 2000.)) == (-1., 2., 0.)
    assert module.cad_to_scan((0., -25.9171, 0.)) == pytest.approx((0., 0., -.047))


def test_collada_element_has_namespaced_tag_text_and_attributes(conversion):
    module = conversion.module
    parent = ET.Element('parent')
    node = module.element(parent, 'color', '0 1 0 1', sid='green')
    assert list(parent) == [node]
    assert node.tag == '{' + module.NS + '}color' and node.text == '0 1 0 1'
    assert node.attrib == {'sid': 'green'}


def test_main_converts_faces_materials_normals_winding_and_report(conversion, capsys):
    case = conversion
    case.module.main()
    case.reader.ReadFile.assert_called_once()
    # The decompressed source exists only inside the converter's temporary directory.
    assert not Path(case.reader.ReadFile.call_args.args[0]).exists()
    case.reader.TransferRoots.assert_called_once_with()
    case.tessellator.assert_called_once_with(case.faces, .15, False, .35, True)
    case.tessellator.return_value.Perform.assert_called_once_with()
    assert case.normals.call_count == 4
    output = case.module.ROOT / 'meshes/livox_mid360.dae'
    report = json.loads((case.module.ROOT / 'meshes/mid360_mesh.json').read_text())
    assert json.loads(capsys.readouterr().out) == report
    assert report['source_sha256'] == hashlib.sha256(case.raw).hexdigest()
    assert report['mesh_sha256'] == hashlib.sha256(output.read_bytes()).hexdigest()
    assert report['cad_faces'] == 4 and report['triangles'] == 4
    assert report['discarded_degenerate_triangles'] == 1
    assert report['materials'] == {'housing': 2, 'optical_window': 1, 'connector': 1}
    assert report['bounds_m'][0] == pytest.approx([-.042, 0., (-3. - 21.0829) / 1000.])
    assert report['bounds_m'][1] == pytest.approx([0., .001, (20. - 21.0829) / 1000.])
    ns = {'c': case.module.NS}
    document = ET.parse(output)
    assert document.find('.//c:up_axis', ns).text == 'Z_UP'
    assert document.find('.//c:unit', ns).attrib == {'name': 'meter', 'meter': '1'}
    geometries = {g.get('id'): g for g in document.findall('.//c:geometry', ns)}
    assert set(geometries) == {'housing_geo', 'optical_window_geo', 'connector_geo'}
    assert geometries['housing_geo'].find('.//c:triangles/c:p', ns).text == '0 1 2 3 4 5'
    assert geometries['optical_window_geo'].find('.//c:triangles/c:p', ns).text == '2 1 0'
    arrays = {a.get('id'): a.text for a in document.findall('.//c:float_array', ns)}
    assert arrays['housing_normals_array'] == '-1 3 2 -1 3 2 -1 3 2 -1 3 2 -1 3 2 -1 3 2'
    assert arrays['optical_window_normals_array'] == '1 -3 -2 1 -3 -2 1 -3 -2'
    assert {a.get('id'): a.get('count') for a in document.findall('.//c:float_array', ns)} == {
        name + '_' + kind + '_array': ('18' if name == 'housing' else '9')
        for name in ('housing', 'optical_window', 'connector') for kind in ('positions', 'normals')}
    instances = document.findall('.//c:instance_material', ns)
    assert {(i.get('symbol'), i.get('target')) for i in instances} == {
        ('housing', '#housing_mat'), ('optical_window', '#optical_window_mat'), ('connector', '#connector_mat')}


@pytest.mark.parametrize('failure,message', [('digest', 'Unexpected source CAD'), ('read', ''), ('mesh', 'Untessellated CAD face')])
def test_conversion_rejects_corrupt_source_reader_failure_and_unmeshed_faces(conversion, failure, message):
    case = conversion
    if failure == 'digest':
        case.module.SOURCE_SHA256 = 'wrong'
    elif failure == 'read':
        case.reader.ReadFile.return_value = 'failed'
    else:
        case.triangulation.side_effect = None
        case.triangulation.return_value = None
    with pytest.raises(AssertionError, match=message):
        case.module.main()
    assert not (case.module.ROOT / 'meshes/livox_mid360.dae').exists()
    assert not (case.module.ROOT / 'meshes/mid360_mesh.json').exists()


def test_wedge_generator_writes_closed_binary_unit_mesh_without_mutating_assets(monkeypatch):
    captured = []
    def write(path, data):
        captured.append((path, bytes(data)))
        return len(data)
    monkeypatch.setattr(Path, 'write_bytes', write)
    path = SOURCE.with_name('generate_wedge.py')
    runpy.run_path(str(path), run_name='__main__')
    assert len(captured) == 1
    target, data = captured[0]
    assert target == SOURCE.parents[1] / 'meshes/unit_wedge.stl'
    assert data[:80].rstrip(b'\0') == b'LiteSim tilt wedge; Apache-2.0'
    count, = struct.unpack_from('<I', data, 80)
    assert count == 8 and len(data) == 84 + 50 * count
    edges, volume = {}, 0.
    for index in range(count):
        *values, attribute = struct.unpack_from('<12fH', data, 84 + 50 * index)
        normal, a, b, c = [tuple(values[offset:offset+3]) for offset in (0, 3, 6, 9)]
        assert attribute == 0 and sum(v*v for v in normal) == pytest.approx(1.)
        assert all(math.isfinite(v) for v in values)
        for start, end in ((a, b), (b, c), (c, a)):
            edge = tuple(sorted((start, end)))
            edges[edge] = edges.get(edge, 0) + 1
        volume += (a[0] * (b[1]*c[2] - b[2]*c[1]) +
                   a[1] * (b[2]*c[0] - b[0]*c[2]) + a[2] * (b[0]*c[1] - b[1]*c[0])) / 6
    assert set(edges.values()) == {2}
    assert volume == pytest.approx(.5)
