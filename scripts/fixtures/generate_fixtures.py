#!/usr/bin/env python3
"""Deterministically regenerate the checked-in M1 model and mesh fixtures."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT
import math

ROOT = REPO_ROOT / "litesim3d_core/resources"

def write(relative, content):
    path = ROOT / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content.strip() + "\n", encoding="utf-8")

vertices = [(-.5,-.5,-.5),(.5,-.5,-.5),(.5,.5,-.5),(-.5,.5,-.5),
            (-.5,-.5,.5),(.5,-.5,.5),(.5,.5,.5),(-.5,.5,.5)]
triangles = [(0,2,1),(0,3,2),(4,5,6),(4,6,7),(0,1,5),(0,5,4),
             (3,7,6),(3,6,2),(0,4,7),(0,7,3),(1,2,6),(1,6,5)]
write("meshes/cube.obj", "\n".join(["v " + " ".join(map(str,v)) for v in vertices] +
                                     ["f " + " ".join(str(i+1) for i in t) for t in triangles]))
stl = ["solid cube"]
for tri in triangles:
    a,b,c = [vertices[i] for i in tri]
    u=[b[i]-a[i] for i in range(3)]; v=[c[i]-a[i] for i in range(3)]
    n=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]]
    stl += ["facet normal " + " ".join(map(str,n)),"outer loop"]
    stl += ["vertex " + " ".join(map(str,vertices[i])) for i in tri]
    stl += ["endloop", "endfacet"]
write("meshes/cube.stl", "\n".join(stl + ["endsolid cube"]))
write("meshes/cube.dae", f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
<asset><created>2026-10-17T00:00:00Z</created><modified>2026-10-17T00:00:00Z</modified><unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
<library_geometries><geometry id="cube"><mesh>
<source id="positions"><float_array id="array" count="24">{' '.join(str(x) for v in vertices for x in v)}</float_array>
<technique_common><accessor source="#array" count="8" stride="3"><param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
<vertices id="verts"><input semantic="POSITION" source="#positions"/></vertices>
<triangles count="12"><input semantic="VERTEX" source="#verts" offset="0"/><p>{' '.join(str(i) for t in triangles for i in t)}</p></triangles>
</mesh></geometry></library_geometries>
<library_visual_scenes><visual_scene id="scene"><node id="cube_node"><instance_geometry url="#cube"/></node></visual_scene></library_visual_scenes>
<scene><instance_visual_scene url="#scene"/></scene></COLLADA>''')

def link(name, geometry, xyz="0 0 0", rpy="0 0 0", inertial=False):
    inert = '<inertial><mass value="10"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial>' if inertial else ''
    return f'<link name="{name}">{inert}' + ''.join(
        f'<{role}><origin xyz="{xyz}" rpy="{rpy}"/><geometry>{geometry}</geometry></{role}>'
        for role in ('visual','collision')) + '</link>'

def joint(name, parent, child, kind="fixed", xyz="0 0 0", axis="0 1 0", extra=""):
    return f'<joint name="{name}" type="{kind}"><parent link="{parent}"/><child link="{child}"/><origin xyz="{xyz}"/><axis xyz="{axis}"/>{extra}</joint>'

def robot(body, footprint=False):
    parts = ['<robot name="r01" version="1.0">']
    if footprint: parts += ['<link name="base_footprint"/>',joint('base_fixed','base_footprint','base_link',xyz="0 0 0.2")]
    parts += [link('base_link',body,'0 0 0.3',inertial=True)]
    for side,y in [('left',.25),('right',-.25)]:
        parts += [link(f'{side}_wheel','<cylinder radius="0.1" length="0.05"/>',rpy=f'{math.pi/2} 0 0'),
                  joint(f'{side}_wheel_joint','base_link',f'{side}_wheel','continuous',f'0 {y} 0.1',extra='<limit effort="100" velocity="1000"/>')]
    parts += [link('lidar_link','<cylinder radius="0.04" length="0.08"/>'),joint('lidar_fixed','base_link','lidar_link',xyz='0.2 0 0.65'),'</robot>']
    return '\n'.join(parts)

write('urdf/r01.urdf',robot('<box size="0.8 0.6 0.4"/>'))
write('urdf/r01_footprint.urdf',robot('<box size="0.8 0.6 0.4"/>',True))
for ext in ['obj','stl','dae']:
    write(f'urdf/r02_{ext}.urdf',robot(f'<mesh filename="../meshes/cube.{ext}" scale="0.8 0.6 0.4"/>'))
parts=['<robot name="r03">','<link name="base_link"/>']
for name in ['arm','slider','follower','chain','tip']:parts += [link(name,'<box size="0.1 0.1 0.1"/>')]
parts += [joint('main','base_link','arm','revolute','0.2 0 0.5','1 1 1','<limit lower="-1" upper="1" effort="1" velocity="2"/>'),
          joint('slide','arm','slider','prismatic','0 0 0','1 0 0','<limit lower="0" upper="1" effort="1" velocity="0.5"/>'),
          joint('mimic','base_link','follower','continuous',extra='<mimic joint="main" multiplier="2" offset="0.1"/>'),
          joint('mimic_chain','follower','chain','continuous',extra='<mimic joint="mimic" multiplier="0.5" offset="0.2"/>'),
          joint('fixed','slider','tip',xyz='0 0 0.2'),'</robot>']
write('urdf/r03.urdf','\n'.join(parts))
write('urdf/body.xacro','''<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:macro name="body" params="length"><link name="base_link"><visual><geometry><box size="${length} 0.6 0.4"/></geometry></visual><collision><geometry><box size="${length} 0.6 0.4"/></geometry></collision></link></xacro:macro></robot>''')
write('urdf/r04.xacro','''<robot name="xacro_fixture" xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="length" default="0.8"/><xacro:include filename="body.xacro"/><xacro:body length="$(arg length)"/></robot>''')
angles=', '.join(f'{-15+30*i/31:.9f}' for i in range(32))
write('config/demo.yaml',f'''
schema_version: 1
units: m_rad_s
simulation:
  step_size: 0.005
  seed: 42
  odom_rate: 50
  command_timeout: 0.5
  real_time_factor: 1.0
  threads: 1
robot:
  id: robot_0
  urdf: ../urdf/r01.urdf
  base_link: base_link
  motion_model: differential_drive
  initial_pose: {{xyz: [0, 0, 0], rpy: [0, 0, 0]}}
  drive:
    wheel_radius: 0.1
    wheel_separation: 0.5
    max_v: 1.0
    max_w: 2.0
    acceleration: 0.5
    deceleration: 1.0
    angular_acceleration: 2.0
    support_links: [left_wheel, right_wheel]
lidars:
  - name: lidar
    link: lidar_link
    topic: points_raw
    rate_hz: 10
    columns: 1024
    rings: 32
    vertical_angles_deg: [{angles}]
    min_range_m: 0.1
    max_range_m: 100
    scan_mode: snapshot
    point_layout: xyzirt_v1
    self_filter: sensor_link
entities:
  - id: ground
    pose: {{xyz: [0, 0, -0.1]}}
    geometry: {{type: box, size: [30, 30, 0.2]}}
    support: true
  - id: front_wall
    pose: {{xyz: [5.005, 0, 1.5]}}
    geometry: {{type: box, size: [0.01, 16, 3]}}
  - id: left_wall
    pose: {{xyz: [0, 5, 1.5]}}
    geometry: {{type: box, size: [10, 0.1, 3]}}
  - id: column
    pose: {{xyz: [2.5, -2, 1]}}
    geometry: {{type: cylinder, radius: 0.3, length: 2}}
  - id: low_obstacle
    pose: {{xyz: [2, 1, 0.2]}}
    geometry: {{type: box, size: [0.6, 0.6, 0.4]}}
  - id: overhead
    pose: {{xyz: [3, 0, 2]}}
    geometry: {{type: box, size: [2, 3, 0.2]}}
''')
