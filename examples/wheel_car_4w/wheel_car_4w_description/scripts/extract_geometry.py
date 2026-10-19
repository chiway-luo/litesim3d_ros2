"""Derive both drive and encoder geometry from the unchanged mechanical URDF."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

robot = ET.parse(sys.argv[1]).getroot()
joints = {j.get("name"): j for j in robot.findall("joint")}
links = {l.get("name"): l for l in robot.findall("link")}
left, right = (joints[n] for n in ("rear_left_wheel_joint", "rear_right_wheel_joint"))
positions = [list(map(float, j.find("origin").get("xyz").split())) for j in (left, right)]
radii = [float(links[j.find("child").get("link")].find("collision/geometry/cylinder").get("radius")) for j in (left, right)]
assert radii[0] == radii[1] and positions[0][1] == -positions[1][1]
output = Path(sys.argv[2])
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(f'''# 构建时从原始 wheel_car_4w.urdf 提取；不要手改生成文件。
# 轮半径，m；驱动和编码器解算共用同一值。
wheel_radius_m: {radii[0]}
# 左右轮中心距，m；左侧为 +y。
wheel_separation_m: {positions[0][1] - positions[1][1]}
# 同侧前轮由 URDF mimic 同步，不重复列入独立驱动关节。
left_joints: [rear_left_wheel_joint]
right_joints: [rear_right_wheel_joint]
''')
