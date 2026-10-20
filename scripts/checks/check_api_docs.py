#!/usr/bin/env python3
"""Check public IDL, declared parameter coverage and generated plugin tables."""
import ast
import importlib.util
import json
from pathlib import Path
import re

ROOT = Path(__file__).resolve().parents[2]


def launch_arguments(path):
    tree = ast.parse(path.read_text())
    names = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
            if node.func.id == "DeclareLaunchArgument" and node.args and isinstance(node.args[0], ast.Constant):
                names.add(node.args[0].value)
        if isinstance(node, ast.Assign) and any(isinstance(t, ast.Name) and t.id == "defaults" for t in node.targets):
            if isinstance(node.value, ast.Dict):
                names.update(key.value for key in node.value.keys if isinstance(key, ast.Constant))
    return names


def check(root=ROOT):
    failures, coverage = [], {}
    interface_text = (root / "docs/api/消息与服务.md").read_text()
    idls = sorted((root / "litesim3d_interfaces").glob("*/*"))
    idls = [p for p in idls if p.suffix in (".msg", ".srv")]
    for path in idls:
        if "```text\n" + path.read_text().strip() + "\n```" not in interface_text:
            failures.append(f"IDL missing or stale: {path.relative_to(root)}")
    reference = (root / "docs/api/启动与节点参数.md").read_text()
    launches = [
        "litesim3d_bringup/launch/simulation.launch.py",
        "litesim3d_ros2/launch/demo.launch.py",
        "litesim3d_ros2/launch/world.launch.py",
        "examples/wheel_car_4w/wheel_car_4w_sim/launch/wheel_car_4w.launch.py",
    ]
    for relative in launches:
        names = launch_arguments(root / relative)
        missing = sorted(name for name in names if f"`{name}`" not in reference)
        coverage[relative] = len(names)
        failures += [f"{relative}: undocumented launch argument {name}" for name in missing]
    for source in ("sim_node", "sim_world_node"):
        relative = f"litesim3d_ros2/src/{source}.cpp"
        names = set(re.findall(r'declare_parameter<[^;]*?>\(\s*"([^"]+)"', (root / relative).read_text()))
        coverage[relative] = len(names)
        failures += [f"{relative}: undocumented node parameter {name}" for name in sorted(names)
                     if f"`{name}`" not in reference and f"`{name}=" not in reference]
    spec = importlib.util.spec_from_file_location("plugin_docs", root / "scripts/docs/generate_plugin_reference.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for relative, content in module.generated(root).items():
        if not (root / relative).is_file() or (root / relative).read_text() != content:
            failures.append(f"generated reference stale: {relative}")
    return {"status": "FAIL" if failures else "PASS", "idl_files": len(idls),
            "declared_parameters": coverage, "failures": failures,
            "scope": "IDL text, parameter-name coverage, required-field metadata and generated tables; "
                     "runtime defaults are separately checked by installed plugin contract tests."}


def main():
    result = check()
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return int(bool(result["failures"]))


if __name__ == "__main__":
    raise SystemExit(main())
