#!/usr/bin/env python3
"""T-DEL-06: check local references, traceability and documented public defaults."""
import ast
import hashlib
import json
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from paths import REPO_ROOT, EVIDENCE_DIR, resolve_reference, python_sources
import re
import subprocess
from urllib.parse import unquote
import xml.etree.ElementTree as ET

ROOT = REPO_ROOT
OUT = EVIDENCE_DIR / "final"


def main():
    checks = []
    failures = []

    def check(label, passed, detail=""):
        checks.append({"check": label, "passed": bool(passed), "detail": detail})
        if not passed:
            failures.append(label + (": " + detail if detail else ""))

    docs = [ROOT / "README.md", *sorted((ROOT / "docs").rglob("*.md"))]
    links = 0
    for path in docs:
        for link in re.findall(r"\[[^\]]*\]\(([^)]+)\)", path.read_text()):
            if re.match(r"\w+://", link) or link.startswith(("#", "mailto:")):
                continue
            local = unquote(link.split("#")[0].strip("<>"))
            if local:
                links += 1
                check(f"link {path.name}: {local}", (path.parent / local).exists())
    specs = resolve_reference("docs/testing/测试用例与需求追踪.md").read_text()
    ids = re.findall(r"^\| (T-[A-Z0-9]{3}-\d{2}) .+；M[123]；", specs, re.M)
    check("140 unique specification IDs", len(ids) == len(set(ids)) == 140)
    index = json.loads((OUT / "test-index.json").read_text())
    check("82 unique M1 mappings", len(index["cases"]) == len({c["id"] for c in index["cases"]}) == 82)
    for case in index["cases"]:
        check(case["id"] + " requirements", bool(case["requirements"]))
        check(case["id"] + " executable mapping", bool(case["runs"]))
        for run in case["runs"]:
            check(case["id"] + " evidence exists", (resolve_reference(run["evidence"])).exists(), run["evidence"])
    for script in sorted((ROOT / "scripts").glob("*.sh")):
        result = subprocess.run(["bash", "-n", str(script)], capture_output=True, text=True)
        check("shell syntax " + script.name, result.returncode == 0, result.stderr)
    python_files = python_sources()
    for script in python_files:
        try:
            ast.parse(script.read_text(), filename=str(script))
            check("Python syntax " + script.name, True)
        except SyntaxError as exc:
            check("Python syntax " + script.name, False, str(exc))
    defaults = (ROOT / "litesim3d_core/resources/config/demo.yaml").read_text()
    for item in ("step_size: 0.005", "command_timeout: 0.5", "odom_rate: 50", "columns: 1024",
                 "rings: 32", "rate_hz: 10", "scan_mode: snapshot", "point_layout: xyzirt_v1"):
        check("documented default " + item, item in defaults)
    bridge = (ROOT / "litesim3d_ros2/src/sim_node.cpp").read_text()
    for name in ("cmd_mode", "sensor_reliable", "publish_odom_tf", "manifest_path", "frame_prefix"):
        check("documented ROS parameter " + name, f'"{name}"' in bridge)
    launch = (ROOT / "litesim3d_ros2/launch/demo.launch.py").read_text()
    for name in ("config", "namespace", "frame_prefix", "paused", "rviz", "manifest"):
        check("documented launch argument " + name, f'DeclareLaunchArgument("{name}"' in launch)
    # Require actual evidence for commands and claims rather than only syntax.
    for relative in ("core-final.xml", "ros.xml", "launch.xml", "install-launch.xml", "install-mesh.xml",
                     "gui-independence/result.xml", "coverage.xml"):
        path = OUT / relative
        tree = ET.parse(path).getroot()
        cases = list(tree.iter("testcase"))
        check("real passing evidence " + relative, bool(cases) and all(
            c.find("failure") is None and c.find("error") is None and c.find("skipped") is None for c in cases))
    install = json.loads((OUT / "install-check.json").read_text())
    check("installation source copy was hidden", install["status"] == "PASS" and install["source_hidden"])
    fingerprints = ""
    for relative in ("include/litesim3d/sim.hpp", "src/geometry.cpp", "src/model.cpp", "src/config.cpp", "src/simulator.cpp"):
        content = (ROOT / "litesim3d_core" / relative).read_bytes()
        fingerprints += relative + ":" + hashlib.sha256(content).hexdigest() + "\n"
    manifest = json.loads((OUT / "install-manifest.json").read_text())
    check("manifest core fingerprint matches delivered runtime source",
          manifest["core_source_sha256"] == hashlib.sha256(fingerprints.encode()).hexdigest())
    # The 2026-10-18 user decision makes native macOS/Humble the M1 gate.
    environment = json.loads((OUT / "environment.json").read_text())
    delivery = next(c for c in index["cases"] if c["id"] == "T-DEL-01")
    check("M1 accepted platform", index["platform"] == "macOS / ROS 2 Humble")
    check("explicit platform decision recorded", index["platform_decision"]["date"] == "2026-10-18"
          and index["platform_decision"]["linux_required"] is False
          and (resolve_reference(index["platform_decision"]["source"])).exists())
    check("native Humble execution environment", install["platform"] == "Darwin"
          and environment["ROS_DISTRO"] == "humble")
    check("native fresh-copy build and installed execution passed", install["status"] == "PASS"
          and all(install.get(name) == 0 for name in ("build_exit", "launch_exit", "headless_exit", "mesh_exit")))
    check("native isolated build log available", (OUT / "install-isolated-build.log").exists())
    check("T-DEL-01 uses native evidence", delivery["status"] == "PASS" and
          all(run["evidence"] == "evidence/final/install-check.json" for run in delivery["runs"]))
    result = {"status": "FAIL" if failures else "PASS", "checked_documents": len(docs),
              "local_links": links, "checks": checks, "failures": failures,
              "limits": "Static documentation/evidence audit of existing macOS/Humble M1 results; Linux is supplemental, not an M1 gate. No new runtime, GUI pixel, hardware or M2 verification."}
    (OUT / "docs-check.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    suite = ET.Element("testsuite", name="Documentation", tests="1", failures=str(int(bool(failures))))
    case = ET.SubElement(suite, "testcase", classname="T-DEL-06", name="references_defaults_traceability")
    if failures:
        ET.SubElement(case, "failure", message="; ".join(failures))
    ET.ElementTree(suite).write(OUT / "docs.xml", encoding="utf-8", xml_declaration=True)
    print(json.dumps({"status": result["status"], "checks": len(checks), "failures": failures}, ensure_ascii=False))
    raise SystemExit(bool(failures))


if __name__ == "__main__":
    main()
