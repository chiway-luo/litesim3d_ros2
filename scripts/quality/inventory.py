"""Discover current runtime sources; newly added packages join the denominator."""
from pathlib import Path
import xml.etree.ElementTree as ET


# These files contain declarations and data initializers, but no function bodies.
# Keep this explicit: a new header cannot silently disappear from coverage.
DECLARATIONS = {
    "litesim3d_bringup/src/model.hpp",
    "litesim3d_plugin_api/include/litesim3d_plugin_api/declarations.hpp",
    "litesim3d_plugin_api/include/litesim3d_plugin_api/motion.hpp",
    "litesim3d_plugin_api/include/litesim3d_plugin_api/model_geometry.hpp",
    "litesim3d_core/include/litesim3d/sensor_plugin.hpp",
    "litesim3d_ros2/src/point_adapter.hpp",
    "litesim3d_ros2/include/litesim3d_ros2/lidar_output_plugin.hpp",
}
EXCLUDED_PACKAGES = {
    "litesim3d_interfaces": "Generated ROS message/service bindings",
    "livox_ros_driver2": "Upstream message definitions and generated bindings",
    "litesim3d_test_support": "Test harness, not simulator runtime",
    "litesim3d_plugin_tests": "Contract tests and test probes",
}
EXCLUDED_FILES = {
    "litesim3d_core/src/load_benchmark.cpp": "Standalone load measurement tool",
    "litesim3d_gui/resources/export_icons.py": "Development-only icon generator; not installed or executed by the simulator",
}


def inventory(root):
    root = Path(root).resolve()
    sources, excluded, packages = {}, {}, {}
    for manifest in sorted(root.rglob("package.xml")):
        package = manifest.parent
        if any((parent / "COLCON_IGNORE").exists()
               for parent in (package, *package.parents) if parent.is_relative_to(root)):
            excluded[package.relative_to(root).as_posix() + "/"] = "Separately built external example"
            continue
        name = ET.parse(manifest).findtext("name")
        if name in EXCLUDED_PACKAGES:
            excluded[package.relative_to(root).as_posix() + "/"] = EXCLUDED_PACKAGES[name]
            continue
        packages[name] = package.relative_to(root).as_posix()
        for path in sorted(package.rglob("*")):
            if not path.is_file() or any(part in {"test", "tests", "__pycache__", ".pytest_cache"}
                                         for part in path.relative_to(package).parts):
                continue
            relative = path.relative_to(root).as_posix()
            if relative in EXCLUDED_FILES:
                excluded[relative] = EXCLUDED_FILES[relative]
                continue
            language = "cpp" if path.suffix in {".cpp", ".cc", ".cxx", ".hpp", ".h"} else None
            if path.suffix == ".py" or (path.parent.name == "scripts" and
                    ("python" in path.read_text(errors="replace").split("\n", 1)[0]
                     or path.name == "sim_session")):
                language = "python"
            if language:
                sources[relative] = {"language": language, "package": name,
                                     "declarations_only": relative in DECLARATIONS}
    # This compatibility entry point is executed by users outside ROS packages.
    if (root / "scripts/prepare_xacro.py").is_file():
        sources["scripts/prepare_xacro.py"] = {"language": "python", "package": "tools",
                                               "declarations_only": False}
    return {"sources": sources, "packages": packages, "exclusions": excluded,
            "policy": "Own installed runtime and example launch/configuration code; headers with executable mappings included. Tests, generated bindings, resources and development/report tooling excluded."}


def python_aliases(root, prefix, item):
    """Map installed Python back to the same source without copying counters."""
    root, prefix = Path(root), Path(prefix)
    aliases = {}
    for relative, metadata in item["sources"].items():
        if metadata["language"] != "python":
            continue
        path = Path(relative)
        package = metadata["package"]
        source = root / path
        candidates = [source]
        if "launch" in path.parts:
            candidates.append(prefix / "share" / package / "launch" / path.name)
        if path.parent.name == "scripts":
            candidates.append(prefix / "lib" / package / path.name)
        if package in path.parts and path.parent.name == package:
            for library in ("lib", "local/lib"):
                for layout in ("site-packages", "dist-packages"):
                    candidates.extend(prefix.glob(f"{library}/python*/{layout}/{package}/{path.name}"))
        aliases[relative] = [str(p) for p in candidates]
    return aliases


def coverage_config(root, prefix, item):
    lines = ["[run]", "branch = True", "parallel = True", "data_file = ${COVERAGE_FILE}",
             "source =", f"    {root}", f"    {prefix}", "", "[paths]"]
    # coverage.py aliases are directory prefixes, not exact file names.
    directories = dict.fromkeys(tuple(dict.fromkeys(str(Path(path).parent) for path in paths))
                                for paths in python_aliases(root, prefix, item).values())
    for index, paths in enumerate(directories):
        lines += [f"source_{index} =", *[f"    {path}" for path in paths]]
    return "\n".join(lines) + "\n"
