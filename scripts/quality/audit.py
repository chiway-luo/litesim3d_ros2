"""Shared checks for instrumentation and source attribution."""
import json
from pathlib import Path
import re


def compilation_errors(root, build, item, flag):
    root, build = Path(root).resolve(), Path(build)
    commands = {}
    for database in build.glob("*/compile_commands.json"):
        for entry in json.loads(database.read_text()):
            path = Path(entry["file"])
            if not path.is_absolute():
                path = Path(entry["directory"]) / path
            commands.setdefault(path.resolve(), []).append(entry.get("command", " ".join(entry.get("arguments", []))))
    failures = []
    for name, source in item["sources"].items():
        if source["language"] == "cpp" and Path(name).suffix in {".cpp", ".cc", ".cxx"}:
            entries = commands.get(root / name, [])
            if not entries or any(flag not in command for command in entries):
                failures.append(f"Missing {flag} instrumentation: {name}")
    return failures


def canonical_source(filename, root, prefix, item):
    path = Path(filename).resolve()
    root, prefix = Path(root).resolve(), Path(prefix).resolve()
    if path.is_relative_to(root):
        relative = path.relative_to(root).as_posix()
        return relative if relative in item["sources"] else None
    if path.is_relative_to(prefix / "include"):
        suffix = "/include/" + path.relative_to(prefix / "include").as_posix()
        matches = [name for name in item["sources"] if name.endswith(suffix)]
        if len(matches) == 1:
            return matches[0]
    return None


def has_function_body(text):
    text = re.sub(r"/\*.*?\*/|//[^\n]*", "", text, flags=re.S)
    return bool(re.search(r"\)\s*(?:(?:const|noexcept|override|final)\s*)*(?:->[^{;]+)?\{", text))
