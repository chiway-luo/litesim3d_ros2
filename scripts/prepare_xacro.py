#!/usr/bin/env python3
"""Expand Xacro transactionally and record source/resource provenance."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def expand(source, output, mappings=(), resource_base=None):
    source, output = source.resolve(), output.resolve()
    base = (resource_base or source.parent).resolve()
    result = subprocess.run(["xacro", str(source), *mappings], capture_output=True, text=True, check=True, timeout=30)
    included = subprocess.run(["xacro", "--deps", str(source), *mappings], capture_output=True, text=True, check=True, timeout=30)
    root = ET.fromstring(result.stdout)
    resources = {}
    for mesh in root.findall(".//mesh"):
        name = mesh.attrib["filename"]
        if "://" not in name:
            path = (base / name).resolve()
            resources[str(path)] = digest(path)
            mesh.set("filename", str(path))
    expanded = ET.tostring(root, encoding="unicode") + "\n"
    provenance = {"source": str(source), "source_sha256": digest(source), "resource_base": str(base),
                  "mappings": list(mappings), "includes": {p: digest(Path(p)) for p in included.stdout.split()},
                  "resources": resources, "expanded_sha256": hashlib.sha256(expanded.encode()).hexdigest()}
    output.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=output.name+".", dir=output.parent)
    try:
        with os.fdopen(fd, "w") as stream:
            stream.write(expanded)
        Path(temporary).replace(output)
        output.with_suffix(output.suffix+".json").write_text(json.dumps(provenance, indent=2)+"\n")
    finally:
        Path(temporary).unlink(missing_ok=True)
    return provenance


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("mappings", nargs="*")
    parser.add_argument("--resource-base", type=Path)
    args = parser.parse_args()
    expand(args.source, args.output, args.mappings, args.resource_base)
