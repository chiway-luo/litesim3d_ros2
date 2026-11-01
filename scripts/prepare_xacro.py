#!/usr/bin/env python3
"""Compatibility adapter for native Xacro resource preparation and provenance."""
import json
import os
import subprocess
import sys
from ament_index_python.packages import get_package_prefix


def _executable():
    return os.path.join(get_package_prefix("litesim3d_bringup"), "lib", "litesim3d_bringup", "prepare_xacro")


def expand(source, output, mappings=(), resource_base=None):
    command = [_executable(), str(source), str(output), *mappings]
    if resource_base is not None:
        command += ["--resource-base", str(resource_base)]
    result = subprocess.run(command, text=True, capture_output=True, check=True, timeout=90)
    return json.loads(result.stdout)


if __name__ == "__main__":
    executable = _executable()
    os.execv(executable, [executable, *sys.argv[1:]])
