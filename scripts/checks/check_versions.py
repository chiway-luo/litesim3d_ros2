#!/usr/bin/env python3
"""Ensure ROS package metadata matches the repository VERSION file."""
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


def main():
    version = (ROOT / 'VERSION').read_text().strip()
    failures = []
    if not re.fullmatch(r'(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)\.(0|[1-9][0-9]*)', version):
        failures.append('VERSION must contain a major.minor.patch version without leading zeros')
    manifests = sorted(ROOT.glob('*/package.xml'))
    manifests.extend(sorted((ROOT / 'examples').rglob('package.xml')))
    packages = {}
    if not manifests:
        failures.append('No ROS package manifests found')
    for path in manifests:
        relative = path.relative_to(ROOT).as_posix()
        package = ET.parse(path).getroot()
        package_version = package.findtext('version', '').strip()
        packages[package.findtext('name')] = package_version
        if package_version != version:
            failures.append(f'{relative}: {package_version!r} does not match VERSION {version!r}')
    print(json.dumps({'status': 'FAIL' if failures else 'PASS', 'version': version,
                      'packages': packages, 'failures': failures}, indent=2))
    return int(bool(failures))


if __name__ == '__main__':
    raise SystemExit(main())
