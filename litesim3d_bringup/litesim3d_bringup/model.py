"""Launch adapters for native model preparation; all validation lives in C++."""
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET
import yaml

from ament_index_python.packages import get_package_prefix


def _executable():
    return os.path.join(get_package_prefix("litesim3d_bringup"), "lib", "litesim3d_bringup", "prepare_config")


def _request(operation, **values):
    result = subprocess.run([_executable(), "--request"],
                            input=json.dumps({"operation": operation, **values}),
                            text=True, capture_output=True, timeout=180)
    if result.returncode:
        raise ValueError(result.stderr.strip() or "native model preparation failed")
    return json.loads(result.stdout)


def load_yaml(text):
    return _request("load_yaml", text=text)


def resolve(uri, base):
    return Path(_request("resolve", uri=str(uri), base=str(base)))


def ros_robot_description(urdf):
    return _request("ros_description", urdf=str(urdf))


def boolean(value, name):
    # Launch input conversion remains in the Python launch adapter.
    value = str(value).strip().lower()
    if value not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return value == "true"


def profile(selection, directory):
    return _request("profile", selection=str(selection), directory=str(directory))


def merge(base, overlay):
    return _request("merge", base=base, overlay=overlay)


def _replace_element(target, text):
    replacement = ET.fromstring(text)
    target.attrib.clear()
    target.attrib.update(replacement.attrib)
    target.text = replacement.text
    target[:] = list(replacement)


def override(sensor, values):
    _replace_element(sensor, _request("override", xml=ET.tostring(sensor, encoding="unicode"), values=values))


def apply_noise_config(data, root, filename, base):
    result = _request("noise", data=data, xml=None if root is None else ET.tostring(root, encoding="unicode"),
                      filename=str(filename), base=str(base))
    data.clear()
    data.update(result["data"])
    if root is not None:
        _replace_element(root, result["xml"])


def prepare_config(source, directory, *, robot_urdf="", mappings=None, cameras="", lidar_profile="config", profiles_dir="", noise_config=""):
    return Path(_request("prepare", source=str(source), directory=str(directory), robot_urdf=str(robot_urdf),
                         mappings_yaml=yaml.safe_dump(mappings), cameras=cameras, lidar_profile=str(lidar_profile),
                         profiles_dir=str(profiles_dir), noise_config=str(noise_config)))


def lidar_preview_topics(config, interface, xfer_format):
    return _request("preview_topics", config=str(config), interface=interface, format=xfer_format)


def main():
    executable = _executable()
    os.execv(executable, [executable, *sys.argv[1:]])
