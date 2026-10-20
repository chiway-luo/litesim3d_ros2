"""Expand a model once; the simulator, renderer and RSP consume that same file."""
import argparse
import copy
import json
import os
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_prefix, get_package_share_directory
import xacro
import yaml


class UniqueLoader(yaml.SafeLoader):
    """Do not hide malformed duplicate keys while preparing a temporary file."""


def unique_mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ValueError(f"duplicate YAML key: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, unique_mapping)


def load_yaml(text):
    return yaml.load(text, Loader=UniqueLoader)


def resolve(uri, base):
    if uri.startswith("package://"):
        package, relative = uri[10:].split("/", 1)
        path = Path(relative)
        if path.is_absolute() or ".." in path.parts:
            raise ValueError(f"invalid package resource: {uri}")
        result = Path(get_package_share_directory(package)) / path
    else:
        result = Path(base) / uri.removeprefix("file://")
    result = result.expanduser().resolve()
    if not result.is_file():
        raise ValueError(f"resource not found: {uri}")
    return result


def boolean(value, name):
    text = str(value).strip().lower()
    if text not in {"true", "false"}:
        raise ValueError(f"{name} must be true or false")
    return text == "true"


def profile(selection, directory):
    if selection in {"vlp16", "mid360", "mid40"}:
        if not directory:
            raise ValueError("named lidar_profile requires lidar_profiles_dir")
        source = Path(directory) / (selection + ".yaml")
    else:
        source = Path(selection).expanduser()
    if not source.is_file():
        raise ValueError("lidar_profile must be config, vlp16, mid360, mid40, or an existing YAML path")
    data = load_yaml(source.read_text())
    if not isinstance(data, dict) or set(data) != {"lidar"} or not isinstance(data["lidar"], dict):
        raise ValueError("lidar profile must contain a single lidar mapping")
    settings = data["lidar"]
    if {"name", "link", "topic"} & settings.keys():
        raise ValueError("lidar profile cannot replace name, link or topic bindings")
    if settings.get("pattern_file"):
        settings["pattern_file"] = str(resolve(settings["pattern_file"], source.parent))
    return settings


def merge(base, overlay):
    result = copy.deepcopy(base)
    for key, value in overlay.items():
        result[key] = merge(result[key], value) if isinstance(result.get(key), dict) and isinstance(value, dict) else value
    return result


def override(sensor, values):
    elements = sensor.findall("overrides")
    if len(elements) > 1:
        raise ValueError(f"duplicate overrides: {sensor.get('name')}")
    element = elements[0] if elements else ET.SubElement(sensor, "overrides")
    old = load_yaml(element.text or "{}")
    if not isinstance(old, dict):
        raise ValueError("plugin overrides must be a mapping")
    element.text = "\n" + yaml.safe_dump(merge(old, values), sort_keys=False)


SENSOR_NOISE_FIELDS = {
    "lidar": {"range_bias_m", "range_sigma_m", "dropout"},
    "imu": {"accel_bias", "gyro_bias", "accel_sigma", "gyro_sigma"},
    "rgb": {"color_bias_dn", "color_sigma_dn"},
    "depth": {"depth_bias_m", "depth_sigma_m"},
}
SENSOR_NOISE_FIELDS["rgbd"] = SENSOR_NOISE_FIELDS["rgb"] | SENSOR_NOISE_FIELDS["depth"]
SENSOR_KINDS = {
    "litesim3d_lidar/Lidar": "lidar", "litesim3d_imu/Imu": "imu",
    "litesim3d_rgb_camera/RgbCamera": "rgb", "litesim3d_depth_camera/DepthCamera": "depth",
    "litesim3d_rgbd_camera/RgbdCamera": "rgbd",
}
ODOMETRY_NOISE_FIELDS = {
    "linear_scale_error", "angular_scale_error", "linear_bias_mps", "linear_sigma_mps",
    "angular_bias_radps", "angular_sigma_radps",
}


def apply_noise_config(data, root, filename, base):
    """Apply measurement errors after model/lidar selection, preserving bindings."""
    document = load_yaml(resolve(str(filename), base).read_text())
    if not isinstance(document, dict) or set(document) - {"odometry", "sensors"}:
        raise ValueError("noise_config accepts only odometry and sensors mappings")

    def checked(values, fields, name):
        if not isinstance(values, dict) or set(values) - fields:
            raise ValueError(f"{name}: unsupported noise fields; allowed: {', '.join(sorted(fields))}")
        return values

    if "odometry" in document:
        odometry = [] if root is None else root.findall("./litesim3d/odometry")
        supported = {
            "litesim3d_diff_odometry/DifferentialOdometry",
            "litesim3d_ackermann_odometry/AckermannOdometry",
            "litesim3d_omni_odometry/OmniOdometry",
        }
        if len(odometry) != 1 or odometry[0].get("class") not in supported:
            raise ValueError("noise_config.odometry requires one standard wheel odometry plugin; legacy YAML uses odom")
        values = checked(document["odometry"], ODOMETRY_NOISE_FIELDS, "odometry")
        override(odometry[0], {"noise": values})

    requested = document.get("sensors", {})
    if not isinstance(requested, dict):
        raise ValueError("noise_config.sensors must be a mapping keyed by sensor name")
    bindings = {}

    def add(name, kind, target, plugin):
        if name in bindings:
            raise ValueError(f"ambiguous sensor name in noise_config: {name}")
        bindings[name] = (kind, target, plugin)

    if root is not None:
        for sensor in root.findall("./litesim3d/sensor"):
            add(sensor.get("name"), SENSOR_KINDS.get(sensor.get("class")), sensor, True)
    for group, kind in (("lidars", "lidar"), ("imus", "imu"), ("cameras", None)):
        for sensor in data.get(group, []):
            add(sensor.get("name"), kind or sensor.get("type", "rgb"), sensor, False)
    for name, values in requested.items():
        if name not in bindings:
            raise ValueError(f"noise_config: unknown sensor name: {name}")
        kind, target, plugin = bindings[name]
        if kind not in SENSOR_NOISE_FIELDS:
            raise ValueError(f"noise_config: unsupported sensor type: {name}")
        values = checked(values, SENSOR_NOISE_FIELDS[kind], f"sensors.{name}")
        if plugin:
            override(target, values)
        else:
            target.update(values)


def prepare_config(source, directory, *, mappings=None, cameras="", lidar_profile="config", profiles_dir="", noise_config=""):
    """Write resolved config and combined URDF; input files are never changed.

    Relative URDF resources belong to the top-level model directory. Includes
    should use $(find package) or package:// paths for portable resources.
    """
    source, directory = Path(source).resolve(), Path(directory).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    data = load_yaml(source.read_text())
    if not isinstance(data, dict) or not isinstance(data.get("robot"), dict):
        raise ValueError("configuration must contain a robot mapping")
    mappings = {} if mappings is None else mappings
    if not isinstance(mappings, dict) or any(not isinstance(k, str) or isinstance(v, (dict, list)) for k, v in mappings.items()):
        raise ValueError("model_arguments must be a YAML mapping of scalar Xacro arguments")
    sensors = []
    root = None
    if data["robot"].get("urdf"):
        model = resolve(data["robot"]["urdf"], source.parent)
        if model.suffix == ".xacro":
            document = xacro.process_file(str(model), mappings={k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in mappings.items()})
            root = ET.fromstring(document.toxml())
        else:
            if mappings:
                raise ValueError("model_arguments requires a .xacro model")
            root = ET.parse(model).getroot()
        if root.tag != "robot":
            raise ValueError("model must contain a robot root")
        # Pin paths before writing the expanded document in a different directory.
        for element, key in [(e, "filename") for e in root.findall(".//mesh") + root.findall(".//texture")] + [
                (e, "file") for e in root.findall("./litesim3d/*/parameters")]:
            if element.get(key):
                element.set(key, str(resolve(element.get(key), model.parent)))
        sensors = root.findall("./litesim3d/sensor")
        for index, sensor in enumerate(sensors):
            if sensor.get("class") != "litesim3d_lidar/Lidar":
                continue
            for file_index, element in enumerate(sensor.findall("parameters")):
                document = load_yaml(Path(element.get("file")).read_text())
                selected = document
                if element.get("section"):
                    for key in element.get("section").split("."):
                        selected = selected[key]
                if isinstance(selected, dict) and selected.get("pattern_file"):
                    selected["pattern_file"] = str(resolve(selected["pattern_file"], model.parent))
                    pinned = directory / f"sensor-{index}-parameters-{file_index}.yaml"
                    pinned.write_text(yaml.safe_dump(document, sort_keys=False))
                    element.set("file", str(pinned))
            for element in sensor.findall("overrides"):
                selected = load_yaml(element.text or "{}")
                if isinstance(selected, dict) and selected.get("pattern_file"):
                    override(sensor, {"pattern_file": str(resolve(selected["pattern_file"], model.parent))})
    elif mappings:
        raise ValueError("model_arguments requires robot.urdf")
    if cameras != "":
        enabled = boolean(cameras, "cameras")
        camera_plugins = [s for s in sensors if s.get("class") in {
            "litesim3d_rgb_camera/RgbCamera", "litesim3d_depth_camera/DepthCamera", "litesim3d_rgbd_camera/RgbdCamera"}]
        if enabled and not camera_plugins and not data.get("cameras"):
            raise ValueError("cameras:=true requires a camera plugin or legacy camera definitions")
        for sensor in camera_plugins:
            override(sensor, {"enabled": enabled})
        for sensor in data.get("cameras", []):
            sensor["enabled"] = enabled
    if lidar_profile != "config":
        settings = profile(lidar_profile, profiles_dir)
        plugin_lidars = [s for s in sensors if s.get("class") == "litesim3d_lidar/Lidar"]
        if len(plugin_lidars) == 1 and not data.get("lidars"):
            # Plugin point layout is fixed, not a configurable parameter.
            if settings.pop("point_layout", "xyzirt_v1") != "xyzirt_v1":
                raise ValueError("only xyzirt_v1 point layout is supported")
            # Reset spinning-only fields when selecting a non-spinning profile.
            rings = settings.pop("rings", None)
            settings.setdefault("vertical_angles_deg", [0])
            if rings is not None and rings != len(settings["vertical_angles_deg"]):
                raise ValueError("profile rings must match vertical_angles_deg")
            settings.setdefault("ring_delays_s", [])
            override(plugin_lidars[0], settings)
        elif not plugin_lidars and len(data.get("lidars", [])) == 1:
            binding = data["lidars"][0]
            data["lidars"] = [{**settings, **{k: binding[k] for k in ("name", "link", "topic") if k in binding}}]
        else:
            raise ValueError("lidar_profile requires exactly one lidar and no mixed plugin/YAML sensor definitions")
    if noise_config:
        apply_noise_config(data, root, noise_config, source.parent)
    if root is not None:
        combined = directory / "robot.urdf"
        ET.indent(root)
        ET.ElementTree(root).write(combined, encoding="utf-8", xml_declaration=True)
        data["robot"]["urdf"] = str(combined)
    for sensor in data.get("lidars", []):
        if sensor.get("pattern_file"):
            sensor["pattern_file"] = str(resolve(sensor["pattern_file"], source.parent))
    for entity in data.get("entities", []):
        for role in ("geometry", "visual", "collision", "sensor_geometry"):
            geometry = entity.get(role, {})
            if isinstance(geometry, dict) and geometry.get("type") == "mesh":
                geometry["uri"] = str(resolve(geometry["uri"], source.parent))
    destination = directory / "config.yaml"
    destination.write_text(yaml.safe_dump(data, sort_keys=False))
    return destination



def lidar_preview_topics(config, interface, xfer_format):
    """RViz topic substitutions for resolved Mid-360 bindings only."""
    if interface == "native":
        return {}
    config = Path(config)
    data = load_yaml(config.read_text())
    lidars = list(data.get("lidars", []))
    model = resolve(data["robot"]["urdf"], config.parent)
    for sensor in ET.parse(model).getroot().findall("./litesim3d/sensor"):
        if sensor.get("class") != "litesim3d_lidar/Lidar":
            continue
        values = {}
        for parameters in sensor.findall("parameters"):
            settings = load_yaml(resolve(parameters.get("file"), model.parent).read_text())
            for key in filter(None, parameters.get("section", "").split(".")):
                settings = settings[key]
            values = merge(values, settings)
        for element in sensor.findall("overrides"):
            values = merge(values, load_yaml(element.text or "{}"))
        lidars.append(values)
    result = {}
    for lidar in lidars:
        if lidar.get("scan_pattern") == "livox_mid360":
            topic = "/" + lidar["topic"].lstrip("/")
            result[topic] = "/livox/lidar/pointcloud" if xfer_format == 1 else "/livox/lidar"
            result[topic + "/range_boundary"] = "/livox/lidar/range_boundary"
    return result



def main():
    parser = argparse.ArgumentParser(description="Expand one model for LiteSim3D core, RSP and rendering")
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path, help="new or empty output directory")
    parser.add_argument("--model-arguments", default="{}", help="YAML mapping of Xacro arguments")
    parser.add_argument("--cameras", choices=("true", "false"), default="")
    parser.add_argument("--lidar-profile", default="config")
    parser.add_argument("--lidar-profiles-dir", default="")
    parser.add_argument("--noise-config", default="", help="measurement errors YAML; sensor names must match the model")
    parser.add_argument("--world", type=Path, help="optional SDF world; config is then SDF runtime bindings")
    parser.add_argument("--assets-root", type=Path)
    parser.add_argument("--collision-budget-enabled", choices=("true", "false"), default="false")
    parser.add_argument("--real-time-factor", default="")
    args = parser.parse_args()
    output = args.output.resolve()
    if output.exists() and any(output.iterdir()):
        parser.error("output directory must be empty")
    try:
        prepared = prepare_config(args.config, output / "model", mappings=yaml.safe_load(args.model_arguments),
                                  cameras=args.cameras, lidar_profile=args.lidar_profile, profiles_dir=args.lidar_profiles_dir,
                                  noise_config=args.noise_config)
        result = {"config": str(prepared)}
        if args.world:
            env = os.environ.copy()
            if args.assets_root:
                env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([str(args.assets_root.resolve() / p) for p in ("models", "worlds")] + [env.get("GZ_SIM_RESOURCE_PATH", "")])
            importer = Path(get_package_prefix("litesim3d_assets")) / "lib/litesim3d_assets/sdf_import"
            command = [str(importer), str(args.world.resolve()), str(prepared), str(output / "scene"), args.collision_budget_enabled]
            if args.real_time_factor:
                command.append(args.real_time_factor)
            subprocess.run(command, env=env, check=True, timeout=120)
            result = {"config": str(output / "scene/runtime.yaml"), "scene_description": str(output / "scene/scene.json")}
        print(json.dumps(result))
    except Exception as error:
        parser.exit(1, f"model preparation: {error}\n")
