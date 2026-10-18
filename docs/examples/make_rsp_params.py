#!/usr/bin/env python3
"""从仿真 YAML 生成独立 robot_state_publisher 的参数文件，不修改原配置。"""
import argparse
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--frame-prefix", default="")
    args = parser.parse_args()
    if args.frame_prefix and (args.frame_prefix.startswith("/") or not args.frame_prefix.endswith("/")):
        parser.error("frame-prefix 必须是以 / 结尾的相对前缀，例如 r0/")
    source = args.config.resolve()
    config = yaml.safe_load(source.read_text())
    uri = config["robot"]["urdf"]
    if uri.startswith("package://"):
        package, relative = uri[10:].split("/", 1)
        model = Path(get_package_share_directory(package)) / relative
    else:
        model = source.parent / uri.removeprefix("file://")
    params = {"/**": {"ros__parameters": {
        "robot_description": model.resolve().read_text(),
        "use_sim_time": True,
        "frame_prefix": args.frame_prefix,
        "publish_frequency": 200.0,
    }}}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(params, allow_unicode=True, sort_keys=False))
    print(args.output.resolve())


if __name__ == "__main__":
    main()
