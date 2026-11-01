#!/usr/bin/env python3
"""Compatibility entry point for the native ros_client tool."""
import os
import sys
from ament_index_python.packages import get_package_prefix


def main():
    executable = os.path.join(get_package_prefix("litesim3d_ros2"), "lib", "litesim3d_ros2", "ros_client")
    os.execv(executable, [executable, *sys.argv[1:]])


if __name__ == "__main__":
    main()
