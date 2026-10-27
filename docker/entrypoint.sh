#!/usr/bin/env bash
set -eo pipefail
# ROS setup scripts can read unset variables, so do not enable nounset here.
source /opt/ros/humble/setup.bash
source /opt/litesim3d/install/setup.bash
exec "$@"
