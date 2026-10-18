#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
// Original Ouster-compatible field layout, frozen to upstream commit 19071eb1.
// Intensity/reflectivity are synthetic; ambient is unknown (zero), not a sensor
// physics model.
sensor_msgs::msg::PointCloud2
ousterCloud(const sensor_msgs::msg::PointCloud2 &source);
