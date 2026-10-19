"""Installed M3 shared-world launch and clean required-process shutdown."""
import os
from pathlib import Path
import time
import unittest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def generate_test_description():
    os.environ.setdefault('ROS_DOMAIN_ID','4')
    os.environ['ROS_LOCALHOST_ONLY']='1'
    path=Path(get_package_share_directory('litesim3d_ros2'))/'launch/world.launch.py'
    demo=IncludeLaunchDescription(PythonLaunchDescriptionSource(str(path)))
    return LaunchDescription([demo,launch_testing.actions.ReadyToTest()])


class TestWorldLaunch(unittest.TestCase):
    def test_four_sensors_and_unique_clock(self):
        context=Context();rclpy.init(context=context)
        node=Node('world_launch_probe',context=context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        messages=[[] for _ in range(4)]
        subscriptions=[node.create_subscription(PointCloud2,f'/r{i}/points_raw',messages[i].append,
                                               qos_profile_sensor_data) for i in range(4)]
        end=time.monotonic()+15
        try:
            while time.monotonic()<end and not all(len(m)>=3 for m in messages):
                executor.spin_once(timeout_sec=.05)
            self.assertEqual(node.count_publishers('/clock'),1)
            for i,clouds in enumerate(messages):
                self.assertGreaterEqual(len(clouds),3)
                self.assertEqual(clouds[-1].width*clouds[-1].height,16*512)
                self.assertEqual(clouds[-1].header.frame_id,f'r{i}/lidar_link')
                self.assertEqual(len(clouds[-1].data),16*512*24)
        finally:
            executor.shutdown();node.destroy_node();context.shutdown()


@launch_testing.post_shutdown_test()
class TestWorldShutdown(unittest.TestCase):
    def test_clean_exit(self,proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
