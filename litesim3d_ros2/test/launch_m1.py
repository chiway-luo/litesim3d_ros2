"""launch_testing verifies the installed launch lifecycle and real message flow."""
import os
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock


def generate_test_description():
    os.environ.setdefault("ROS_DOMAIN_ID", "5")
    os.environ["ROS_LOCALHOST_ONLY"]="1"
    path=os.path.join(get_package_share_directory("litesim3d_ros2"),"launch","demo.launch.py")
    demo=IncludeLaunchDescription(PythonLaunchDescriptionSource(path),launch_arguments={"rviz":"false"}.items())
    return LaunchDescription([demo,launch_testing.actions.ReadyToTest()])


class TestInstalledLaunch(unittest.TestCase):
    def test_real_clock_and_cloud(self):
        context=Context();rclpy.init(context=context)
        node=Node("launch_probe",context=context)
        executor=SingleThreadedExecutor(context=context);executor.add_node(node)
        clouds=[];clocks=[]
        node.create_subscription(PointCloud2,"/points_raw",clouds.append,qos_profile_sensor_data)
        node.create_subscription(Clock,"/clock",clocks.append,qos_profile_sensor_data)
        import time
        end=time.monotonic()+8
        try:
            while time.monotonic()<end and (len(clouds)<3 or len(clocks)<10):executor.spin_once(timeout_sec=.05)
            self.assertGreaterEqual(len(clouds),3)
            self.assertEqual(clouds[-1].point_step,24)
            self.assertEqual(clouds[-1].width*clouds[-1].height,32768)
            self.assertEqual(len(clouds[-1].data),32768*24)
            self.assertEqual(clouds[-1].header.frame_id,"lidar_link")
            self.assertGreaterEqual(len(clocks),10)
        finally:
            executor.shutdown();node.destroy_node();context.shutdown()


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_clean_exit(self,proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
