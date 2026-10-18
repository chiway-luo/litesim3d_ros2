#!/usr/bin/env python3
"""Test consumer with a per-process cache. No ground-truth subscription or output algorithm claim."""
import argparse
import json
import os
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, Imu
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.executors import ExternalShutdownException

from rclpy.signals import SignalHandlerOptions
parser=argparse.ArgumentParser()
parser.add_argument('--subscription-delay',type=float,default=0)
parser.add_argument('--executor-delay',type=float,default=0)
parser.add_argument('--skip-points-on-reset',action='store_true')
parser.add_argument('--reliable-points',action='store_true')
args=parser.parse_args()
rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
stopping=False
def stop(*args):
    global stopping
    stopping=True
signal.signal(signal.SIGINT,stop)
signal.signal(signal.SIGTERM,stop)
node=Node('m2_consumer',parameter_overrides=[Parameter('use_sim_time',value=True)])
state={'pid':os.getpid(),'run_id':os.environ['LWS_RUN_ID'],'points':0,'imu':0,'odom':0,'first_stamp':None}
output=Path(os.environ['LWS_RUN_DIR'])/'consumer.json'
def save():
    target=output.with_suffix('.tmp');target.write_text(json.dumps(state));target.replace(output)
state['subscriptions_created']=False
state['node_created_ns']=time.time_ns()
save()
# Expose the real Node-before-subscriptions startup window deterministically in regressions.
time.sleep(args.subscription_delay)
buffer=Buffer();listener=TransformListener(buffer,node)
def callback(key,msg):
    state[key]+=1
    if key=='points' and state['first_stamp'] is None:
        state['first_stamp']=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        state['first_bytes']=bytes(msg.data[:24]).hex()
    save()
subs=[node.create_subscription(Imu,'/imu/data_raw',lambda m:callback('imu',m),qos_profile_sensor_data),
      node.create_subscription(Odometry,'/odom',lambda m:callback('odom',m),10)]
if not (args.skip_points_on_reset and output.parent.name!='epoch-0001'):
    subs.append(node.create_subscription(PointCloud2,'/points_raw',lambda m:callback('points',m),
                                         5 if args.reliable_points else qos_profile_sensor_data))
state['subscriptions_created']=True
state['subscriptions_created_ns']=time.time_ns()
save()
time.sleep(args.executor_delay)
state['executor_started_ns']=time.time_ns()
save()
try:
    while not stopping:
        rclpy.spin_once(node,timeout_sec=.05)
except (KeyboardInterrupt,ExternalShutdownException):
    pass
finally:
    state['clean_exit']=True;save();listener.unregister();node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
