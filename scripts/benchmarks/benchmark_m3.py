#!/usr/bin/env python3
"""B3: four real ROS robots; reuse the unchanged M2 measurement boundaries/gates."""
import hashlib
import json
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import time

import psutil
import yaml
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from diagnostic_msgs.msg import DiagnosticArray
from ament_index_python.packages import get_package_share_directory
import benchmarks.benchmark_m2 as measurement


def resolve(uri, base):
    if uri.startswith('package://'):
        package, relative = uri[10:].split('/', 1)
        return Path(get_package_share_directory(package))/relative
    return (base/uri.removeprefix('file://')).resolve()


class WorldRun(measurement.Run):
    def __init__(self, source, directory, rtf, gui=False):
        if gui:
            raise ValueError('B3 baseline is headless; use --gui off')
        self.directory=directory;directory.mkdir(parents=True,exist_ok=False)
        self.processes=[];self.status={};self.hashes={};self.clouds=0
        self.last_stamps={};self.delivery_gaps=0
        self.context=Context();rclpy.init(context=self.context)
        self.node=Node('m3_benchmark',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.sub=self.node.create_subscription(DiagnosticArray,'/sim/status',self.on_status,10)
        world=yaml.safe_load(source.read_text())
        common=resolve(world['config'],source.parent)
        data=yaml.safe_load(common.read_text());data['simulation']['real_time_factor']=rtf
        data['robot']['urdf']=str(resolve(data['robot']['urdf'],common.parent))
        for entity in data['entities']:
            for role in ('geometry','visual','collision','sensor_geometry'):
                shape=entity.get(role,{})
                if shape.get('type')=='mesh':shape['uri']=str(resolve(shape['uri'],common.parent))
        self.config=directory/'robot.yaml';self.config.write_text(yaml.safe_dump(data))
        world['config']=str(self.config)
        path=directory/'world.yaml';path.write_text(yaml.safe_dump(world))
        self.cloudsubs=[]
        for robot in world['robots']:
            name=robot['id']
            self.cloudsubs.append(self.node.create_subscription(PointCloud2,'/'+name+'/points_raw',
                lambda m, name=name:self.on_robot_cloud(name,m),qos_profile_sensor_data))
            rsp=directory/(name+'-rsp.yaml')
            rsp.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
                'robot_description':Path(data['robot']['urdf']).read_text(),'use_sim_time':True,
                'frame_prefix':name+'/','publish_frequency':200.0}}}))
            self.spawn(name+'-rsp',[measurement.exe('robot_state_publisher','robot_state_publisher'),
                '--ros-args','--params-file',str(rsp),'-r','__ns:=/'+name])
        sim=directory/'sim.yaml';sim.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
            'config':str(path),'use_sim_time':True,'manifest_path':str(directory/'manifest.json')}}}))
        self.start=time.monotonic()
        self.spawn('sim',[measurement.exe('litesim3d_ros2','sim_world_node'),'--ros-args','--params-file',str(sim)])
        self.until(lambda:self.status.get('ready')=='true',60)
        self.ready_seconds=time.monotonic()-self.start
        self.resources={name:psutil.Process(p.pid) for name,p in self.processes}
        for process in self.resources.values():process.cpu_percent()

    def on_status(self,message):
        if not message.status:return
        robots=[{v.key:v.value for v in s.values} for s in message.status]
        if len(robots)!=4 or len({r['time_ns'] for r in robots})!=1:
            raise RuntimeError('B3 requires four synchronized robots')
        expected=(int(robots[0]['time_ns'])-1)//100_000_000
        if any(int(r['published_clouds'])!=expected for r in robots):
            raise RuntimeError('published count differs from the 10 Hz schedule')
        self.status=dict(robots[0])
        for key in ('published_clouds','internal_missed_scans'):
            self.status[key]=str(sum(int(r[key]) for r in robots))
        self.status['per_robot_clouds']=[int(r['published_clouds']) for r in robots]
        if len(set(self.status['per_robot_clouds']))!=1:
            raise RuntimeError('one robot missed a scheduled scan')
        if self.node.count_publishers('/clock')!=1:
            raise RuntimeError('world clock ownership violation')

    def on_robot_cloud(self,name,message):
        self.clouds+=1;stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        previous=self.last_stamps.get(name)
        if previous is not None and stamp-previous>100_000_001:self.delivery_gaps+=1
        self.last_stamps[name]=stamp
        if len(self.hashes)<100:self.hashes[name+'/'+str(stamp)]=hashlib.sha256(bytes(message.data)).hexdigest()


if __name__=='__main__':
    # Identical warm-up, sampling, process RSS, timing and exit checks; only process setup differs.
    measurement.Run=WorldRun
    measurement.MILESTONE="M3"
    raise SystemExit(measurement.main())
