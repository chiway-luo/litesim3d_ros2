#!/usr/bin/env python3
"""Wall-clock ROS benchmark/soak with bounded monitor memory and per-process resource accounting."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import platform
import signal
import subprocess
import time
import xml.etree.ElementTree as ET
import psutil
import yaml
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from ament_index_python.packages import get_package_prefix,get_package_share_directory


MILESTONE = "M2"

def exe(package,name):return str(Path(get_package_prefix(package))/'lib'/package/name)
def percentile(values,p):return sorted(values)[min(len(values)-1,max(0,math.ceil(p*len(values))-1))] if values else 0

def environment():
    def command(argv):
        try:return subprocess.check_output(argv,text=True,stderr=subprocess.STDOUT,timeout=5).strip()
        except Exception as error:return str(error)
    return {'platform':platform.platform(),'machine':platform.machine(),'physical_cores':psutil.cpu_count(logical=False),
            'logical_cores':psutil.cpu_count(),'memory_bytes':psutil.virtual_memory().total,
            'cpu':command(['sysctl','-n','machdep.cpu.brand_string']), 'power':command(['pmset','-g','custom']),
            'power_source':command(['pmset','-g','batt']), 'load_average':os.getloadavg(),
            'background_cpu_percent':psutil.cpu_percent(interval=.2),'compiler':command(['/usr/bin/clang++','--version']),
            'ros':'Humble / RoboStack','build':'RelWithDebInfo','ray_threads':4,
            'cpu_percent_convention':'one logical core = 100%; process values may exceed 100%'}


class Run:
    def __init__(self,source,directory,rtf,gui=False):
        self.directory=directory;directory.mkdir(parents=True,exist_ok=False)
        self.processes=[];self.status={};self.hashes={};self.clouds=0
        self.last_stamp=None;self.delivery_gaps=0
        self.context=Context();rclpy.init(context=self.context)
        self.node=Node('m2_benchmark',context=self.context)
        self.executor=SingleThreadedExecutor(context=self.context);self.executor.add_node(self.node)
        self.sub=self.node.create_subscription(DiagnosticArray,'/sim/status',self.on_status,10)
        self.cloudsub=self.node.create_subscription(PointCloud2,'/points_raw',self.on_cloud,qos_profile_sensor_data)
        data=yaml.safe_load(source.read_text());data['simulation']['real_time_factor']=rtf
        def resource(uri):
            if uri.startswith('package://'):
                package,relative=uri[10:].split('/',1)
                return str(Path(get_package_share_directory(package))/relative)
            return str((source.parent/uri.removeprefix('file://')).resolve())
        data['robot']['urdf']=resource(data['robot']['urdf'])
        for entity in data['entities']:
            for role in ('geometry','visual','collision','sensor_geometry'):
                shape=entity.get(role,{})
                if shape.get('type')=='mesh':shape['uri']=resource(shape['uri'])
        self.config=directory/'config.yaml';self.config.write_text(yaml.safe_dump(data))
        urdf=data['robot']['urdf']
        if not Path(urdf).is_absolute():urdf=str((source.parent/urdf).resolve())
        rsp=directory/'rsp.yaml';rsp.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
            'robot_description':Path(urdf).read_text(),'use_sim_time':True,'publish_frequency':200.0}}}))
        self.spawn('rsp',[exe('robot_state_publisher','robot_state_publisher'),'--ros-args','--params-file',str(rsp)])
        sim=directory/'sim.yaml';sim.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
            'config':str(self.config),'use_sim_time':True,'manifest_path':str(directory/'manifest.json')}}}))
        self.start=time.monotonic()
        self.spawn('sim',[exe('litesim3d_ros2','sim_node'),'--ros-args','--params-file',str(sim)])
        self.until(lambda:self.status.get('ready')=='true',60)
        self.ready_seconds=time.monotonic()-self.start
        if gui:
            self.spawn('rviz',[exe('rviz2','rviz2'),'-d',str(Path(get_package_share_directory('litesim3d_ros2'))/'rviz/demo.rviz'),
                               '--ros-args','-p','use_sim_time:=true'])
            self.until(lambda:'OpenGl version' in (directory/'rviz.log').read_text(),20)
        self.resources={name:psutil.Process(p.pid) for name,p in self.processes}
        for process in self.resources.values():process.cpu_percent()

    def spawn(self,name,command):
        with (self.directory/(name+'.log')).open('w') as out:
            process=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
        self.processes.append((name,process))

    def on_status(self,message):
        if message.status:self.status={v.key:v.value for v in message.status[0].values}
    def on_cloud(self,message):
        self.clouds+=1;stamp=message.header.stamp.sec*10**9+message.header.stamp.nanosec
        if self.last_stamp is not None and stamp-self.last_stamp>100_000_001:self.delivery_gaps+=1
        self.last_stamp=stamp
        # Samples for reproducibility, with a hard cap. Raw monitor memory never scales with run length.
        if len(self.hashes)<100:self.hashes[str(stamp)]=hashlib.sha256(bytes(message.data)).hexdigest()

    def spin(self,duration):
        end=time.monotonic()+duration
        while time.monotonic()<end:
            self.executor.spin_once(timeout_sec=min(.02,max(0,end-time.monotonic())))
            for name,p in self.processes:
                if p.poll() is not None:raise RuntimeError(f'{name} exited {p.returncode}')
            if self.status.get('state')=='fault':raise RuntimeError(str(self.status))
    def until(self,predicate,timeout):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>=end:raise TimeoutError('readiness/progress timeout')
            self.spin(.05)
    def pause(self,value):
        client=self.node.create_client(SetBool,'/sim/pause')
        if not client.wait_for_service(timeout_sec=3):raise TimeoutError('pause service missing')
        request=SetBool.Request();request.data=value;future=client.call_async(request)
        self.until(future.done,5)
        if not future.result().success:raise RuntimeError(future.result().message)
        self.node.destroy_client(client);self.spin(.25)
    def close(self):
        exits={}
        for name,p in reversed(self.processes):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
            exits[name]=p.returncode
        (self.directory/'exits.json').write_text(json.dumps(exits,indent=2))
        self.executor.shutdown();self.node.destroy_node();self.context.shutdown()
        return exits


def measure(source,directory,rtf,gui,warmup,duration,soak=False):
    run=None;result={'status':'RUNNING','rtf_target':rtf,'gui':gui,'warmup_seconds':warmup,'duration_seconds':duration}
    directory.parent.mkdir(parents=True,exist_ok=True)
    try:
        run=Run.__new__(Run)
        run.__init__(source,directory,rtf,gui)
        result['startup_seconds']=run.ready_seconds
        result['startup_cache']='uncontrolled OS cache; not a verified cold-start measurement'
        run.spin(warmup)
        run.pause(True);baseline=run.resources['sim'].memory_info().rss;run.pause(False)
        run.spin(.3)
        begin=time.monotonic();sim_begin=int(run.status['time_ns']);frames_begin=int(run.status['published_clouds'])
        samples={key:{'cpu':[],'rss_peak':0,'rss_first':None} for key in run.resources}
        slope_samples=[];last_progress=begin;last_sim=sim_begin
        peak_compute=peak_bridge=0
        with (directory/'samples.jsonl').open('w',buffering=1) as out:
            while time.monotonic()-begin<duration:
                run.spin(min(1,duration-(time.monotonic()-begin)))
                now=time.monotonic();elapsed=now-begin
                row={'wall_seconds':elapsed,'time_ns':int(run.status['time_ns']), 'status':dict(run.status),'processes':{}}
                if row['time_ns']>last_sim:last_progress=now;last_sim=row['time_ns']
                if now-last_progress>30:raise TimeoutError('simulation made no progress for 30 wall seconds')
                peak_compute=max(peak_compute,float(run.status.get('compute_p99_ms',0)))
                peak_bridge=max(peak_bridge,float(run.status.get('bridge_p99_ms',0)))
                for key,proc in run.resources.items():
                    rss=proc.memory_info().rss;cpu=proc.cpu_percent();entry=samples[key]
                    # Online sum + bounded histogram samples for soak; performance has exactly <=120 samples.
                    entry.setdefault('cpu_sum',0);entry.setdefault('cpu_count',0);entry.setdefault('cpu_max',0)
                    entry['cpu_sum']+=cpu;entry['cpu_count']+=1;entry['cpu_max']=max(entry['cpu_max'],cpu)
                    if len(entry['cpu'])<4096:entry['cpu'].append(cpu)
                    entry['rss_peak']=max(entry['rss_peak'],rss)
                    if entry['rss_first'] is None:entry['rss_first']=rss
                    row['processes'][key]={'rss_bytes':rss,'cpu_percent':cpu,'fds':proc.num_fds()}
                if not slope_samples or elapsed-slope_samples[-1][0]>=600 or elapsed>=duration-.1:
                    slope_samples.append((elapsed,row['processes']['sim']['rss_bytes']))
                out.write(json.dumps(row)+'\n')
                if int(elapsed)%60==0:(directory/'progress.json').write_text(json.dumps({'wall_seconds':elapsed,'target':duration,'status':'RUNNING'}))
        wall=time.monotonic()-begin;advanced=(int(run.status['time_ns'])-sim_begin)*1e-9
        result.update(wall_seconds=wall,simulated_seconds=advanced,rtf=advanced/wall,
                      generated_frames=int(run.status['published_clouds'])-frames_begin,
                      internal_missed_scans=int(run.status['internal_missed_scans']),
                      compute_p99_ms=float(run.status['compute_p99_ms']),bridge_p99_ms=float(run.status['bridge_p99_ms']),
                      peak_reported_compute_p99_ms=peak_compute,peak_reported_bridge_p99_ms=peak_bridge,
                      received_clouds=run.clouds,observed_delivery_gaps=run.delivery_gaps,hashes=run.hashes)
        run.pause(True);result['same_state_rss_delta']=run.resources['sim'].memory_info().rss-baseline
        result['resources']={key:{'cpu_average':v['cpu_sum']/v['cpu_count'],'cpu_p95':percentile(v['cpu'],.95),
                                  'cpu_max':v['cpu_max'],'rss_peak':v['rss_peak']} for key,v in samples.items()}
        result['timing_window_frames']=int(run.status['timing_window_frames'])
        if soak:
            xs=[x/3600 for x,y in slope_samples];ys=[y/1024**2 for x,y in slope_samples]
            mx=sum(xs)/len(xs);my=sum(ys)/len(ys)
            slope=sum((x-mx)*(y-my) for x,y in zip(xs,ys))/sum((x-mx)**2 for x in xs)
            result['rss_slope_mib_per_hour']=slope;result['ten_minute_samples']=slope_samples
            result['status']='PASS' if duration>=8*3600 and wall>=8*3600 and slope<=1 and result['same_state_rss_delta']<=32*1024**2 and samples['sim']['rss_peak']<=1024**3 else 'FAIL'
        else:
            result['status']='PASS' if ((rtf==0 and result['rtf']>=1) or (rtf==1 and abs(result['rtf']-1)<=.02)) and result['internal_missed_scans']==0 and peak_compute<=80 and peak_bridge<=20 and samples['sim']['rss_peak']<=1024**3 else 'FAIL'
    except Exception as error:
        result.update(status='ERROR',error=repr(error))
    finally:
        if run:
            exits=run.close();result['exits']=exits
            if any(exits.values()):result['status']='FAIL'
        directory.mkdir(parents=True,exist_ok=True)
        (directory/'result.json').write_text(json.dumps(result,indent=2))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--mode',choices=['performance','soak','explore'],default='performance')
    parser.add_argument('--warmup',type=float,default=30);parser.add_argument('--duration',type=float)
    parser.add_argument('--resume',action='store_true');parser.add_argument('--domain',type=int,default=162);parser.add_argument('--repeat',type=int,default=5);parser.add_argument('--gui',choices=['off','on','both'],default='both')
    args=parser.parse_args();args.output=args.output.resolve();args.config=args.config.resolve()
    args.output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']=str(args.domain);os.environ['ROS_LOCALHOST_ONLY']='1';os.environ['ROS_LOG_DIR']=str(args.output/'ros_logs')
    b0=environment();b0['fixture_sha256']=hashlib.sha256(args.config.read_bytes()).hexdigest()
    b0['started_wall_time_ns']=time.time_ns()
    if args.resume and (args.output/'B0.json').exists():
        old=json.loads((args.output/'B0.json').read_text())
        if old['fixture_sha256']!=b0['fixture_sha256']:raise ValueError('resume fixture differs')
        (args.output/f'B0-resume-{time.time_ns()}.json').write_text(json.dumps(b0,indent=2))
    else:(args.output/'B0.json').write_text(json.dumps(b0,indent=2))
    duration=args.duration if args.duration is not None else (8*3600 if args.mode=='soak' else 120)
    cases=[(1,False,0)] if args.mode=='soak' else [(rtf,gui,i) for gui in ([False,True] if args.gui=='both' else [args.gui=='on']) for rtf in (0,1) for i in range(args.repeat)]
    results=[]
    for rtf,gui,index in cases:
        name=f'rtf-{rtf}-gui-{int(gui)}-{index}'
        existing=args.output/name/'result.json'
        previous=json.loads(existing.read_text()) if args.resume and existing.exists() else None
        if previous and previous.get('status')=='PASS' and previous.get('warmup_seconds')==args.warmup and previous.get('duration_seconds')==duration:
            result=previous
        else:
            if args.resume and (args.output/name).exists():
                archive=args.output/'interrupted';archive.mkdir(exist_ok=True)
                (args.output/name).rename(archive/(name+'-'+str(time.time_ns())))
            result=measure(args.config,args.output/name,rtf,gui,args.warmup,duration,args.mode=='soak')
        results.append(result);print(json.dumps({'case':name,'status':result['status'],'rtf':result.get('rtf'),'error':result.get('error')}),flush=True)
        (args.output/'results.json').write_text(json.dumps(results,indent=2))
    failures=sum(r['status']!='PASS' for r in results)
    suite=ET.Element('testsuite',name=MILESTONE+'_'+args.mode,tests=str(len(results)),failures=str(failures))
    for index,result in enumerate(results):
        case=ET.SubElement(suite,'testcase',name=f'case-{index}')
        if result['status']!='PASS':ET.SubElement(case,'failure',message=json.dumps(result)[:2000])
    ET.ElementTree(suite).write(args.output/'result.xml',encoding='utf-8',xml_declaration=True)
    return int(failures>0)

if __name__=='__main__':raise SystemExit(main())
