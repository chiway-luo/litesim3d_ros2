"""M2 real DDS/ROS acceptance; external SLAM/navigation algorithms are out of this run's scope."""
import json
import math
import os
from pathlib import Path
import signal
import struct
import sys
import time
import psutil
import pytest
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, Imu
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from litesim3d_interfaces.srv import Reset
from ament_index_python.packages import get_package_prefix
from test_ros import Harness, h, ns


def m2_config(h):
    h.config['lidars'][0]['scan_mode']='rolling'
    h.config['imus']=[{'name':'imu','link':'base_link','topic':'imu/data_raw','rate_hz':200}]
    h.config['simulation']['threads']=4


def test_IMU07_real_ros_sixty_sim_seconds(h):
    m2_config(h)
    h.subscribe('imu',Imu,'/imu/data_raw')
    h.subscribe('points',PointCloud2,'/points_raw')
    h.rsp();h.sim(start_paused=True,sensor_reliable=True);h.ready()
    h.until(lambda:all(json.loads(h.status().get('subscription_matches','{}')).get(topic,0)>=1
                       for topic in ['/points_raw','/imu/data_raw']))
    # Small batches let the real executor drain all 200 Hz samples without manufacturing counts.
    for i in range(600):
        assert h.step(20,f'batch-{i}').success
        h.until(lambda:len(h.messages['imu']) >= (i+1)*20,timeout=2)
    assert len(h.messages['imu'])==12000
    assert len(h.messages['points'])==600
    assert [ns(i.header.stamp) for i in h.messages['imu']]==[1+i*5_000_000 for i in range(1,12001)]
    assert [ns(c.header.stamp) for c in h.messages['points']]==[1+i*100_000_000 for i in range(600)]
    for m in h.messages['imu']:
        assert m.orientation_covariance[0]==-1
        assert m.linear_acceleration.z==pytest.approx(9.80665,abs=1e-9)
        assert m.header.frame_id=='base_link'
    for c in h.messages['points']:
        times=[v[-1] for v in struct.iter_unpack('<ffffH2xf',bytes(c.data))]
        assert min(times)==0 and 0<max(times)<.1
        assert ns(c.header.stamp)+round(max(times)*1e9)<=ns(h.messages['imu'][-1].header.stamp)


def start_session(h, member_args=(), *, timeout=15, wait_ready=True, ready_topics=None, sensor_reliable=True):
    import yaml
    m2_config(h)
    config=h.directory/'config.yaml';config.write_text(yaml.safe_dump(h.config))
    members=h.directory/'members.json'
    # Single-step assertions require reliable delivery at both ends. Best-effort
    # compatibility is tested separately; a matched endpoint can still lose a sample.
    member_args=(*member_args, *(['--reliable-sensors'] if sensor_reliable else []))
    member={'name':'m2_consumer','argv':[sys.executable,str(Path(__file__).with_name('measurement_consumer.py')),*member_args]}
    if ready_topics is not None:
        member['ready_topics']=ready_topics
    members.write_text(json.dumps([member]))
    session_dir=h.directory/'session'
    p=h.process('session',[str(Path(get_package_prefix('litesim3d_ros2'))/'lib/litesim3d_ros2/sim_session'),
                         '--config',str(config),'--output',str(session_dir),'--members',str(members),
                         '--observer','m1_test','--timeout',str(timeout),
                         *(['--sensor-reliable'] if sensor_reliable else [])])
    h.ready()
    state=session_dir/'epoch-0001/consumer.json'
    h.until(lambda:state.exists() and json.loads(state.read_text())['subscriptions_created'])
    if wait_ready:
        h.until(lambda:any(r['event']=='ready' for r in session_records(session_dir)),timeout=timeout+2)
        ready=next(r for r in session_records(session_dir) if r['event']=='member_ready')
        assert {'/points_raw','/imu/data_raw','/odom'}<=set(ready['topics'])
    return p,session_dir


def session_records(directory):
    return [json.loads(row) for row in (directory/'session.jsonl').read_text().splitlines()]


def reset(h,run,ident,**kwargs):
    r=Reset.Request();r.request_id=ident;r.expected_run_id=run
    for key,value in kwargs.items():setattr(r,key,value)
    return h.call(Reset,'/sim/reset',r,timeout=20)


@pytest.mark.parametrize('reliable',[False,True])
def test_session_sensor_qos_defaults_and_reliable_option(h, reliable):
    process,directory=start_session(h,sensor_reliable=reliable)
    expected=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT
    for topic in ['/points_raw','/imu/data_raw']:
        def endpoints():
            return (h.node.get_publishers_info_by_topic(topic),
                    [s for s in h.node.get_subscriptions_info_by_topic(topic)
                     if s.node_name=='m2_consumer'])
        # The session and this observer have independent DDS graph caches.
        # Session readiness does not make the observer's graph update atomic.
        h.until(lambda:all(len(group)==1 for group in endpoints()))
        publishers,consumers=endpoints()
        assert publishers[0].qos_profile.reliability==consumers[0].qos_profile.reliability==expected
    h.pause(False)
    state=directory/'epoch-0001/consumer.json'
    h.until(lambda:json.loads(state.read_text())['points']>=2)
    assert process.poll() is None


def test_ROS09_reset_rejects_uncoordinated_and_stale_requests(h):
    p,directory=start_session(h);old=h.status()['run_id']
    assert not reset(h,'wrong','stale').success
    outsider=Node('unowned_consumer',context=h.context)
    sub=outsider.create_subscription(PointCloud2,'/points_raw',lambda m:None,qos_profile_sensor_data)
    h.until(lambda:any(e.node_name=='unowned_consumer' for e in h.node.get_subscriptions_info_by_topic('/points_raw')))
    result=reset(h,old,'outside');assert not result.success and 'uncoordinated' in result.message
    outsider.destroy_node()
    h.until(lambda:all(e.node_name!='unowned_consumer' for e in h.node.get_subscriptions_info_by_topic('/points_raw')))
    r=Reset.Request();r.request_id='invalid-pose';r.expected_run_id=old;r.set_initial_pose=True;r.initial_pose.orientation.w=1.0;r.initial_pose.position.x=5.0
    result=h.call(Reset,'/sim/reset',r);assert not result.success and result.run_id==old
    assert p.poll() is None
    assert h.step(10,'half-frame').success
    result=reset(h,old,'valid');assert result.success and result.run_id!=old
    same=reset(h,old,'valid');assert same.success and same.run_id==result.run_id
    assert h.step(20,'first-new-frame').success
    state_path=directory/'epoch-0002/consumer.json'
    h.until(lambda:state_path.exists() and json.loads(state_path.read_text())['points']==1)
    assert json.loads(state_path.read_text())['first_stamp']==1
    h.subscribe('teleport_truth',Odometry,'/sim/ground_truth/robot_0/odom')
    h.subscribe('teleport_imu',Imu,'/imu/data_raw')
    request=Reset.Request();request.request_id='pose-update';request.expected_run_id=result.run_id
    request.set_initial_pose=True;request.initial_pose.orientation.w=1.0;request.initial_pose.position.x=.5
    moved=h.call(Reset,'/sim/reset',request,timeout=20);assert moved.success,moved.message
    assert moved.run_id!=result.run_id
    h.messages['teleport_truth'].clear();h.messages['teleport_imu'].clear()
    assert h.step(20,'pose-new-first').success
    h.until(lambda:bool(h.messages['teleport_truth']) and bool(h.messages['teleport_imu']))
    assert h.messages['teleport_truth'][-1].pose.pose.position.x==pytest.approx(.5)
    for imu in h.messages['teleport_imu']:
        assert imu.linear_acceleration.z==pytest.approx(9.80665)
        assert math.isfinite(imu.linear_acceleration.x) and abs(imu.linear_acceleration.x)<1e-9


def test_ROB02_ROB07_hundred_full_resets_and_first_frames(h):
    p,directory=start_session(h)
    seen=set();rss=[];old_pid=None
    for cycle in range(101):
        epoch=directory/f'epoch-{cycle+1:04d}';state_path=epoch/'consumer.json'
        h.until(lambda:state_path.exists())
        before=json.loads(state_path.read_text());run=before['run_id']
        ready=next(r for r in session_records(directory)
                   if r['event']=='member_ready' and r['epoch']==cycle+1)
        assert {'/points_raw','/imu/data_raw','/odom'}<=set(ready['topics'])
        assert run not in seen;seen.add(run)
        if old_pid is not None:assert not psutil.pid_exists(old_pid)
        old_pid=before['pid']
        assert before['points']==0 and before['first_stamp'] is None
        assert h.step(20,f'first-{cycle}').success
        h.until(lambda:json.loads(state_path.read_text())['points']>=1)
        state=json.loads(state_path.read_text());assert state['first_stamp']==1
        assert state['run_id']==run
        graph=h.node.get_subscriptions_info_by_topic('/sim/ground_truth/robot_0/odom')
        assert all(e.node_name!='m2_consumer' for e in graph)
        rss.append(psutil.Process(p.pid).memory_info().rss)
        assert h.step(10,f'half-{cycle}').success
        if cycle<100:
            result=reset(h,run,f'cycle-{cycle}')
            assert result.success,result.message
    assert max(rss)<1024**3
    assert rss[-1]-rss[5]<32*1024**2
    (h.directory/'reset-metrics.json').write_text(json.dumps({'resets':100,'run_ids':sorted(seen),'supervisor_rss':rss}))


@pytest.mark.parametrize('ready_topics',[None,['/points_raw','/imu/data_raw','/odom']])
def test_reset_waits_for_delayed_sensor_subscriptions(h, ready_topics):
    process,directory=start_session(h,('--subscription-delay','1.5'),ready_topics=ready_topics)
    old=h.status()['run_id']
    result=reset(h,old,'delayed-subscriptions')
    assert result.success,result.message
    returned_ns=time.time_ns()
    assert h.step(20,'immediate-first-frame').success
    state_path=directory/'epoch-0002/consumer.json'
    h.until(lambda:state_path.exists() and json.loads(state_path.read_text())['subscriptions_created'])
    state=json.loads(state_path.read_text())
    (h.directory/'readiness-timing.json').write_text(json.dumps({
        'reset_returned_ns':returned_ns,'consumer':state,'run_id':result.run_id},indent=2))
    h.until(lambda:json.loads(state_path.read_text())['points']==1)
    assert returned_ns>=state['subscriptions_created_ns']
    assert returned_ns>=state['executor_started_ns']
    assert json.loads(state_path.read_text())['first_stamp']==1
    assert process.poll() is None


def test_reset_fails_if_previous_sensor_subscription_is_missing(h):
    process,directory=start_session(h,('--skip-points-on-reset',))
    assert h.step(20,'old-first-frame').success
    old_state=directory/'epoch-0001/consumer.json'
    h.until(lambda:json.loads(old_state.read_text())['points']==1)
    result=reset(h,h.status()['run_id'],'missing-new-subscription')
    assert not result.success
    assert 'm2_consumer subscription /points_raw' in result.message
    h.until(lambda:process.poll() is not None)
    assert process.returncode!=0
    assert not any(r['event']=='ready' and r['epoch']==2 for r in session_records(directory))


@pytest.mark.parametrize('arguments,reason',[
    (('--executor-delay','20'),('m2_consumer parameter service','m2_consumer executor response')),
    (('--reliable-points',),('incompatible consumer subscription: m2_consumer /points_raw',)),
])
def test_session_rejects_unready_consumer(h, arguments, reason):
    process,directory=start_session(h,arguments,timeout=8,wait_ready=False,
                                    ready_topics=['/points_raw'],sensor_reliable=False)
    h.until(lambda:process.poll() is not None,timeout=16)
    assert process.returncode!=0
    records=session_records(directory)
    assert not any(r['event']=='ready' for r in records)
    # A non-spinning member may time out during service discovery or RPC response,
    # depending on RMW discovery order; both must fail before reporting ready.
    assert any(any(detail in r.get('reason','') for detail in reason) for r in records)


def test_session_cpp_adapter_receives_first_frame_after_reset(h):
    import yaml
    m2_config(h)
    config=h.directory/'config.yaml';config.write_text(yaml.safe_dump(h.config))
    executable=Path(get_package_prefix('litesim3d_ros2'))/'lib/litesim3d_ros2'
    members=h.directory/'members.json'
    members.write_text(json.dumps([{'name':'sim_ouster_adapter',
        'argv':[str(executable/'sim_ouster_adapter'),'--ros-args','-p','sensor_reliable:=true'],
        'ready_topics':['/points_raw']}]))
    directory=h.directory/'session'
    h.subscribe('adapted',PointCloud2,'/points_ouster')
    h.subscribe('adapter_status',String,'/adapter_status')
    def wait_adapter(run):
        def matched():
            if not h.messages['adapter_status']:
                return False
            status=json.loads(h.messages['adapter_status'][-1].data)
            return (status['run_id']==run and status['input_publishers']==1
                    and status['output_subscribers']==1)
        h.until(matched)
    process=h.process('session',[str(executable/'sim_session'),'--config',str(config),
        '--output',str(directory),'--members',str(members),'--observer','m1_test','--sensor-reliable'])
    h.until(lambda:(directory/'session.jsonl').exists() and
            any(r['event']=='ready' for r in session_records(directory)),timeout=17)
    wait_adapter(h.status()['run_id'])
    assert h.step(20,'adapter-old-frame').success
    h.until(lambda:len(h.messages['adapted'])==1)
    result=reset(h,h.status()['run_id'],'adapter-new-epoch')
    assert result.success,result.message
    h.messages['adapted'].clear()
    wait_adapter(result.run_id)
    assert h.step(20,'adapter-new-first-frame').success
    h.until(lambda:len(h.messages['adapted'])==1)
    assert ns(h.messages['adapted'][0].header.stamp)==1
    assert process.poll() is None


def test_reset_initial_pose_preserves_relative_lidar_pattern(h):
    patterns = h.directory / 'patterns'
    patterns.mkdir()
    (patterns / 'scan.csv').write_text('Time/s,Azimuth/deg,Zenith/deg\n1,0,90\n2,30,80\n')
    lidar = h.config['lidars'][0]
    lidar.pop('vertical_angles_deg')
    lidar.update(scan_pattern='livox_mid360', pattern_file='patterns/scan.csv',
                 rings=1, columns=20000, rate_hz=10)
    process, directory = start_session(h)
    old_run = h.status()['run_id']
    request = Reset.Request()
    request.request_id = 'relative-pattern-pose'
    request.expected_run_id = old_run
    request.set_initial_pose = True
    request.initial_pose.position.x = .5
    request.initial_pose.orientation.w = 1.
    result = h.call(Reset, '/sim/reset', request, timeout=20)
    assert result.success, result.message
    assert result.run_id != old_run and process.poll() is None
    assert h.step(20, 'first-pattern-frame').success
    state = directory / 'epoch-0002/consumer.json'
    h.until(lambda: state.exists() and json.loads(state.read_text())['points'] >= 1)
    assert json.loads(state.read_text())['first_stamp'] == 1


def test_ROS12_required_process_crash_is_nonzero(h):
    p,directory=start_session(h)
    records=[json.loads(row) for row in (directory/'session.jsonl').read_text().splitlines()]
    child=next(row['pid'] for row in records if row['event']=='spawn' and row['name']=='m2_consumer')
    os.kill(child,signal.SIGKILL)
    h.until(lambda:p.poll() is not None,timeout=10)
    assert p.returncode!=0
    h.until(lambda:not any(psutil.pid_exists(r['pid']) for r in records if r['event']=='spawn'))


def test_ROS07_slow_consumer_queue_and_thirty_second_pause(h):
    m2_config(h)
    h.subscribe('points',PointCloud2,'/points_raw',reliable=False)
    h.rsp();p=h.sim();h.ready();h.until(lambda:len(h.messages['points'])>=3)
    proc=psutil.Process(p.pid);before=proc.memory_info().rss
    # Model an independent consumer with its own ROS context; the fast
    # observer and deliberately stalled reader have separate participants.
    slow_context=Context();rclpy.init(context=slow_context)
    slow=Node('slow_consumer',context=slow_context)
    slow_executor=SingleThreadedExecutor(context=slow_context);slow_executor.add_node(slow)
    received=[];polls=[];clock_offsets=[]
    slow_sub=slow.create_subscription(PointCloud2,'/points_raw',lambda m:received.append(ns(m.header.stamp)),qos_profile_sensor_data)
    try:
        h.until(lambda:json.loads(h.status().get('subscription_matches','{}')).get('/points_raw',0)>=2)
        clock_offsets.append(time.time_ns()-time.monotonic_ns())
        for _ in range(10):
            # Make the consumer slow relative to actual production, after DDS
            # matching, so each poll follows a known backlog of new scans.
            target=len(h.messages['points'])+10
            h.until(lambda:len(h.messages['points'])>=target)
            # spin_once may handle another entity or a timeout. One consumer
            # poll means one cloud callback, not merely one executor call.
            previous=len(received);deadline=time.monotonic()+2
            while len(received)==previous and time.monotonic()<deadline:
                slow_executor.spin_once(timeout_sec=.05)
            assert len(received)==previous+1
            latest=ns(h.messages['points'][-1].header.stamp)
            polls.append({'latest':latest,'received':received.copy(),'lag_ns':latest-received[-1]})
            clock_offsets.append(time.time_ns()-time.monotonic_ns())
        # DDS source timestamps use system time, independently of /clock. A
        # backwards system-clock step can poison Fast DDS 2.6 KEEP_LAST history.
        # Record that condition without skipping/retrying a failing assertion.
        clock_steps=[b-a for a,b in zip(clock_offsets,clock_offsets[1:])]
        period_ns=round(10**9/h.config['lidars'][0]['rate_hz'])
        max_lag_ns=(slow_sub.qos_profile.depth+2)*period_ns
        (h.directory/'slow-consumer.json').write_text(json.dumps({
            'polls':polls,'status':h.status(),'clock_offset_steps_ns':clock_steps,
            'max_allowed_lag_ns':max_lag_ns},indent=2))
        assert 5<=len(received)<=10
        assert received[-1]-received[0]>5*10**9
        # A broad timestamp span can pass while the reader remains seconds
        # behind. Bound every poll by its five-frame queue plus two scan periods
        # of delivery/observation tolerance, not by the slow polling interval.
        assert all(poll['lag_ns']<=max_lag_ns for poll in polls), {
            'polls':polls,'clock_offset_steps_ns':clock_steps,
            'max_allowed_lag_ns':max_lag_ns}
    finally:
        slow_executor.shutdown();slow.destroy_node();slow_context.shutdown()
    before_pause_count=len(h.messages['points'])
    # Stop the receiving executor for 30 wall seconds. DDS and simulator continue.
    time.sleep(30)
    middle=proc.memory_info().rss
    h.spin(.3)
    instrumented=os.environ.get('LWS_INSTRUMENTED_MEMORY') in ('asan','tsan')
    (h.directory/'slow-consumer-memory.json').write_text(json.dumps({
        'rss_before':before,'rss_after':middle,'delta_bytes':middle-before,
        'rss_budget_enforced':not instrumented,'budget_bytes':32*1024**2,
        'allocator':os.environ.get('LWS_INSTRUMENTED_MEMORY','normal')},indent=2))
    # Sanitizer quarantine/shadow memory is not the production allocator's RSS.
    # Its separate leak/race gate remains mandatory; normal builds keep the cap.
    if not instrumented:
        assert middle-before<32*1024**2
    h.until(lambda:len(h.messages['points'])>=6)
    status=h.status();assert status['sensor_queue_depth']=='5'
    assert 'keep_last' in status['sensor_queue_policy']
    assert int(status['internal_missed_scans'])==0
    stamps=[ns(c.header.stamp) for c in h.messages['points']]
    assert stamps[-1]>25*10**9
    assert len(stamps)-before_pause_count<150 # bounded late replay, not all 300 frames
    assert max(b-a for a,b in zip(stamps,stamps[1:]))>10**9
