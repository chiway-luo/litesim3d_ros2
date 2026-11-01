"""M3 acceptance through real DDS, RSP, and the installed PCL algorithm."""
import copy
import json
import math
from pathlib import Path
import struct
import signal

import pytest
import yaml
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField, Imu
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from visualization_msgs.msg import MarkerArray
from test_ros import Harness, h, ns
from test_m2_ros import m2_config


def executable(name):
    return str(Path(get_package_prefix('litesim3d_ros2'))/'lib/litesim3d_ros2'/name)


def world_transforms_ready(h, count):
    """Wait for the listener, not another topic's independently queued callback."""
    h.until(lambda: all(h.buffer.can_transform(f'r{k}/odom', f'r{k}/lidar_link', Time())
                        for k in range(count)))


def world_tf_pairs(h, count):
    """Observe the raw TF collector before checking its complete robot edges."""
    required = {(f'r{k}/odom', f'r{k}/base_link') for k in range(count)}

    def collected():
        return {(t.header.frame_id, t.child_frame_id)
                for message in h.messages['tf'] for t in message.transforms}

    # TransformListener and the raw /tf collector are separate subscriptions.
    # Buffer.can_transform() does not establish that the collector callback ran.
    h.until(lambda: required <= collected())
    pairs = collected()
    assert all(a.split('/')[0] == b.split('/')[0] for a, b in pairs)
    assert required <= pairs
    return pairs


def start_world(h, count, *, ready=True, command_timeout=100.0, range_boundary=False, mid360=False, rsp=True, **settings):
    m2_config(h)
    h.config['entities'] = []
    h.config['simulation']['command_timeout'] = command_timeout
    h.config['robot']['drive']['unlimited_acceleration'] = True
    # Lower the fixture sensor to the peer chassis height; no synthetic point injection.
    urdf = h.directory/'peer.urdf'
    urdf.write_text(Path(h.config['robot']['urdf']).read_text().replace('0.2 0 0.65', '0.2 0 0.3'))
    h.config['robot']['urdf'] = str(urdf)
    h.config['lidars'][0].update(rings=1, vertical_angles_deg=[0.0], self_filter='robot',
                               horizontal_start_deg=0.0, horizontal_end_deg=1.0,
                               publish_range_boundary=range_boundary)
    if mid360:
        h.config['lidars'][0].update(scan_pattern='livox_mid360',
            pattern_file='package://litesim3d_core/lidar_patterns/mid360.csv',
            columns=20000, rate_hz=10.0, horizontal_start_deg=-180.0, horizontal_end_deg=180.0)
    config = h.directory/'base.yaml'
    config.write_text(yaml.safe_dump(h.config))
    robots = [{'id':f'r{i}', 'xyz':xyz} for i, xyz in enumerate([[0,0,0],[2,0,0],[0,3,0],[2,3,0]][:count])]
    world = h.directory/'world.yaml'
    world.write_text(yaml.safe_dump({'schema_version':1, 'config':str(config), 'robots':robots}))
    for r in robots:
        name = r['id']
        params = h.directory/(name+'-rsp.yaml')
        params.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
            'robot_description':urdf.read_text(), 'frame_prefix':name+'/',
            'use_sim_time':True, 'publish_frequency':200.0}}}))
        if rsp:
            h.process(name+'-rsp', [str(Path(get_package_prefix('robot_state_publisher'))/'lib/robot_state_publisher/robot_state_publisher'),
                                   '--ros-args', '--params-file', str(params), '-r', '__ns:=/'+name])
        h.subscribe(name+'-odom', Odometry, '/'+name+'/odom')
        reliable = settings.get('sensor_reliable', False)
        h.subscribe(name+'-cloud', PointCloud2, '/'+name+'/points_raw', reliable=reliable)
        if range_boundary:
            h.subscribe(name+'-boundary', PointCloud2, '/'+name+'/points_raw/range_boundary', reliable=reliable)
        h.subscribe(name+'-imu', Imu, '/'+name+'/imu/data_raw', reliable=reliable)
    params = h.directory/'world-params.yaml'
    params.write_text(yaml.safe_dump({'/**':{'ros__parameters':{
        'config':str(world), 'use_sim_time':True, 'start_paused':True, 'manifest_path':str(h.directory/'world-manifest.json'), **settings}}}))
    process = h.process('world', [executable('sim_world_node'), '--ros-args', '--params-file', str(params)])
    if ready:
        h.ready()
        required = [f'/{r["id"]}/{topic}' for r in robots
                    for topic in ['points_raw', 'imu/data_raw'] +
                    (['points_raw/range_boundary'] if range_boundary else [])]
        def matched():
            counts = {}
            for status in h.messages['status'][-1].status:
                data = {v.key:v.value for v in status.values}
                counts.update(json.loads(data.get('subscription_matches', '{}')))
            return all(counts.get(topic, 0) >= 1 for topic in required)
        h.until(matched)
    return process


def test_world_range_boundary_is_namespaced_and_respects_peer_occlusion(h):
    start_world(h, 2, range_boundary=True)
    assert h.step(20, 'boundary-frame').success
    h.until(lambda: all(h.messages[f'r{k}-{kind}'] for k in range(2) for kind in ('cloud', 'boundary')))
    for k in range(2):
        raw = h.messages[f'r{k}-cloud'][-1]
        boundary = h.messages[f'r{k}-boundary'][-1]
        assert boundary.header == raw.header
        assert boundary.header.frame_id == f'r{k}/lidar_link'
        for rv, bv in zip(struct.iter_unpack('<ffffH2xf', bytes(raw.data)),
                          struct.iter_unpack('<ffffH2xf', bytes(boundary.data))):
            if k == 0:  # All rays hit the nearby peer; no synthetic points behind it.
                assert math.isfinite(rv[0]) and math.isnan(bv[0])
            else:
                assert math.isnan(rv[0])
                assert math.sqrt(sum(v*v for v in bv[:3])) == pytest.approx(h.config['lidars'][0]['max_range_m'], abs=2e-5)
            assert bv[4:] == rv[4:]
    assert h.node.count_publishers('/points_raw/range_boundary') == 0


def test_mid360_world_keeps_peer_occlusion_and_separate_clouds(h):
    start_world(h, 2, range_boundary=True, mid360=True, sensor_reliable=True)
    assert h.step(20, 'mid360-world-frame').success
    h.until(lambda: all(h.messages[f'r{k}-{kind}'] for k in range(2) for kind in ('cloud', 'boundary')))
    counts = []
    for k in range(2):
        raw, boundary = (h.messages[f'r{k}-{kind}'][-1] for kind in ('cloud', 'boundary'))
        assert raw.header == boundary.header
        assert raw.header.frame_id == f'r{k}/lidar_link'
        assert (raw.height, raw.width, raw.point_step) == (1, 20000, 24)
        hits = empty = masked = 0
        for rv, bv in zip(struct.iter_unpack('<ffffH2xf', bytes(raw.data)),
                          struct.iter_unpack('<ffffH2xf', bytes(boundary.data))):
            if math.isfinite(rv[0]):
                assert math.isnan(bv[0])  # The peer blocks the far-range helper.
                assert 1 < math.sqrt(sum(v*v for v in rv[:3])) < 3
                hits += 1
            elif math.isfinite(bv[0]):
                assert math.sqrt(sum(v*v for v in bv[:3])) == pytest.approx(h.config['lidars'][0]['max_range_m'], abs=2e-5)
                empty += 1
            else:
                masked += 1
            assert bv[4:] == rv[4:]
        assert hits > 0 and empty > 19000 and masked > 0
        counts.append({'peer_hits': hits, 'boundary': empty, 'masked': masked})
    assert h.node.count_publishers('/points_raw') == 0
    assert h.node.count_publishers('/clock') == 1
    (h.directory/'mid360-world-check.json').write_text(json.dumps(counts, indent=2))


@pytest.mark.parametrize('count', [2, 4])
def test_EXT03_namespaces_clock_tf_commands_and_peer_cloud(h, count):
    start_world(h, count)
    command = h.node.create_publisher(Twist, '/r0/cmd_vel', 1)
    h.until(lambda:command.get_subscription_count()==1)
    value = Twist(); value.linear.x=.2
    h.publish_paused_command(command, value, 'r0')
    for i in range(10):
        assert h.step(20, f'block-{i}').success
        h.until(lambda:all(len(h.messages[f'r{k}-cloud'])>=i+1 for k in range(count)))
    # Service responses, clock, clouds and each robot's odometry have independent
    # DDS/callback delivery order. Compare poses only after the requested step is
    # observed on every odometry stream, not merely after its first message.
    expected_time = 1_000_000_001
    h.wait_for_stamp(expected_time, 'clock', *(f'r{k}-odom' for k in range(count)))
    observed_times = {'clock': ns(h.messages['clock'][-1].clock),
                      'odom': {f'r{k}': ns(h.messages[f'r{k}-odom'][-1].header.stamp)
                               for k in range(count)}}
    assert h.node.count_publishers('/clock') == 1
    assert ns(h.messages['clock'][-1].clock) == expected_time
    assert h.messages['r0-odom'][-1].pose.pose.position.x == pytest.approx(.2, abs=1e-8)
    for k in range(1,count):
        assert h.messages[f'r{k}-odom'][-1].pose.pose.position.x == 0
        assert h.messages[f'r{k}-odom'][-1].pose.pose.position.y == 0
    world_transforms_ready(h, count)
    for k in range(count):
        cloud = h.messages[f'r{k}-cloud'][-1]
        assert cloud.header.frame_id == f'r{k}/lidar_link'
        assert ns(cloud.header.stamp) == 900_000_001
        assert h.messages[f'r{k}-imu'][-1].header.frame_id == f'r{k}/base_link'
        assert h.buffer.can_transform(f'r{k}/odom', f'r{k}/lidar_link', __import__('rclpy').time.Time())
    # Independent plane reference for the moving observer and stationary peer box.
    cloud = h.messages['r0-cloud'][-1]
    for x,y,z,intensity,ring,t in struct.iter_unpack('<ffffH2xf', bytes(cloud.data)):
        assert x == pytest.approx(2-.4-.2-.2*(.9+t), abs=1e-5)
    pairs = world_tf_pairs(h, count)
    cached = h.step(20, 'block-9')
    assert cached.success and cached.time_ns==1_000_000_001
    assert not h.step(21, 'block-9').success
    rejected = int(h.status('r0')['rejected_commands'])
    bad=Twist(); bad.linear.y=.5;command.publish(bad)
    h.until(lambda: int(h.status('r0')['rejected_commands']) == rejected + 1)
    assert h.step(20,'invalid-command').success
    assert int(h.status()['rejected_commands'])>=1
    (h.directory/'world-observations.json').write_text(json.dumps({
        'count':count,'tf_edges':sorted(pairs),'clock_publishers':h.node.count_publishers('/clock'),
        'pose_comparison_times':observed_times,
        'point_plane_reference':'2 - 0.4 - 0.2 - 0.2 * (0.9 + t)',
        'statuses':[{v.key:v.value for v in s.values} for s in h.messages['status'][-1].status]}, indent=2))


def test_EXT03_duplicate_clock_faults_world(h):
    process=start_world(h,2)
    publisher=h.node.create_publisher(Clock,'/clock',1)
    publisher.publish(Clock())
    h.until(lambda:process.poll() is not None)
    assert process.returncode!=0
    manifest=json.loads((h.directory/'world-manifest.json').read_text())
    assert manifest['exit_reason']=='fault' and 'clock' in manifest['error']


@pytest.mark.parametrize('rtf',[0.0,1.0])
def test_EXT04_shared_world_sigint_during_active_publishing(h,rtf):
    h.config['simulation']['real_time_factor']=rtf
    process=start_world(h,4)
    # The shutdown observer only needs one cloud and diagnostics. A Python TF listener
    # plus 4x200 Hz IMU subscriptions becomes an unrelated slow-consumer stress at RTF > 10.
    from rclpy.parameter import Parameter
    h.node.set_parameters([Parameter('use_sim_time',value=False)])
    h.listener.unregister()
    for subscription in h.subs:
        if subscription.topic_name not in ('/sim/status','/r0/points_raw'):
            h.node.destroy_subscription(subscription)
    h.pause(False)
    h.until(lambda:len(h.messages['r0-cloud'])>=3)
    process.send_signal(signal.SIGINT)
    h.until(lambda:process.poll() is not None)
    assert process.returncode==0
    manifest=json.loads((h.directory/'world-manifest.json').read_text())
    assert manifest['exit_reason']=='shutdown'
    assert all(r['status']['state']=='exited' for r in manifest['robots'])


def test_EXT05_ackermann_feedback_loop_from_measured_odom(h):
    m2_config(h);h.config['entities']=[]
    h.config['robot']['motion_model']='ackermann'
    h.config['robot']['drive'].update(wheelbase=1.0,max_steering_rad=.6,unlimited_acceleration=True)
    h.subscribe('odom',Odometry,'/odom')
    h.rsp();h.sim(start_paused=True);h.ready()
    publisher=h.node.create_publisher(Twist,'/cmd_vel',1)
    h.until(lambda:publisher.get_subscription_count()==1)
    assert h.step(4,'initial').success;h.until(lambda:bool(h.messages['odom']))
    # Smooth 3 m radius arc; heading and cross-track feedback use only /odom.
    trace=[]
    for i in range(160):
        observed=h.messages['odom'][-1].pose.pose
        theta=2*math.atan2(observed.orientation.z,observed.orientation.w)
        phase=i*.05/6
        reference=(3*math.sin(phase),3*(1-math.cos(phase)))
        dx,dy=reference[0]-observed.position.x,reference[1]-observed.position.y
        along=math.cos(theta)*dx+math.sin(theta)*dy
        cross=-math.sin(theta)*dx+math.cos(theta)*dy
        value=Twist();value.linear.x=max(.1,min(.7,.5+along));value.angular.z=1/6+1.5*cross+math.sin(phase-theta)
        h.publish_paused_command(publisher, value)
        before=ns(h.messages['odom'][-1].header.stamp)
        assert h.step(10,f'feedback-{i}').success
        h.until(lambda:ns(h.messages['odom'][-1].header.stamp)>before)
        o=h.messages['odom'][-1]
        assert abs(o.twist.twist.angular.z)<=abs(o.twist.twist.linear.x)*math.tan(.6)+1e-9
        trace.append([i,o.pose.pose.position.x,o.pose.pose.position.y,o.twist.twist.linear.x,o.twist.twist.angular.z])
    end=h.messages['odom'][-1].pose.pose.position
    target=(3*math.sin(8/6),3*(1-math.cos(8/6)))
    error=math.hypot(end.x-target[0],end.y-target[1])
    assert error<.05
    (h.directory/'feedback.json').write_text(json.dumps({'target':target,'position_error':error,'trace':trace}))


def test_EXT01_configured_obstacle_stays_fixed_in_ros_points_and_visuals(h):
    m2_config(h)
    h.config['lidars'][0].update(rings=1,vertical_angles_deg=[0.0],columns=100,
                               horizontal_start_deg=0.0,horizontal_end_deg=.5,self_filter='robot')
    h.config['entities']=[{'id':'moving','pose':{'xyz':[5.05,0,.65]},'geometry':{'type':'box','size':[.1,20,20]},
                          'motion':{'linear_velocity':[1,0,0],'yaw_rate':0.0}}]
    h.subscribe('dynamic',PointCloud2,'/points_raw',reliable=False)
    h.subscribe('scene',MarkerArray,'/scene',durable=True)
    h.rsp();h.sim(start_paused=True);h.ready()
    assert h.step(20,'moving-scan').success
    h.until(lambda:bool(h.messages['dynamic']))
    for x,y,z,intensity,ring,t in struct.iter_unpack('<ffffH2xf',bytes(h.messages['dynamic'][0].data)):
        assert x == pytest.approx(5-.2,abs=1e-5)
    # Fixed geometry is latched once; advancing time does not animate markers.
    h.until(lambda:bool(h.messages['scene']))
    assert ns(h.messages['scene'][-1].markers[0].header.stamp)==1
    assert h.messages['scene'][-1].markers[0].pose.position.x==pytest.approx(5.05)


def test_EXT06_vendor_bytes_nan_timing_and_actual_pcl_read(h):
    m2_config(h)
    h.subscribe('source',PointCloud2,'/points_raw')
    h.subscribe('vendor',PointCloud2,'/points_ouster')
    h.subscribe('probe',String,'/probe_status',durable=True)
    h.subscribe('adapter',String,'/adapter_status')
    h.process('adapter',[executable('sim_ouster_adapter'),'--ros-args','-p','sensor_reliable:=true'])
    h.process('pcl',[executable('sim_cloud_probe'),'--ros-args','-p','sensor_reliable:=true'])
    h.rsp();h.sim(start_paused=True,sensor_reliable=True);h.ready()
    # Graph discovery alone does not guarantee writer-side DDS matching.
    def pipeline_ready():
        if not h.messages['adapter']: return False
        adapter=json.loads(h.messages['adapter'][-1].data)
        matches=json.loads(h.status().get('subscription_matches','{}'))
        return (adapter['input_publishers']>=1 and adapter['output_subscribers']>=2
                and matches.get('/points_raw',0)>=2)
    h.until(pipeline_ready)
    assert h.step(20,'adapter-first').success
    h.until(lambda:bool(h.messages['source']) and bool(h.messages['vendor']) and bool(h.messages['probe']))
    s,v=h.messages['source'][0],h.messages['vendor'][0]
    assert s.header==v.header and (s.width,s.height)==(v.width,v.height)
    assert v.point_step==48 and v.row_step==v.width*48 and not v.is_bigendian
    assert [(f.name,f.offset,f.datatype,f.count) for f in v.fields]==[
        ('x',0,7,1),('y',4,7,1),('z',8,7,1),('intensity',16,7,1),('t',20,6,1),
        ('reflectivity',24,4,1),('ring',26,4,1),('ambient',28,4,1),('range',32,6,1)]
    finite=0
    for i,p in enumerate(struct.iter_unpack('<ffffH2xf',bytes(s.data))):
        x,y,z,intensity,ring,t=p; b=bytes(v.data[i*48:(i+1)*48])
        assert b[:12]==bytes(s.data[i*24:i*24+12])
        assert struct.unpack_from('<f',b,12)[0]==1.0
        assert struct.unpack_from('<fIH',b,16)==(intensity,math.floor(t*1e9+.5),math.floor(intensity*65535+.5) if math.isfinite(x) else 0)
        assert struct.unpack_from('<HH',b,26)==(ring,0)
        expected=math.floor(math.sqrt(x*x+y*y+z*z)*1000+.5) if math.isfinite(x) else 0
        assert struct.unpack_from('<I',b,32)[0]==expected
        assert b[30:32]+b[36:48]==bytes(14)
        finite+=int(math.isfinite(x))
    result=json.loads(h.messages['probe'][-1].data)
    assert result['algorithm']=='PCL VoxelGrid' and result['fields_ok']
    assert result['input']==s.width*s.height and result['finite']==finite
    assert 0<result['output']<=finite and result['max_ring']==3
    assert result['max_t_ns']==math.floor(31/32*.1*1e9+.5) or abs(result['max_t_ns']-31/32*.1*1e9)<5
    (h.directory/'pcl-result.json').write_text(json.dumps(result,indent=2))
    # Invalid payloads are rejected; a subsequent valid cloud still reaches the real algorithm.
    pub=h.node.create_publisher(PointCloud2,'/points_raw',10)
    h.until(lambda:pub.get_subscription_count()>=2)
    bad=[]
    for attribute,value in [('is_bigendian',True),('point_step',23),('row_step',1),('height',0),('width',0),('data',bytes(3))]:
        m=copy.deepcopy(s);setattr(m,attribute,value);bad.append(m)
    m=copy.deepcopy(s);m.fields[0].offset=1;bad.append(m)
    m=copy.deepcopy(s);m.fields.pop();bad.append(m)
    for offset,value in [(20,-1.0),(20,5.0),(20,float('nan')),(12,2.0),(0,1e9)]:
        m=copy.deepcopy(s);data=bytearray(m.data);struct.pack_into('<f',data,offset,value)
        if offset==0: struct.pack_into('<ff',data,4,0.0,0.0)
        m.data=bytes(data);bad.append(m)
    for i,m in enumerate(bad):
        pub.publish(m);h.until(lambda:json.loads(h.messages['adapter'][-1].data)['rejected']>=i+1)
    pub.publish(s);h.until(lambda:len(h.messages['probe'])>=2)
    assert json.loads(h.messages['adapter'][-1].data)['accepted']==2


def test_EXT06_independent_voxel_centroids_and_invalid_vendor_input(h):
    # The input and result endpoints are discovered independently. Request the
    # retained result so a single cloud cannot outrun status-side discovery.
    h.subscribe('probe',String,'/probe_status',durable=True)
    # This validates exact payloads, including rejection of each invalid input;
    # best-effort delivery may drop the only sample while DDS settles.
    h.process('pcl',[executable('sim_cloud_probe'),'--ros-args','-p','sensor_reliable:=true'])
    pub=h.node.create_publisher(PointCloud2,'/points_ouster',10)
    h.until(lambda:pub.get_subscription_count()==1)
    cloud=PointCloud2();cloud.width=3;cloud.height=1;cloud.point_step=48;cloud.row_step=144
    for name,offset,kind in [('x',0,7),('y',4,7),('z',8,7),('intensity',16,7),('t',20,6),
                             ('reflectivity',24,4),('ring',26,4),('ambient',28,4),('range',32,6)]:
        field=PointField();field.name=name;field.offset=offset;field.datatype=kind;field.count=1;cloud.fields.append(field)
    data=bytearray(144)
    for i,v in enumerate([.01,.03,.21]):
        struct.pack_into('<fffffIHHH2xI',data,i*48,v,v,v,1.0,0.0,i*1000,0,i,0,round(math.sqrt(3)*v*1000))
    cloud.data=bytes(data)
    pub.publish(cloud);h.until(lambda:bool(h.messages['probe']))
    result=json.loads(h.messages['probe'][-1].data)
    assert result['output']==2 and result['fields_ok']
    assert result['first_centroids'][0]==pytest.approx([.02,.02,.02],abs=1e-7)
    assert result['first_centroids'][1]==pytest.approx([.21,.21,.21],abs=1e-7)
    for i,kind in enumerate(['size','field']):
        bad=copy.deepcopy(cloud)
        if kind=='size':bad.data=bytes(1)
        else:bad.fields[0].offset=5
        pub.publish(bad);h.until(lambda:len(h.messages['probe'])>=i+2)
        assert not json.loads(h.messages['probe'][-1].data)['fields_ok']
        assert 'error' in json.loads(h.messages['probe'][-1].data)
