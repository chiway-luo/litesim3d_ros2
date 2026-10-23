#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>
using namespace litesim3d;
TEST(M2Smoke, RandomStreams) {
  EXPECT_EQ(randomNormal(42, 1), randomNormal(42, 1));
  EXPECT_NE(randomNormal(42, 1), randomNormal(42, 2));
}
#include <cstring>
#include <fstream>
#include <numeric>
namespace {
const std::filesystem::path resources2 = SIM_RESOURCES;
float field2(const Cloud &c, size_t slot, size_t field) {
  uint32_t bits = 0;
  for (size_t i = 0; i < 4; ++i) bits |= uint32_t(c.data.at(slot * 24 + field + i)) << (8 * i);
  float value; std::memcpy(&value, &bits, 4); return value;
}
Vec3 point2(const Cloud &c, size_t slot) { return {field2(c,slot,0), field2(c,slot,4), field2(c,slot,8)}; }
Scene makeScene2(std::vector<Geometry> geoms) {
  Scene s; s.visual = s.collision = s.sensor = std::move(geoms); s.rebuild(); return s;
}
Geometry support2(const std::string &id, double angle = 0, double z = 0) {
  Geometry g{id, "", Shape::box({20,20,.2})};
  g.transform = pose(Vec3::Zero(), {0,-angle,0});
  g.transform.translation() = Vec3(0,0,z) - g.transform.linear().col(2) * .1;
  g.support = true; return g;
}
Config cfg2() {
  Scene ignored; auto c = Config::load(resources2 / "config/demo.yaml", ignored);
  c.lidars[0].columns = 64; c.lidars[0].vertical_angles = {0}; c.lidars[0].scan_mode = "rolling";
  c.drive.unlimited_acceleration = true; c.timeout = 1000; return c;
}
Simulator sim2(Config c = cfg2(), Scene s = {}) { return Simulator(c, Model::load(c.urdf), s); }
LidarConfig lidar2() {
  LidarConfig c; c.vertical_angles = {0}; c.columns = 64;
  c.horizontal_start = -.3; c.horizontal_end = .3; c.max_range = 100; c.scan_mode = "rolling";
  return c;
}
Scene wall2() { return makeScene2({{"wall", "", Shape::box({.1,200,200}), pose({10.05,0,0})}}); }
std::vector<Event> step2(Simulator &s, unsigned n) {
  s.pause(true); std::vector<Event> events;
  auto result = s.singleStep(std::to_string(s.stepIndex()), n, events);
  EXPECT_TRUE(result.success) << result.message; return events;
}
}
TEST(GEO10M2, RebuildOnlyOnExplicitUpdate) {
  auto s = wall2(); auto c = lidar2();
  auto before = s.build_count;
  for (int i=0; i<100; ++i) scan(s,c,Pose::Identity(),1);
  EXPECT_EQ(s.build_count,before);
  s.updateEntity("wall",pose({20.05,0,0}));
  EXPECT_EQ(s.build_count,before+1);
  EXPECT_NEAR(s.ray(Vec3::Zero(),Vec3::UnitX(),100)->distance,20,1e-10);
  EXPECT_THROW(s.updateEntity("absent",Pose::Identity()),std::runtime_error);
  EXPECT_EQ(s.build_count,before+1);
}
TEST(GEO12M2, UndersideSensingAndSupportAreDifferent) {
  auto s = makeScene2({support2("ground"),support2("bridge",0,2)});
  EXPECT_EQ(s.supportAt(0,0,0,.02)->entity,"ground");
  EXPECT_EQ(s.supportAt(0,0,2,.02)->entity,"bridge");
  EXPECT_FALSE(s.supportAt(0,0,1.8,.02));
  auto hit = s.ray({0,0,1},Vec3::UnitZ(),10);
  ASSERT_TRUE(hit); EXPECT_EQ(hit->entity,"bridge"); EXPECT_NEAR(hit->distance,.8,1e-10);
}
TEST(MOT08M2, TenDegreeSlopeAndRejectTwentyFive) {
  auto c=cfg2(); c.terrain.enabled=true;
  auto ground=support2("ramp",10*pi/180); auto s=sim2(c,makeScene2({ground}));
  s.command({.5,0}); step2(s,400);
  auto p=s.snapshot().world_base; auto n=ground.transform.linear().col(2);
  EXPECT_NEAR(p.translation().dot(n),0,.002);
  EXPECT_LT(std::acos(std::clamp(p.linear().col(2).dot(n),-1.0,1.0)),1e-3);
  EXPECT_NEAR(p.translation().x(),std::cos(10*pi/180),.001);
  EXPECT_THROW(sim2(c,makeScene2({support2("steep",25*pi/180)})),std::runtime_error);
}
TEST(MOT09M2, BridgeLayerDoesNotSnapToGround) {
  auto c=cfg2(); c.terrain.enabled=true; c.initial_pose=pose({0,0,2});
  auto s=sim2(c,makeScene2({support2("ground"),support2("bridge",0,2)}));
  s.command({.5,0}); step2(s,400);
  EXPECT_NEAR(s.snapshot().world_base.translation().z(),2,1e-10);
  c.initial_pose=Pose::Identity(); auto lower=sim2(c,makeScene2({support2("ground"),support2("bridge",0,2)}));
  lower.command({.5,0}); step2(lower,400);
  EXPECT_NEAR(lower.snapshot().world_base.translation().z(),0,1e-10);
}
TEST(MOT10M2, VoidAndStepStopWithoutFalling) {
  auto c=cfg2(); c.terrain.enabled=true;
  auto g=support2("ground"); g.shape=Shape::box({2,20,.2});
  auto s=sim2(c,makeScene2({g})); s.command({1,0}); step2(s,400);
  // R01 wheel contact lines are at base x=0; support ends at x=1.
  EXPECT_GT(s.snapshot().world_base.translation().x(),.99);
  EXPECT_LE(s.snapshot().world_base.translation().x(),1.0 + 1e-9);
  EXPECT_NEAR(s.snapshot().world_base.translation().z(),0,1e-10);
  EXPECT_NE(s.diagnostics()["last_error"].get<std::string>().find("terrain stop"),std::string::npos);
  auto raised=support2("step",0,.1); raised.shape=Shape::box({2,20,.2}); raised.transform.translation().x()=2;
  auto t=sim2(c,makeScene2({g,raised})); t.command({1,0}); step2(t,400);
  EXPECT_LT(t.snapshot().world_base.translation().x(),1);
}
TEST(LID10M2, RingOffsetsAndLayout) {
  auto c=lidar2(); c.columns=1024; c.vertical_angles.resize(32,0);
  for(size_t r=0;r<32;++r)c.ring_delays.push_back(r * 2e-6);
  auto cloud=scan(wall2(),c,Pose::Identity(),100);
  for(size_t r=0;r<32;++r) for(size_t col=0;col<1024;++col) {
    size_t slot=r*1024+col;
    EXPECT_NEAR(field2(cloud,slot,20), col*.1/1024+r*2e-6,1e-6);
    EXPECT_LT(field2(cloud,slot,20),.1);
    EXPECT_EQ(cloud.data[slot*24+16],r); EXPECT_EQ(cloud.data[slot*24+18],0); EXPECT_EQ(cloud.data[slot*24+19],0);
  }
  EXPECT_LT(field2(cloud,1024,20),field2(cloud,1023,20));
}
TEST(LID11M2, TranslationalRollingIsPhysicalNotFakeTime) {
  auto c=lidar2();
  auto cloud=scanTrajectory(wall2(),c,1,[](uint64_t t){return pose({double(t-1)*1e-9,0,0});});
  for(size_t i=0;i<c.columns;++i) EXPECT_NEAR(point2(cloud,i).x(),10-c.offsetNs(0,i)*1e-9,1e-3);
  EXPECT_GT(point2(cloud,0).x()-point2(cloud,63).x(),.09);
}
TEST(LID12M2, RotationalLeverArmAndIndependentDeskew) {
  auto c=lidar2(); const Pose mount=pose({1,.2,0});
  auto trajectory=[mount](uint64_t t)->Pose {return pose(Vec3::Zero(), {0,0,double(t-1)*1e-9})*mount;};
  auto cloud=scanTrajectory(wall2(),c,1,trajectory);
  const Pose start=trajectory(1);
  for(size_t i=0;i<c.columns;++i) {
    double t=c.offsetNs(0,i)*1e-9, a=c.horizontal_start+(c.horizontal_end-c.horizontal_start)*i/c.columns;
    double origin_x=std::cos(t)-.2*std::sin(t);
    double range=(10-origin_x)/std::cos(t+a);
    EXPECT_LT((point2(cloud,i)-c.direction(0,i)*range).norm(),.001);
    // Independent rigid-body deskew: scan-start coordinates, then intersect the world wall.
    Vec3 corrected=start.inverse()*(trajectory(1+c.offsetNs(0,i))*point2(cloud,i));
    EXPECT_NEAR((start*corrected).x(),10,.001);
  }
}
TEST(LID14M2, NondivisorRatesHaveNoFutureSamples) {
  auto c=cfg2(); c.lidars[0].rate=13; c.step_ns=7000000;
  auto s=sim2(c,wall2()); s.command({.4,.2}); auto events=step2(s,1000);
  size_t frames=0;
  for(const auto&e:events) for(const auto&cloud:e.clouds) {
    EXPECT_EQ(cloud.stamp,1+uint64_t(std::llround(frames*1e9/13)));
    EXPECT_LE(cloud.completion_stamp,e.snapshot.stamp);
    EXPECT_GE(e.snapshot.stamp,cloud.stamp+uint64_t(field2(cloud,63,20)*1e9)); ++frames;
  }
  EXPECT_EQ(frames,91);
}
TEST(LID15M2, GaussianStatisticsFiveSeeds) {
  auto c=lidar2();c.columns=25000;c.vertical_angles={0,0,0,0};c.range_sigma=.01;
  auto s=wall2();
  for(uint64_t seed=0;seed<5;++seed) {
    auto cloud=scanTrajectory(s,c,1,[](uint64_t){return Pose::Identity();},{},seed);
    double sum=0,square=0;
    for(size_t i=0;i<100000;++i){ double e=point2(cloud,i).norm()-10/c.direction(i/c.columns,i%c.columns).x();sum+=e;square+=e*e; }
    double mean=sum/100000, sigma=std::sqrt(square/100000-mean*mean);
    EXPECT_LT(std::abs(mean),5*.01/std::sqrt(100000)); EXPECT_LT(std::abs(sigma/.01-1),.02);
  }
}
TEST(LID16M2, DropoutStatisticsAndSlots) {
  auto c=lidar2();c.columns=25000;c.vertical_angles={0,0,0,0};
  auto s=makeScene2({{"sphere","",Shape::sphere(10),Pose::Identity()}});
  for(double p:{0.0,.1,.5,1.0}) {
    c.dropout=p;auto cloud=scanTrajectory(s,c,1,[](uint64_t){return Pose::Identity();});size_t invalid=0;
    for(size_t i=0;i<100000;++i) if(!point2(cloud,i).allFinite()) {++invalid;EXPECT_EQ(field2(cloud,i,12),0);}
    EXPECT_EQ(cloud.data.size(),2400000);
    if(p==0||p==1) EXPECT_EQ(invalid,size_t(p*100000));
    else EXPECT_LE(std::abs(double(invalid)/100000-p),5*std::sqrt(p*(1-p)/100000));
  }
}
TEST(LID17M2, IntensityAndRangeNoiseNeverClamp) {
  auto c=lidar2(); c.horizontal_start=0;c.horizontal_end=.001;c.columns=1;
  auto s=wall2();s.sensor[0].reflectivity=.7;
  auto good=scan(s,c,Pose::Identity(),1);EXPECT_NEAR(field2(good,0,12),.7,1e-6);
  c.max_range=10.1;c.range_bias=.2;auto high=scan(s,c,Pose::Identity(),1);EXPECT_TRUE(std::isnan(field2(high,0,0)));
  c.min_range=9.9;c.range_bias=-.2;auto low=scan(s,c,Pose::Identity(),1);EXPECT_TRUE(std::isnan(field2(low,0,0)));
  c.range_sigma=-1;EXPECT_THROW(scan(s,c,Pose::Identity(),1),std::runtime_error);
}
TEST(LID19M2, StreamsParallelAndOtherSensorsAreIndependent) {
  auto c=lidar2();c.range_sigma=.02;c.dropout=.1;auto s=wall2();
  for(size_t frame=0;frame<100;++frame) {
    auto a=scanTrajectory(s,c,1,[](uint64_t){return Pose::Identity();},{},42,frame,1);
    auto other=c;other.name="other";scanTrajectory(s,other,1,[](uint64_t){return Pose::Identity();},{},42,frame,2);
    auto b=scanTrajectory(s,c,1,[](uint64_t){return Pose::Identity();},{},42,frame,4);
    EXPECT_EQ(a.data,b.data);
    auto dropped=c;dropped.dropout=0;auto d=scanTrajectory(s,dropped,1,[](uint64_t){return Pose::Identity();},{},42,frame,3);
    for(size_t i=0;i<c.columns;++i)if(point2(a,i).allFinite())EXPECT_EQ(field2(a,i,0),field2(d,i,0));
  }
}
TEST(LID22M2, PausePreservesHalfScanAndFreshRunDiscardsIt) {
  auto c=cfg2(); auto s=sim2(c,wall2());s.command({.5,0});step2(s,10);auto time=s.snapshot().stamp;
  EXPECT_TRUE(s.tick().empty());EXPECT_EQ(s.snapshot().stamp,time);
  auto tail=step2(s,10);ASSERT_FALSE(tail.back().clouds.empty());EXPECT_EQ(tail.back().clouds[0].stamp,1);
  auto fresh=sim2(c,wall2());EXPECT_NE(s.run_id,fresh.run_id);
  auto initial=step2(fresh,20);EXPECT_EQ(initial.back().clouds[0].stamp,1);
  EXPECT_GT(field2(initial.back().clouds[0],32,0),field2(tail.back().clouds[0],32,0));
}
TEST(ODO03M2, ErrorModelDoesNotModifyTruth) {
  auto c=cfg2();c.odom_error.enabled=true;c.odom_error.linear_scale=1.1;c.odom_error.angular_bias=.01;
  auto a=sim2(c); c.odom_error.enabled=false;auto b=sim2(c);
  a.command({.5,0});b.command({.5,0});step2(a,400);step2(b,400);
  EXPECT_TRUE(a.snapshot().world_base.isApprox(b.snapshot().world_base,1e-12));
  EXPECT_NEAR(a.snapshot().world_base.translation().x(),1,1e-9);
  EXPECT_NEAR(yaw(a.snapshot().odom_base),.02,1e-9);
  EXPECT_GT(a.snapshot().odom_base.translation().x(),1.09);
}
TEST(IMU01M2, StaticGravityRotationAndRawCovariance) {
  ImuConfig c;Kinematics k;auto a=sampleImu(c,k,Pose::Identity(),1);
  EXPECT_LT((a.acceleration-Vec3(0,0,9.81)).norm(),1e-10);EXPECT_EQ(a.angular_velocity.norm(),0);
  EXPECT_EQ(a.orientation_covariance[0],-1);EXPECT_EQ(a.acceleration_covariance[0],0);
  auto b=sampleImu(c,k,pose(Vec3::Zero(), {pi/2,0,0}),1);
  EXPECT_LT((b.acceleration-Vec3(0,9.81,0)).norm(),1e-10);
}
TEST(IMU03M2, LeverArmCentripetalTangentialAndGyroMount) {
  ImuConfig c;Kinematics k;k.omega={0,0,1};k.alpha={0,0,2};
  auto a=sampleImu(c,k,pose({1,0,0}),1);
  EXPECT_LT((a.acceleration-Vec3(-1,2,9.81)).norm(),1e-10);
  auto b=sampleImu(c,k,pose({1,0,0},{pi/2,0,0}),1);
  EXPECT_LT((b.angular_velocity-Vec3(0,1,0)).norm(),1e-10);
}
TEST(IMU04M2, AnalyticSmoothTrajectory) {
  ImuConfig c;
  for(size_t n=0;n<1000;++n) {
    double t=n*.005;Kinematics k;
    // p=(sin t, cos 2t, .2 sin 3t), yaw=.4 sin t; analytic first/second derivatives.
    k.world_body=pose({std::sin(t),std::cos(2*t),.2*std::sin(3*t)},{0,0,.4*std::sin(t)});
    k.acceleration={-std::sin(t),-4*std::cos(2*t),-1.8*std::sin(3*t)};
    k.omega={0,0,.4*std::cos(t)};k.alpha={0,0,-.4*std::sin(t)};
    auto a=sampleImu(c,k,Pose::Identity(),1+n*5000000);
    Vec3 expected=k.world_body.linear().transpose()*(k.acceleration+Vec3(0,0,9.81));
    EXPECT_LT((a.acceleration-expected).norm(),1e-3);EXPECT_NEAR(a.angular_velocity.z(),.4*std::cos(t),1e-4);
  }
}
TEST(IMU06M2, BiasNoiseStatisticsAndDeterminism) {
  ImuConfig c;c.accel_sigma=Vec3::Constant(.01);c.gyro_sigma=Vec3::Constant(.001);
  c.accel_bias={.1,.2,.3};c.gyro_bias={.01,.02,.03};Kinematics k;
  Vec3 sum=Vec3::Zero(),square=Vec3::Zero();
  for(uint64_t i=0;i<100000;++i) {
    auto a=sampleImu(c,k,Pose::Identity(),i+1,42,i),b=sampleImu(c,k,Pose::Identity(),i+1,42,i);
    EXPECT_EQ(a.acceleration,b.acceleration);
    auto e=(a.acceleration-Vec3(0,0,9.81)-c.accel_bias).eval();sum+=e;square+=e.cwiseProduct(e);
  }
  for(int j=0;j<3;++j) {
    double mean=sum[j]/100000,sd=std::sqrt(square[j]/100000-mean*mean);
    EXPECT_LT(std::abs(mean),5*.01/std::sqrt(100000));EXPECT_LT(std::abs(sd/.01-1),.02);
  }
  auto a=sampleImu(c,k,Pose::Identity(),1);EXPECT_DOUBLE_EQ(a.acceleration_covariance[0],.0001);
  EXPECT_DOUBLE_EQ(a.angular_velocity_covariance[4],.000001);
}
TEST(IMU07M2, ExactMultiRateSixtySeconds) {
  auto c=cfg2();c.imus.push_back(ImuConfig{});auto s=sim2(c);size_t imu=0,cloud=0,odom=0;
  for(int j=0;j<12;++j)for(const auto&e:step2(s,1000)) {
    for(const auto&i:e.imus){++imu;EXPECT_EQ(i.stamp,1+imu*5000000);EXPECT_LE(i.stamp,e.snapshot.stamp);}
    cloud+=e.clouds.size();odom+=e.odom;
  }
  EXPECT_EQ(imu,12000);EXPECT_EQ(cloud,600);EXPECT_EQ(odom,3000);
}
TEST(IMU08M2, DiscontinuityDoesNotGenerateImpulse) {
  auto c=cfg2();c.imus.push_back(ImuConfig{});auto s=sim2(c);
  s.command({1,2});auto events=step2(s,1);ASSERT_EQ(events.size(),1);
  EXPECT_LT(events[0].imus[0].acceleration.norm(),11);
  EXPECT_GT(s.diagnostics()["discontinuities"].get<uint64_t>(),0);
  EXPECT_THROW(s.setPose(pose({10,0,0})),std::runtime_error);
}
TEST(ROB08M2, HistoriesJournalsAndStepOutputsAreBounded) {
  auto c=cfg2();c.journal_limit=8;c.history_limit=128;auto s=sim2(c);
  s.pause(false);for(int i=0;i<1000;++i){s.command({0,0});s.tick();}
  EXPECT_EQ(s.diagnostics()["retained_inputs"],8);EXPECT_EQ(s.diagnostics()["evicted_inputs"],992);
  EXPECT_LT(s.diagnostics()["trajectory_samples"].get<size_t>(),128);
  c.history_limit=4;EXPECT_THROW(c.validate(),std::runtime_error);
  c=cfg2();c.lidars[0].columns=65536;c.lidars[0].vertical_angles.resize(64,0);
  auto huge=sim2(c);huge.pause(true);std::vector<Event> out;
  EXPECT_FALSE(huge.singleStep("budget",10000,out).success);EXPECT_EQ(huge.snapshot().stamp,1);
}
TEST(ROB05M2, ThousandDeterministicBoundedParserMutations) {
  const auto dir=std::filesystem::temp_directory_path()/
      ("lws-m2-fuzz-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  std::filesystem::create_directories(dir);
  size_t accepted=0,rejected=0;
  const std::string xml=readText(resources2/"urdf/r01.urdf");
  for(size_t i=0;i<1000;++i) {
    const bool valid=randomUniform(173,i)>.5;
    try {
      if(i%3==0) {
        auto file=dir/"candidate.urdf";{std::ofstream out(file);out<<(valid?xml:xml.substr(0,size_t(randomUniform(173,i,1)*xml.size())));}
        Model::load(file);
      } else if(i%3==1) {
        auto text=readText(resources2/"config/demo.yaml");auto pos=text.find("../urdf/r01.urdf");
        text.replace(pos,std::string("../urdf/r01.urdf").size(),(resources2/"urdf/r01.urdf").string());
        if(!valid)text+="\nunknown_"+std::to_string(i)+": true\n";
        auto file=dir/"candidate.yaml";{std::ofstream out(file);out<<text;}
        Scene output;Config::load(file,output);
      } else {
        auto file=dir/"candidate.obj";{std::ofstream out(file);out<<"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 "<<(valid?"3":"99999")<<"\n";}
        Shape::mesh(file,Vec3::Ones());
      }
      ++accepted;EXPECT_TRUE(valid)<<i;
    } catch(const std::exception&) {++rejected;EXPECT_FALSE(valid)<<i;}
  }
  RecordProperty("seed",173);RecordProperty("inputs",1000);RecordProperty("accepted",int(accepted));RecordProperty("rejected",int(rejected));
  EXPECT_EQ(accepted+rejected,1000);EXPECT_GT(accepted,400);EXPECT_GT(rejected,400);
  std::filesystem::remove_all(dir);
}
TEST(ROB06M2, FailedAndCancelledLoadsRetainPreviousScene) {
  auto scene=wall2();const double before=scene.ray(Vec3::Zero(),Vec3::UnitX(),100)->distance;
  for(size_t fail=0;fail<8;++fail) {
    auto control=std::make_shared<LoadControl>();control->fail_allocation_after=fail;
    EXPECT_THROW(Config::load(resources2/"config/demo.yaml",scene,control),std::runtime_error);
    EXPECT_EQ(scene.ray(Vec3::Zero(),Vec3::UnitX(),100)->distance,before);
  }
  auto control=std::make_shared<LoadControl>();control->byte_budget=8;
  EXPECT_THROW(Config::load(resources2/"config/demo.yaml",scene,control),std::runtime_error);
  control=std::make_shared<LoadControl>();control->cancelled=[] {return true;};
  EXPECT_THROW(Config::load(resources2/"config/demo.yaml",scene,control),std::runtime_error);
  auto mesh_control=std::make_shared<LoadControl>();mesh_control->triangle_budget=1;
  EXPECT_THROW(Shape::mesh(resources2/"meshes/cube.stl",Vec3::Ones(),mesh_control),std::runtime_error);
  EXPECT_THROW(Config::load(resources2/"missing.yaml",scene),std::exception);
  EXPECT_EQ(scene.ray(Vec3::Zero(),Vec3::UnitX(),100)->distance,before);
  EXPECT_NO_THROW(Config::load(resources2/"config/demo.yaml",scene));
  EXPECT_NE(scene.ray({0,0,1},Vec3::UnitX(),100)->distance,before);
}

TEST(MOT09M2, ContinuousSmoothRampChangesLayerOnlyThroughConnection) {
  const auto path=std::filesystem::temp_directory_path()/("lws-m2-smooth-ramp-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".obj");
  {
    std::ofstream out(path);out.precision(17);
    for(int i=0;i<=1200;++i) {
      const double x=i*.01,t=x/12,z=2*(6*std::pow(t,5)-15*std::pow(t,4)+10*std::pow(t,3));
      out<<"v "<<x<<" -3 "<<z<<"\nv "<<x<<" 3 "<<z<<"\n";
    }
    for(int i=0;i<1200;++i)out<<"f "<<2*i+1<<' '<<2*i+3<<' '<<2*i+4<<"\nf "<<2*i+1<<' '<<2*i+4<<' '<<2*i+2<<"\n";
  }
  Geometry ramp{"ramp","",Shape::mesh(path,Vec3::Ones()),Pose::Identity(),.5,true};
  std::filesystem::remove(path);
  auto before=support2("before");before.shape=Shape::box({10,6,.2});before.transform.translation().x()=-5;
  auto bridge=support2("bridge",0,2);bridge.shape=Shape::box({10,6,.2});bridge.transform.translation().x()=17;
  auto below=support2("below");below.shape=Shape::box({10,6,.2});below.transform.translation().x()=17;
  auto c=cfg2();c.terrain.enabled=true;c.initial_pose=pose({-1,0,0});
  auto s=sim2(c,makeScene2({before,ramp,bridge,below}));s.command({1,0});
  double last_z=0;
  for(int i=0;i<3000;++i) {
    step2(s,1);double z=s.snapshot().world_base.translation().z();
    ASSERT_LT(std::abs(z-last_z),.002);last_z=z;
  }
  EXPECT_NEAR(s.snapshot().world_base.translation().z(),2,.002);
  EXPECT_GT(s.snapshot().world_base.translation().x(),12);
}
TEST(MOT08M2, SupportMeshDoesNotHideWallWheelContact) {
  auto path=std::filesystem::temp_directory_path()/("lws-m2-wall-floor-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())+".obj");
  {std::ofstream out(path);out<<"v -10 -10 0\nv 10 -10 0\nv 10 10 0\nv -10 10 0\n"
      "v -10 0.28 0\nv 10 0.28 0\nv 10 0.28 1\nv -10 0.28 1\nf 1 2 3\nf 1 3 4\nf 5 6 7\nf 5 7 8\n";}
  Geometry floor{"floor_and_wall","",Shape::mesh(path,Vec3::Ones()),Pose::Identity(),.5,true};
  std::filesystem::remove(path);auto c=cfg2();c.terrain.enabled=true;
  EXPECT_THROW(sim2(c,makeScene2({floor})),std::runtime_error);
}
TEST(M2Validation, SensorLimitsAndWorkerExceptionPropagation) {
  for(int which=0;which<13;++which) {
    auto c=lidar2();
    if(which==0)c.scan_mode="invalid";
    if(which==1)c.range_bias=NAN;
    if(which==2)c.range_sigma=NAN;
    if(which==3)c.range_sigma=-1;
    if(which==4)c.dropout=NAN;
    if(which==5)c.dropout=-.1;
    if(which==6)c.dropout=1.1;
    if(which==7)c.ring_delays={0,0};
    if(which==8)c.ring_delays={-1};
    if(which==9)c.ring_delays={NAN};
    if(which==10)c.ring_delays={.1};
    if(which==11){c.scan_mode="snapshot";c.ring_delays={1e-6};}
    if(which==12){c.columns=1;c.rate=1000;c.ring_delays={.0009999999};}
    EXPECT_THROW(c.validate(),std::runtime_error)<<which;
  }
  auto c=lidar2();auto world=[](uint64_t){return Pose::Identity();};
  EXPECT_THROW(scanTrajectory(wall2(),c,1,world,{},42,0,0),std::runtime_error);
  EXPECT_THROW(scanTrajectory(wall2(),c,1,world,{},42,0,5),std::runtime_error);
  EXPECT_THROW(scanTrajectory(wall2(),c,1,[](uint64_t)->Pose{throw std::runtime_error("injected trajectory failure");},{},42,0,4),std::runtime_error);
  for(int which=0;which<10;++which) {
    ImuConfig i;
    if(which==0)i.name="";if(which==1)i.link="";if(which==2)i.topic="";
    if(which==3)i.rate=0;if(which==4)i.rate=2001;
    if(which==5)i.accel_bias.x()=NAN;if(which==6)i.gyro_bias.y()=NAN;
    if(which==7)i.accel_sigma.x()=-1;if(which==8)i.gyro_sigma.z()=-1;
    if(which==9)i.rate=NAN;
    EXPECT_THROW(i.validate(),std::runtime_error);
  }
  for(int which=0;which<5;++which) {
    Kinematics k;Pose mount=Pose::Identity();
    if(which==0)k.world_body.translation().x()=NAN;
    if(which==1)k.acceleration.x()=NAN;if(which==2)k.omega.y()=NAN;
    if(which==3)k.alpha.z()=NAN;if(which==4)mount.translation().z()=NAN;
    EXPECT_THROW(sampleImu(ImuConfig{},k,mount,1),std::runtime_error);
  }
  OdomError e;e.linear_scale=0;EXPECT_THROW(e.validate(),std::runtime_error);
  e=OdomError{};e.angular_scale=0;EXPECT_THROW(e.validate(),std::runtime_error);
  e=OdomError{};e.linear_bias=NAN;EXPECT_THROW(e.validate(),std::runtime_error);
  e=OdomError{};e.linear_sigma=-1;EXPECT_THROW(e.validate(),std::runtime_error);
  e=OdomError{};e.angular_sigma=-1;EXPECT_THROW(e.validate(),std::runtime_error);
}
TEST(M2Validation, SupportQueriesAndTransactionalEntityChanges) {
  auto s=makeScene2({support2("floor")});
  for(int i=0;i<4;++i) {
    double x=0,y=0,z=0,delta=.02;
    if(i==0)x=NAN;if(i==1)y=NAN;if(i==2)z=NAN;if(i==3)delta=-1;
    EXPECT_THROW(s.supportAt(x,y,z,delta),std::runtime_error);
  }
  EXPECT_FALSE(s.supportAt(100,0,0,.02));EXPECT_FALSE(s.supportAt(0,100,0,.02));
  EXPECT_FALSE(s.supportAt(0,0,2,.02));EXPECT_FALSE(s.supportAt(0,0,0,.02,"unknown"));
  Pose bad=Pose::Identity();bad.translation().x()=NAN;EXPECT_THROW(s.updateEntity("floor",bad),std::runtime_error);
  bad=Pose::Identity();bad.linear()(0,1)=1;EXPECT_THROW(s.updateEntity("floor",bad),std::runtime_error);
  bad=Pose::Identity();bad.linear()(2,2)=-1;EXPECT_THROW(s.updateEntity("floor",bad),std::runtime_error);
  auto floor=support2("inverted");floor.transform=pose(Vec3::Zero(),{pi,0,0});
  EXPECT_FALSE(makeScene2({floor}).supportAt(0,0,0,1));
  floor.transform=pose(Vec3::Zero(),{pi/2,0,0});EXPECT_FALSE(makeScene2({floor}).supportAt(0,0,0,1));
  floor.shape=Shape::sphere(1);floor.transform=Pose::Identity();EXPECT_FALSE(makeScene2({floor}).supportAt(0,0,0,1));
  for(int i=0;i<7;++i) {
    TerrainConfig c;
    if(i==0)c.max_slope=0;if(i==1)c.max_slope=pi/2;if(i==2)c.max_slope=NAN;
    if(i==3)c.clearance=-1;if(i==4)c.clearance=NAN;if(i==5)c.continuity=0;if(i==6)c.continuity=.2;
    EXPECT_THROW(c.validate(),std::runtime_error);
  }
}

TEST(MOT08M2, TerrainAttitudeSweepCannotTunnelLidarThroughThinWall) {
  auto make_ramp=[](const char *id,double pitch,double offset) {
    Geometry g{id,"",Shape::box({1,6,.2}),pose(Vec3::Zero(),{0,pitch,0}),.5,true};
    g.transform.translation()=g.transform.linear()*Vec3(offset,0,-.1);return g;
  };
  auto left=make_ramp("left",-2*pi/180,-.5),right=make_ramp("right",2*pi/180,.5);
  Geometry wall{"thin_wall","",Shape::box({.001,3,.02}),pose({.2,0,2})};
  auto c=cfg2();c.terrain.enabled=true;c.initial_pose=pose({-.003,0,-.003*std::tan(2*pi/180)});
  auto model=Model::load(c.urdf);model.joints.at("lidar_fixed").origin=pose({.2,0,2});
  Simulator s(c,model,makeScene2({left,right,wall}));s.command({1,0});step2(s,1);
  EXPECT_LT(s.snapshot().world_base.translation().x(),0);
  EXPECT_EQ(s.snapshot().v,0);
  EXPECT_FALSE(s.diagnostics()["last_error"].get<std::string>().empty());
}


#include "litesim3d/sensor_plugin.hpp"
TEST(ImuGravityParameter, CustomMagnitudeRotationAndAcceleration) {
  ImuConfig c; c.gravity=9.7; Kinematics k; k.acceleration.z()=1.;
  EXPECT_NEAR(sampleImu(c,k,Pose::Identity(),1).acceleration.z(),10.7,1e-12);
  const auto tilted=sampleImu(c,k,pose(Vec3::Zero(),{pi/2,0,0}),1);
  EXPECT_LT((tilted.acceleration-Vec3(0,10.7,0)).norm(),1e-12);
}
TEST(ImuGravityParameter, DefaultsSerializationAndValidation) {
  using namespace litesim3d::plugins;
  EXPECT_DOUBLE_EQ(sensorDefaults("imu").at("gravity_mps2").get<double>(),9.81);
  auto c=imuSettings("test",{{"link","imu_link"},{"topic","imu"},{"gravity_mps2",1.62}});
  EXPECT_DOUBLE_EQ(c.gravity,1.62);
  EXPECT_DOUBLE_EQ(sensorParameters(c).at("gravity_mps2").get<double>(),1.62);
  EXPECT_DOUBLE_EQ(sampleImu(c,Kinematics{},Pose::Identity(),1).acceleration.z(),1.62);
  for(double bad:{0.,-1.,std::numeric_limits<double>::infinity(),std::numeric_limits<double>::quiet_NaN()}) {
    c.gravity=bad; EXPECT_THROW(c.validate(),std::runtime_error);
  }
  EXPECT_THROW(imuSettings("bad",{{"link","imu_link"},{"topic","imu"},{"gravity_mps2",-1.}}),std::runtime_error);
}
