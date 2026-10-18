#include "litesim3d/sim.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace litesim3d;

TEST(CameraConfig, DefaultsToColorWithoutDepth) {
  CameraConfig c;
  EXPECT_EQ(c.type,"rgb");EXPECT_TRUE(c.color());EXPECT_FALSE(c.depth());
  c.type="rgbd";EXPECT_TRUE(c.color());EXPECT_TRUE(c.depth());
  c.type="depth";EXPECT_FALSE(c.color());EXPECT_TRUE(c.depth());
}
TEST(CameraConfig, RejectsInvalidOpticsResolutionAndRates) {
  CameraConfig c;EXPECT_NO_THROW(c.validate());
  for(double rate:{0.,-1.,241.,std::numeric_limits<double>::quiet_NaN()}) {
    c.rate=rate;EXPECT_THROW(c.validate(),std::runtime_error);
  }
  c=CameraConfig{};c.width=0;EXPECT_THROW(c.validate(),std::runtime_error);
  c=CameraConfig{};c.width=4096;c.height=4096;EXPECT_THROW(c.validate(),std::runtime_error);
  c=CameraConfig{};c.far_clip=c.near_clip;EXPECT_THROW(c.validate(),std::runtime_error);
  c=CameraConfig{};c.horizontal_fov=pi;EXPECT_THROW(c.validate(),std::runtime_error);
  c=CameraConfig{};c.topic="/global";EXPECT_THROW(c.validate(),std::runtime_error);
  c=CameraConfig{};c.type="stereo";EXPECT_THROW(c.validate(),std::runtime_error);
}
TEST(CameraConfig, NamesAndActualPublishedTopicsCannotCollide) {
  Scene scene;auto cfg=Config::load(std::filesystem::path(SIM_RESOURCES)/"config/demo.yaml",scene);
  CameraConfig camera;camera.link=cfg.base_link;cfg.cameras={camera};EXPECT_NO_THROW(cfg.validate());
  cfg.lidars[0].topic=camera.topic+"/color/image_raw";EXPECT_THROW(cfg.validate(),std::runtime_error);
  cfg.lidars[0].topic="points_raw";cfg.cameras.push_back(camera);EXPECT_THROW(cfg.validate(),std::runtime_error);
}
TEST(CameraSchedule, ExactIndependentRatesAndPause) {
  Scene scene;auto cfg=Config::load(std::filesystem::path(SIM_RESOURCES)/"config/demo.yaml",scene);
  CameraConfig a;a.link=cfg.base_link;a.rate=30;
  CameraConfig b=a;b.name="rear";b.topic="camera/rear";b.rate=7;
  CameraConfig off=a;off.name="disabled";off.topic="camera/off";off.enabled=false;
  cfg.cameras={a,b,off};cfg.lidars[0].columns=8;
  auto model=Model::load(cfg.urdf,cfg.model_options);Simulator sim(cfg,model,scene);
  sim.pause(true);EXPECT_TRUE(sim.tick().empty());
  sim.pause(false);std::vector<uint64_t> times[3];
  for(unsigned i=0;i<200;++i)for(const auto &event:sim.tick())
    for(auto index:event.cameras)times[index].push_back(event.snapshot.stamp);
  ASSERT_EQ(times[0].size(),30U);ASSERT_EQ(times[1].size(),7U);EXPECT_TRUE(times[2].empty());
  for(size_t channel=0;channel<2;++channel)for(size_t i=0;i<times[channel].size();++i)
    EXPECT_EQ(times[channel][i],1+uint64_t(std::llround((i+1)*1e9/cfg.cameras[channel].rate)));
  const auto before=sim.snapshot().stamp;sim.pause(true);
  EXPECT_TRUE(sim.tick().empty());EXPECT_EQ(sim.snapshot().stamp,before);
}
TEST(CameraBinding, UnknownLinkAndUnsupportedWorldFailExplicitly) {
  Scene scene;auto cfg=Config::load(std::filesystem::path(SIM_RESOURCES)/"config/demo.yaml",scene);
  CameraConfig c;cfg.cameras={c};auto model=Model::load(cfg.urdf,cfg.model_options);
  EXPECT_THROW(Simulator(cfg,model,scene),std::runtime_error);
  cfg.cameras[0].link=cfg.base_link;
  EXPECT_THROW(World(cfg,model,scene,{{"r0",cfg.initial_pose}}),std::runtime_error);
}
