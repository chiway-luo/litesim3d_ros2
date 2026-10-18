#include "litesim3d/sim.hpp"
#include "litesim3d/rate.hpp"
#include <gtest/gtest.h>
#include <fstream>
using namespace litesim3d;

TEST(SdfBudget, DefaultOffAndEnabledChecksCountInstances) {
  Config cfg;Model model;Scene scene;
  EXPECT_FALSE(cfg.collision_budget_enabled);
  auto mesh=std::make_shared<Shape>();mesh->kind=Shape::Kind::Mesh;mesh->triangles.resize(10000);
  for(int i=0;i<21;++i)scene.collision.push_back({"copies","",mesh});
  EXPECT_NO_THROW(validateCollisionBudget(cfg,model,scene));
  cfg.collision_budget_enabled=true;
  EXPECT_THROW(validateCollisionBudget(cfg,model,scene),std::runtime_error);
  scene.collision.resize(20);EXPECT_NO_THROW(validateCollisionBudget(cfg,model,scene));
  model.collision={{"robot","",mesh},{"robot","",mesh},{"robot","",mesh}};
  EXPECT_THROW(validateCollisionBudget(cfg,model,{}),std::runtime_error);
}
TEST(SdfBudget, ShapeAndPairLimitsOnlyApplyWhenEnabled) {
  Config cfg;Model model;Scene scene;auto box=Shape::box(Vec3::Ones());
  model.collision.resize(64,{"robot","",box});scene.collision.resize(313,{"wall","",box});
  EXPECT_NO_THROW(validateCollisionBudget(cfg,model,scene));
  cfg.collision_budget_enabled=true;EXPECT_THROW(validateCollisionBudget(cfg,model,scene),std::runtime_error);
  scene.collision.resize(312);EXPECT_NO_THROW(validateCollisionBudget(cfg,model,scene));
  model.collision.push_back(model.collision.front());EXPECT_THROW(validateCollisionBudget(cfg,model,scene),std::runtime_error);
}
TEST(SdfBudget, VisualMeshDoesNotAllocateFclAndCacheCanUpgrade) {
  const auto file=std::filesystem::temp_directory_path()/"litesim-role-mesh.obj";
  std::ofstream(file)<<"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";
  auto control=std::make_shared<LoadControl>();control->collision_budget_enabled=true;
  auto visual=Shape::mesh(file,Vec3::Ones(),control,false);
  EXPECT_FALSE(visual->collision);EXPECT_DOUBLE_EQ(visual->ray({.2,.2,1},{0,0,-1},2),1);
  auto collision=Shape::mesh(file,Vec3::Ones(),control,true);
  EXPECT_EQ(collision,visual);EXPECT_TRUE(collision->collision);
  std::filesystem::remove(file);
}
TEST(SdfBudget, OffRetainsCollisionDetection) {
  Scene scene;auto cfg=Config::load(std::filesystem::path(SIM_RESOURCES)/"config/demo.yaml",scene);
  auto model=Model::load(cfg.urdf,cfg.model_options);cfg.collision_budget_enabled=false;
  auto parts=model.placed(model.collision,cfg.initial_pose,cfg.base_link,cfg.initial_joints);
  scene.collision.push_back({"blocked_start","",Shape::box({10,10,10})});scene.rebuild();
  EXPECT_THROW(Simulator(cfg,model,scene),std::runtime_error);
}
TEST(SdfGeometry, ColladaKeepsAuthoredZUpAndNodeTransform) {
  auto text=readText(std::filesystem::path(SIM_RESOURCES)/"meshes/cube.dae");
  const auto where=text.find("<node id=\"cube_node\">");
  ASSERT_NE(where,std::string::npos);
  text.insert(where+std::string("<node id=\"cube_node\">").size(),"<translate>1 0 3</translate>");
  const auto file=std::filesystem::temp_directory_path()/"litesim-z-up.dae";
  std::ofstream(file)<<text;
  auto mesh=Shape::mesh(file,Vec3::Ones());
  EXPECT_NEAR(mesh->ray({1,0,10},{0,0,-1},20),6.5,1e-6);
  std::filesystem::remove(file);
}
TEST(SimRate, WallTimeActualRatePauseAndEpoch) {
  RealTimeRate rate;EXPECT_FALSE(rate.sample("a",1,10));
  EXPECT_NEAR(*rate.sample("a",100000001,10.2),.5,1e-10);
  EXPECT_NEAR(*rate.sample("a",500000001,11),.5,1e-10);
  rate.sample("a",500000001,11.2);rate.sample("a",500000001,11.8);
  EXPECT_DOUBLE_EQ(*rate.sample("a",500000001,12.2),0);
  EXPECT_FALSE(rate.sample("b",1,12.4));
  EXPECT_NEAR(*rate.sample("b",400000001,12.6),2,1e-10);
  EXPECT_FALSE(rate.sample("b",1,12.8));
}
