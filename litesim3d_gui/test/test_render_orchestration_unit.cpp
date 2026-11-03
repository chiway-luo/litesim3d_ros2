// Production RenderScene/CameraRenderer policy with a recording renderer boundary.
#include "../src/camera_renderer.hpp"
#include <gtest/gtest.h>
#include <QTemporaryDir>
namespace {
using namespace litesim3d_render;
namespace backend=ignition::rendering;
class RenderUnits:public testing::Test {
protected:
  void SetUp()override{
    backend::unit::reset();previous=qgetenv("IGN_RENDERING_RESOURCE_PATH");had=qEnvironmentVariableIsSet("IGN_RENDERING_RESOURCE_PATH");
    ASSERT_TRUE(temp.isValid());auto root=temp.path()+"/resources";
    ASSERT_TRUE(QDir().mkpath(root+"/ogre2/media/materials/scripts"));
    QFile file(root+"/ogre2/media/materials/scripts/depth_camera.material");ASSERT_TRUE(file.open(QIODevice::WriteOnly));file.write("unit material");file.close();
    qputenv("IGN_RENDERING_RESOURCE_PATH",root.toUtf8());
  }
  void TearDown()override{
    backend::test_engine.current.reset();if(had)qputenv("IGN_RENDERING_RESOURCE_PATH",previous);else qunsetenv("IGN_RENDERING_RESOURCE_PATH");
  }
  J base(){return {{"schema_version",1},{"resources",J::object()},{"lights",J::array()},{"visuals",J::array()},{"initial_link_poses",J::object()}};}
  J material(){return {{"transparency",.25},{"cast_shadows",true}};}
  J visual(J geometry,std::string binding=""){
    return {{"id","shape"},{"geometry",geometry},{"material",material()},{"binding",binding},{"pose",J::array({1,2,3,0,0,0,1})}};
  }
  J camera(std::string name,std::string type){
    return {{"name",name},{"type",type},{"enabled",true},{"width",4},{"height",1},{"horizontal_fov_deg",90},
            {"near_clip_m",1},{"far_clip_m",5},{"link","optical"}};
  }
  std::string image(){
    auto path=temp.filePath("texture.png");QImage im(2,2,QImage::Format_RGB888);im.fill(Qt::red);EXPECT_TRUE(im.save(path));return path.toStdString();
  }
  std::string mesh(){
    auto path=temp.filePath("triangle.obj").toStdString();
    std::ofstream(path)<<"v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n";return path;
  }
  QTemporaryDir temp;QByteArray previous;bool had=false;
};
TEST_F(RenderUnits,DefaultLightAndReleaseOwnExactlyOneScene){
  {RenderScene scene(base());auto s=scene.scene();ASSERT_EQ(s->lights.size(),1u);EXPECT_EQ(s->lights[0]->type,3);
   EXPECT_EQ(s->lights[0]->direction,math::Vector3d(-.6,-.4,-1));EXPECT_TRUE(scene.modelPoses().empty());}
  EXPECT_EQ(backend::unit::destroyed,1);EXPECT_EQ(backend::unit::unloaded,1);
}
TEST_F(RenderUnits,BackendFailuresReleaseOnlyAcquiredResources){
  backend::unit::fail_engine=true;EXPECT_THROW(RenderScene scene(base()),std::runtime_error);EXPECT_EQ(backend::unit::unloaded,0);
  backend::unit::fail_engine=false;backend::unit::fail_scene=true;
  EXPECT_THROW(RenderScene scene(base()),std::runtime_error);EXPECT_EQ(backend::unit::destroyed,0);EXPECT_EQ(backend::unit::unloaded,1);
  backend::unit::fail_scene=false;auto description=base();description.erase("visuals");
  EXPECT_THROW(RenderScene scene(description),J::out_of_range);EXPECT_EQ(backend::unit::destroyed,1);EXPECT_EQ(backend::unit::unloaded,2);
}
TEST_F(RenderUnits,EachLightKindForwardsPoseColorAndAttenuation){
  auto d=base();for(int type:{1,2,3})d["lights"].push_back({
    {"type",type},{"direction",J::array({1,2,3})},{"pose",J::array({4,5,6,0,0,0,1})},
    {"inner",.1},{"outer",.2},{"falloff",.3},{"diffuse",J::array({1,0,0,1})},
    {"specular",J::array({0,1,0,1})},{"cast_shadows",true},{"range",10},{"constant",1},{"linear",2},{"quadratic",3}});
  RenderScene scene(d);auto s=scene.scene();ASSERT_EQ(s->lights.size(),3u);
  for(size_t i=0;i<3;++i){auto l=s->lights[i];EXPECT_EQ(l->type,int(i+1));EXPECT_EQ(l->pose.Pos(),math::Vector3d(4,5,6));EXPECT_TRUE(l->shadows);EXPECT_EQ(l->range,10);EXPECT_EQ(l->quadratic,3);}
  EXPECT_DOUBLE_EQ(s->lights[1]->inner,.1);EXPECT_DOUBLE_EQ(s->lights[1]->outer,.2);
}
TEST_F(RenderUnits,PrimitivesApplyScaleAndOnlyBoundPoseUpdates){
  auto d=base();d["visuals"]=J::array({visual({{"type","box"},{"size",J::array({2,3,4})}},"link"),
    visual({{"type","sphere"},{"radius",2}}),visual({{"type","cylinder"},{"radius",3},{"length",4}},"missing")});
  d["initial_link_poses"]={{"link",J::array({10,0,0,0,0,0,1})}};RenderScene scene(d);auto s=scene.scene();
  ASSERT_EQ(s->visuals.size(),3u);EXPECT_EQ(s->visuals[0]->scale,math::Vector3d(2,3,4));
  EXPECT_EQ(s->visuals[1]->scale,math::Vector3d(4,4,4));EXPECT_EQ(s->visuals[2]->scale,math::Vector3d(6,6,4));
  EXPECT_EQ(scene.modelPoses()[0].Pos(),math::Vector3d(11,2,3));
  scene.applyPoses({{"link",J::array({20,0,0,0,0,0,1})}});
  EXPECT_EQ(scene.modelPoses()[0].Pos(),math::Vector3d(21,2,3));EXPECT_EQ(scene.modelPoses()[1].Pos(),math::Vector3d(1,2,3));
}
TEST_F(RenderUnits,AllMaterialSlotsAndTransparencyAreApplied){
  auto d=base();auto v=visual({{"type","box"},{"size",J::array({1,1,1})}});auto texture=image();
  for(const auto&key:{"ambient","diffuse","specular","emissive"})v["material"][key]=J::array({.2,.3,.4,1});
  for(const auto&key:{"albedo_map","normal_map","roughness_map","metalness_map","emissive_map","light_map"})v["material"][key]=texture;
  v["material"].update({{"roughness",.4},{"metalness",.6},{"light_map_uv",2},{"double_sided",true}});
  d["visuals"]=J::array({v});RenderScene scene(d);auto m=scene.scene()->visuals[0]->material;
  EXPECT_DOUBLE_EQ(m->transparency,.25);EXPECT_DOUBLE_EQ(m->roughness,.4);EXPECT_DOUBLE_EQ(m->metalness,.6);
  EXPECT_TRUE(m->cast);EXPECT_TRUE(m->receive);EXPECT_TRUE(m->alpha);EXPECT_TRUE(m->two_sided);
  EXPECT_EQ(m->texture,texture);EXPECT_EQ(m->light,texture);EXPECT_EQ(m->light_uv,2u);
}
TEST_F(RenderUnits,InvalidGeometryMaterialAndTextureReleaseTheScene){
  auto d=base();d["visuals"]=J::array({visual({{"type","unsupported"}})});
  EXPECT_THROW(RenderScene scene(d),std::runtime_error);
  d["visuals"]=J::array({visual({{"type","box"},{"size",J::array({1,1,1})}})});backend::unit::fail_material=true;
  EXPECT_THROW(RenderScene scene(d),std::runtime_error);backend::unit::fail_material=false;
  d["visuals"][0]["material"]["normal_map"]=temp.filePath("missing.png").toStdString();
  EXPECT_THROW(RenderScene scene(d),std::runtime_error);EXPECT_EQ(backend::unit::destroyed,3);
}
TEST_F(RenderUnits,MeshSubmaterialsRespectSourceAvailabilityAndRenderedCount){
  auto d=base();d["visuals"]=J::array({visual({{"type","mesh"},{"uri",mesh()},{"scale",J::array({2,2,2})}})});
  for(bool missing:{false,true}){backend::unit::missing_source=missing;backend::unit::mesh_count=3;
    RenderScene scene(d);auto v=scene.scene()->visuals[0];auto m=std::dynamic_pointer_cast<backend::Mesh>(v->geometry);
    ASSERT_TRUE(m);ASSERT_EQ(m->pieces.size(),3u);for(auto piece:m->pieces){ASSERT_TRUE(piece->material);EXPECT_DOUBLE_EQ(piece->material->transparency,.25);}
  }
  backend::unit::mesh_count=0;EXPECT_THROW(RenderScene scene(d),std::runtime_error);
  d["visuals"][0]["geometry"]["uri"]=temp.filePath("missing.dae").toStdString();
  EXPECT_THROW(RenderScene scene(d),std::runtime_error);
}
TEST_F(RenderUnits,CameraConfigurationAndRgbDepthSerializationAreExact){
  auto d=base();d["cameras"]=J::array({camera("rgb","rgb"),camera("depth","depth"),camera("both","rgbd"),camera("disabled","rgb")});
  d["cameras"][3]["enabled"]=false;CameraRenderer renderer(d);
  auto s=backend::test_engine.current;ASSERT_EQ(s->cameras.size(),4u);
  for(auto c:s->cameras){EXPECT_EQ(c->width,4u);EXPECT_EQ(c->height,1u);EXPECT_EQ(c->aspect,4);EXPECT_NEAR(c->hfov,M_PI/2,1e-12);EXPECT_EQ(c->near,1);EXPECT_EQ(c->far,5);}
  backend::unit::depth_pixels={.5f,1.f,4.f,5.f};std_msgs::msg::Header header;header.stamp.sec=7;
  auto frames=renderer.capture({"rgb","depth","both"},{{"optical",J::array({1,2,3,0,0,0,1})}},header);
  ASSERT_EQ(frames.size(),3u);EXPECT_TRUE(frames[0].depth.data.empty());EXPECT_TRUE(frames[1].color.data.empty());
  EXPECT_EQ(frames[2].color.encoding,"rgb8");EXPECT_EQ(frames[2].color.data,std::vector<uint8_t>(12,42));
  EXPECT_EQ(frames[2].info.header.frame_id,"optical");EXPECT_EQ(frames[2].info.header.stamp.sec,7);
  EXPECT_NEAR(frames[2].info.k[0],2,1e-12);EXPECT_EQ(frames[2].info.k[2],1.5);EXPECT_EQ(frames[2].depth.step,16u);
  float z[4];std::memcpy(z,frames[2].depth.data.data(),16);EXPECT_TRUE(std::isnan(z[0]));EXPECT_EQ(z[1],1);EXPECT_EQ(z[2],4);EXPECT_TRUE(std::isnan(z[3]));
}
TEST_F(RenderUnits,CameraRejectsDuplicatesMissingBackendAndInvalidBatches){
  auto d=base();d["cameras"]=J::array({camera("one","rgb"),camera("one","rgb")});
  EXPECT_THROW(CameraRenderer renderer(d),std::runtime_error);
  d["cameras"]=J::array({camera("one","rgb")});backend::unit::fail_camera=true;
  EXPECT_THROW(CameraRenderer renderer(d),std::runtime_error);backend::unit::fail_camera=false;
  d["cameras"]=J::array({camera("one","depth")});backend::unit::fail_depth=true;
  EXPECT_THROW(CameraRenderer renderer(d),std::runtime_error);backend::unit::fail_depth=false;
  d["cameras"]=J::array({camera("one","rgb"),camera("two","rgb")});CameraRenderer renderer(d);
  J poses={{"optical",J::array({0,0,0,0,0,0,1})}};std_msgs::msg::Header header;
  for(const auto&names:std::vector<std::vector<std::string>>{{},{"one","two","third"},{"missing"},{"one","one"}})
    EXPECT_THROW(renderer.capture(names,poses,header),std::runtime_error);
}
TEST_F(RenderUnits,DepthCallbackRejectsDimensionsAndMissingFrame){
  auto d=base();d["cameras"]=J::array({camera("depth","depth")});CameraRenderer renderer(d);
  J poses={{"optical",J::array({0,0,0,0,0,0,1})}};std_msgs::msg::Header header;
  backend::unit::depth_pixels={std::numeric_limits<float>::infinity(),std::numeric_limits<float>::quiet_NaN(),2,3};
  auto frames=renderer.capture({"depth"},poses,header);float z[4];std::memcpy(z,frames[0].depth.data.data(),16);
  EXPECT_TRUE(std::isnan(z[0]));EXPECT_TRUE(std::isnan(z[1]));EXPECT_EQ(z[2],2);
  backend::unit::depth_width=2;EXPECT_THROW(renderer.capture({"depth"},poses,header),std::runtime_error);
  backend::unit::depth_width=0;backend::unit::depth_height=2;EXPECT_THROW(renderer.capture({"depth"},poses,header),std::runtime_error);
  backend::unit::depth_height=0;backend::unit::depth_channels=4;EXPECT_THROW(renderer.capture({"depth"},poses,header),std::runtime_error);
  backend::unit::depth_channels=1;backend::unit::emit_depth=false;EXPECT_THROW(renderer.capture({"depth"},poses,header),std::runtime_error);
}
}
