// Isolated data transformations only; deliberately no QApplication, ROS node,
// display server, rendering engine, subscriber, or simulator process.
#define LITESIM_CAMERA_VIEW_TEST
#include "../src/camera_view.cpp"
#include "../src/render_scene.hpp"
#include <gtest/gtest.h>
#include <QTemporaryDir>
#include <limits>

namespace {
using sensor_msgs::msg::Image;
using namespace litesim3d_render;

Image rgb() {
  Image image;
  image.width=2; image.height=2; image.step=8; image.encoding="rgb8";
  image.data={255,0,0,0,255,0,99,98,0,0,255,7,8,9,97,96};
  return image;
}
Image depth(const std::vector<float> &values,bool big) {
  Image image;
  image.width=values.size(); image.height=1; image.step=image.width*4+4;
  image.encoding="32FC1"; image.is_bigendian=big; image.data.resize(image.step,0xaa);
  for(size_t i=0;i<values.size();++i) {
    uint32_t bits; std::memcpy(&bits,&values[i],4);
    for(unsigned k=0;k<4;++k) image.data[i*4+k]=uint8_t(bits>>(8*(big?3-k:k)));
  }
  return image;
}

TEST(CameraPixels, RgbKeepsPaddedRowsAndOwnsItsStorage) {
  auto source=rgb(); auto result=CameraView::decode(source,false,.1,10);
  EXPECT_EQ(result.size(),QSize(2,2));
  EXPECT_EQ(result.pixelColor(0,0),QColor(255,0,0));
  EXPECT_EQ(result.pixelColor(1,0),QColor(0,255,0));
  EXPECT_EQ(result.pixelColor(0,1),QColor(0,0,255));
  EXPECT_EQ(result.pixelColor(1,1),QColor(7,8,9));
  source.data.assign(source.data.size(),0);
  EXPECT_EQ(result.pixelColor(0,0),QColor(255,0,0));
}

TEST(CameraPixels, RejectsEveryMalformedDimensionLayoutAndEncoding) {
  auto expect_bad=[](Image image) {
    EXPECT_THROW(CameraView::decode(image,false,.1,10),std::runtime_error);
  };
  auto image=rgb(); image.width=0; expect_bad(image);
  image=rgb(); image.height=0; expect_bad(image);
  image=rgb(); image.width=4097; expect_bad(image);
  image=rgb(); image.height=4097; expect_bad(image);
  image=rgb(); image.step=5; expect_bad(image);
  image=rgb(); image.data.pop_back(); expect_bad(image);
  image=rgb(); image.encoding="bgr8"; expect_bad(image);
  image=rgb(); EXPECT_THROW(CameraView::decode(image,true,.1,10),std::runtime_error);
  auto d=depth({1},false); d.step=3;
  EXPECT_THROW(CameraView::decode(d,true,.1,10),std::runtime_error);
  d=depth({1},false); d.encoding="16UC1";
  EXPECT_THROW(CameraView::decode(d,true,.1,10),std::runtime_error);
}

TEST(CameraPixels, BothEndianDepthPathsClampAndBlackenInvalidMeasurements) {
  const auto nan=std::numeric_limits<float>::quiet_NaN();
  const auto inf=std::numeric_limits<float>::infinity();
  for(bool big:{false,true}) {
    auto source=depth({nan,inf,-inf,0,-1,.5,1,2,3,4},big);
    auto result=CameraView::decode(source,true,1,3);
    EXPECT_EQ(result.format(),QImage::Format_RGB32);
    for(int x=0;x<5;++x) EXPECT_EQ(result.pixelColor(x,0),QColor(0,0,0));
    for(int x:{5,6}) EXPECT_EQ(result.pixelColor(x,0).rgb(),QColor::fromHsvF(0,1,1).rgb());
    EXPECT_EQ(result.pixelColor(7,0).rgb(),QColor::fromHsvF(.335,1,1).rgb());
    for(int x:{8,9}) EXPECT_EQ(result.pixelColor(x,0).rgb(),QColor::fromHsvF(.67,1,1).rgb());
  }
}

TEST(RenderHelpers, JsonVectorsColorsAndQuaternionOrder) {
  EXPECT_EQ(vec(J::array({1,2,3})),math::Vector3d(1,2,3));
  const auto c=color(J::array({.25,.5,.75,1}));
  EXPECT_FLOAT_EQ(c.R(),.25); EXPECT_FLOAT_EQ(c.G(),.5);
  EXPECT_FLOAT_EQ(c.B(),.75); EXPECT_FLOAT_EQ(c.A(),1);
  const auto p=pose(J::array({1,2,3,0,0,1,0}));
  EXPECT_EQ(p.Pos(),math::Vector3d(1,2,3));
  EXPECT_NEAR(p.Rot().RotateVector(math::Vector3d::UnitX).X(),-1,1e-12);
  EXPECT_THROW(vec(J::array({1,2})),J::out_of_range);
  EXPECT_THROW(color(J::array({1,2,3})),J::out_of_range);
  EXPECT_THROW(pose(J::array({1,2,3,0,0,0})),J::out_of_range);
  EXPECT_NO_THROW(require(true,"not thrown"));
  EXPECT_THROW(require(false,"expected failure"),std::runtime_error);
}

TEST(RenderResources, RejectsSchemaAndMissingOrChangedResourcesBeforeLoadingABackend) {
  EXPECT_THROW(RenderScene(J{{"schema_version",2},{"resources",J::object()}}),std::runtime_error);
  QTemporaryDir directory;
  ASSERT_TRUE(directory.isValid());
  const auto file=directory.filePath("mesh.bin").toStdString();
  J scene={{"schema_version",1},{"resources",{{file,"wrong digest"}}}};
  auto failure=[&]() {
    try { RenderScene invalid(scene); }
    catch(const std::exception &error) { return std::string(error.what()); }
    ADD_FAILURE()<<"invalid scene unexpectedly loaded";
    return std::string();
  };
  EXPECT_NE(failure().find("missing scene resource"),std::string::npos);
  std::ofstream(file)<<"abc";
  EXPECT_NE(failure().find("scene resource changed after import"),std::string::npos);
  // A verified resource still must not bypass the SDK material-resource guard.
  // Select a nonexistent test-local root so no rendering engine is started.
  scene["resources"][file]="ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";
  const bool existed=qEnvironmentVariableIsSet("IGN_RENDERING_RESOURCE_PATH");
  const auto previous=qgetenv("IGN_RENDERING_RESOURCE_PATH");
  qputenv("IGN_RENDERING_RESOURCE_PATH",directory.filePath("absent-renderer").toUtf8());
  const auto message=failure();
  if(existed) qputenv("IGN_RENDERING_RESOURCE_PATH",previous);
  else qunsetenv("IGN_RENDERING_RESOURCE_PATH");
  EXPECT_NE(message.find("Ogre material resources missing"),std::string::npos);
}

std::unique_ptr<ignition::common::SubMesh> triangle() {
  auto sub=std::make_unique<ignition::common::SubMesh>();
  sub->SetPrimitiveType(ignition::common::SubMesh::TRIANGLES);
  sub->AddVertex(0,0,0); sub->AddVertex(1,0,0); sub->AddVertex(0,1,0);
  sub->AddIndex(0); sub->AddIndex(1); sub->AddIndex(2);
  return sub;
}

TEST(RenderNormals, GeneratesMissingAndPartiallyPresentNormals) {
  for(unsigned existing:{0u,1u,3u}) {
    ignition::common::Mesh mesh;
    auto piece=triangle(); auto *sub=piece.get();
    for(unsigned i=0;i<existing;++i) sub->AddNormal(0,0,0);
    mesh.AddSubMesh(std::move(piece)); completeNormals(&mesh);
    ASSERT_EQ(sub->NormalCount(),3u);
    for(unsigned i=0;i<3;++i) EXPECT_EQ(sub->Normal(i),math::Vector3d::UnitZ);
  }
}

TEST(RenderNormals, PreservesValidSuppliedNormalsAndRepairsNonfiniteOnes) {
  ignition::common::Mesh mesh;
  auto piece=triangle(); auto *sub=piece.get();
  for(int i=0;i<3;++i) sub->AddNormal(0,1,0);
  mesh.AddSubMesh(std::move(piece)); completeNormals(&mesh);
  EXPECT_EQ(sub->Normal(1),math::Vector3d::UnitY);
  sub->SetNormal(0,{std::numeric_limits<double>::quiet_NaN(),0,0});
  completeNormals(&mesh);
  for(unsigned i=0;i<3;++i) EXPECT_EQ(sub->Normal(i),math::Vector3d::UnitZ);
}

TEST(RenderNormals, DegenerateFacesUseFiniteFallback) {
  ignition::common::Mesh mesh;
  auto piece=triangle(); auto *sub=piece.get();
  sub->SetVertex(2,{2,0,0}); mesh.AddSubMesh(std::move(piece));
  completeNormals(&mesh);
  for(unsigned i=0;i<3;++i) EXPECT_EQ(sub->Normal(i),math::Vector3d::UnitZ);
  ignition::common::Mesh empty; EXPECT_NO_THROW(completeNormals(&empty));
}

TEST(RenderNormals, RejectsNonTrianglesBadFaceCountsAndOutOfRangeIndices) {
  ignition::common::Mesh lines;
  auto line=triangle(); line->SetPrimitiveType(ignition::common::SubMesh::LINES);
  lines.AddSubMesh(std::move(line)); EXPECT_THROW(completeNormals(&lines),std::runtime_error);
  for(unsigned extra:{1u,2u}) {
    ignition::common::Mesh mesh; auto sub=triangle();
    for(unsigned i=0;i<extra;++i) sub->AddIndex(0);
    mesh.AddSubMesh(std::move(sub)); EXPECT_THROW(completeNormals(&mesh),std::runtime_error);
  }
  for(unsigned index:{0u,1u,2u}) {
    ignition::common::Mesh mesh; auto sub=triangle(); sub->SetIndex(index,3);
    mesh.AddSubMesh(std::move(sub)); EXPECT_THROW(completeNormals(&mesh),std::runtime_error);
  }
}

std::filesystem::path crateMeshes() {
  return std::filesystem::path(__FILE__).parent_path().parent_path().parent_path()/
      "litesim3d_assets/test/fixtures/assets/models/textured_crate/meshes";
}

TEST(RenderMeshImport, ObjAndColladaPreserveIndependentMaterialsUvsAndNormals) {
  RenderScene::MeshCache cache;
  for(const auto *extension:{".obj",".dae"}) {
    const auto path=(crateMeshes()/(std::string("crate")+extension)).string();
    const auto *mesh=RenderScene::loadMesh(path,cache);ASSERT_NE(mesh,nullptr);
    EXPECT_GE(mesh->SubMeshCount(),2u);EXPECT_GE(mesh->MaterialCount(),2u);
    completeNormals(mesh);
    bool textured=false,blue=false;
    for(unsigned i=0;i<mesh->MaterialCount();++i) {
      auto material=mesh->MaterialByIndex(i);ASSERT_NE(material,nullptr);
      textured|=!material->TextureImage().empty();
      const auto color=material->Diffuse();blue|=color.B()>color.R()*2&&color.G()>color.R()*2;
    }
    EXPECT_TRUE(textured);EXPECT_TRUE(blue);
    for(unsigned i=0;i<mesh->SubMeshCount();++i) {
      auto sub=mesh->SubMeshByIndex(i).lock();ASSERT_NE(sub,nullptr);
      EXPECT_GT(sub->VertexCount(),0u);EXPECT_EQ(sub->IndexCount()%3,0u);
      EXPECT_EQ(sub->NormalCount(),sub->VertexCount());EXPECT_GT(sub->TexCoordCount(),0u);
      EXPECT_LT(sub->MaterialIndex(),mesh->MaterialCount());
      for(unsigned n=0;n<sub->NormalCount();++n)EXPECT_TRUE(sub->Normal(n).IsFinite());
    }
    EXPECT_EQ(RenderScene::loadMesh(path,cache),mesh);
  }
  EXPECT_EQ(cache.size(),1u);  // Non-COLLADA ownership remains MeshManager's.
}

TEST(RenderMeshImport, ColladaCacheOwnsItsDecodedMeshAndInvalidFilesFailExplicitly) {
  QTemporaryDir directory;ASSERT_TRUE(directory.isValid());
  const auto copy=std::filesystem::path(directory.path().toStdString())/"crate";
  std::filesystem::copy(crateMeshes().parent_path(),copy,std::filesystem::copy_options::recursive);
  const auto path=copy/"meshes/crate.dae";
  RenderScene::MeshCache cache;
  const auto *first=RenderScene::loadMesh(path.string(),cache);ASSERT_NE(first,nullptr);
  std::ofstream(path)<<"not a COLLADA document";
  EXPECT_EQ(RenderScene::loadMesh(path.string(),cache),first);
  cache.clear();
  EXPECT_THROW(RenderScene::loadMesh(path.string(),cache),std::runtime_error);
  EXPECT_THROW(RenderScene::loadMesh((copy/"missing.dae").string(),cache),std::runtime_error);
  EXPECT_EQ(RenderScene::loadMesh((copy/"missing.unsupported").string(),cache),nullptr);
}

std::string minimalCollada(const char *primitive,const char *indices) {
  return std::string(R"(<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><unit meter="1" name="meter"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries><geometry id="triangle"><mesh>
  <source id="positions"><float_array id="array" count="9">0 0 0 1 0 0 0 1 0</float_array>
  <technique_common><accessor source="#array" count="3" stride="3"><param name="X" type="float"/>
  <param name="Y" type="float"/><param name="Z" type="float"/></accessor></technique_common></source>
  <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices><)")+primitive+
  " count=\"1\"><input semantic=\"VERTEX\" source=\"#vertices\" offset=\"0\"/><p>"+indices+"</p></"+primitive+R"(>
  </mesh></geometry></library_geometries><library_visual_scenes><visual_scene id="scene">
  <node id="node"><instance_geometry url="#triangle"/></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#scene"/></scene></COLLADA>)";
}

TEST(RenderMeshImport, MissingNormalsCanBeCompletedButNonTrianglePrimitivesAreRejected) {
  QTemporaryDir directory;ASSERT_TRUE(directory.isValid());
  const auto path=std::filesystem::path(directory.path().toStdString())/"plain.dae";
  std::ofstream(path)<<minimalCollada("triangles","0 1 2");
  RenderScene::MeshCache cache;
  const auto *mesh=RenderScene::loadMesh(path.string(),cache);ASSERT_NE(mesh,nullptr);
  ASSERT_EQ(mesh->SubMeshCount(),1u);auto sub=mesh->SubMeshByIndex(0).lock();ASSERT_NE(sub,nullptr);
  EXPECT_EQ(sub->NormalCount(),0u);EXPECT_EQ(sub->TexCoordCount(),0u);
  completeNormals(mesh);EXPECT_EQ(sub->NormalCount(),3u);
  for(unsigned i=0;i<3;++i)EXPECT_EQ(sub->Normal(i),math::Vector3d::UnitZ);
  cache.clear();std::ofstream(path)<<minimalCollada("lines","0 1");
  try {RenderScene::loadMesh(path.string(),cache);FAIL()<<"line primitive accepted as triangle";}
  catch(const std::runtime_error &error){EXPECT_NE(std::string(error.what()).find("non-triangle COLLADA primitive"),std::string::npos);}
}
}  // namespace
