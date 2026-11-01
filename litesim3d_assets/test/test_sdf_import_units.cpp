// Importer unit tests deliberately call the adapter's own functions in-process.
// No CLI subprocess, ROS executor, DDS peer or Simulator is used. The graph
// loader and final runtime validation are injected at their module boundaries.
#define main litesimSdfImportMain
#include "../src/sdf_import.cpp"
#undef main
#include <gtest/gtest.h>
#include <assimp/material.h>
#include <chrono>
#include <cstdlib>
#include <memory>

namespace {
class SdfImportUnits : public ::testing::Test {
 protected:
  fs::path dir;
  std::map<std::string, std::optional<std::string>> environment;
  std::vector<std::unique_ptr<sdf::Root>> roots;
  void SetUp() override {
    static unsigned serial = 0;
    dir = fs::temp_directory_path() / ("litesim-import-unit-" +
        std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "-" + std::to_string(serial++));
    fs::create_directories(dir);
    manifest = J::object(); auditedMeshes.clear();
  }
  void TearDown() override {
    for (const auto &[name, value] : environment)
      if (value) setenv(name.c_str(), value->c_str(), 1); else unsetenv(name.c_str());
    std::error_code error; fs::remove_all(dir, error);
    manifest = J::object(); auditedMeshes.clear();
  }
  void env(const std::string &name, const std::optional<std::string> &value) {
    if (!environment.count(name)) {
      const auto *previous = std::getenv(name.c_str());
      environment[name] = previous ? std::optional<std::string>(previous) : std::nullopt;
    }
    if (value) setenv(name.c_str(), value->c_str(), 1); else unsetenv(name.c_str());
  }
  fs::path write(const fs::path &relative, const std::string &text) {
    const auto file = dir / relative; fs::create_directories(file.parent_path());
    std::ofstream(file) << text; return file;
  }
  const sdf::Model &model(const std::string &body, const std::string &name = "robot", const std::string &prefix = "") {
    auto root = std::make_unique<sdf::Root>();
    check(root->LoadSdfString("<sdf version='1.9'><world name='unit'><model name='" + name + "'>" + prefix + body + "</model></world></sdf>"), "unit fixture");
    const auto *value = root->WorldByIndex(0)->ModelByIndex(0);
    roots.push_back(std::move(root)); return *value;
  }
  Importer importer(const std::string &name = "robot") {
    Importer value; value.source = write("source.sdf", "unit-source");
    value.output = dir / ("out" + std::to_string(roots.size())); fs::create_directories(value.output);
    value.robotName = name;
    value.runtime = YAML::Load("robot: {id: " + name + ", base_link: base}\nentity_bindings: {}\nentities: []\n");
    return value;
  }
  static litesim3d::Model fakeModel(const fs::path &, const litesim3d::ModelOptions &options) {
    EXPECT_TRUE(options.allow_visual_only);
    litesim3d::Model value; value.root = "base"; value.links = {"base"}; return value;
  }
  static std::string visual(const std::string &shape = "<box><size>1 2 3</size></box>") {
    return "<visual name='visible'><geometry>" + shape + "</geometry></visual>";
  }
  static std::string collision(const std::string &shape = "<sphere><radius>.4</radius></sphere>") {
    return "<collision name='solid'><geometry>" + shape + "</geometry></collision>";
  }
  static std::string joint(const std::string &name, const std::string &type, const std::string &body, const std::string &extra = "") {
    return "<joint name='" + name + "' type='" + type + "'><parent>base</parent><child>" + body + "</child>" + extra + "</joint>";
  }
  static std::string world(const std::string &contents, const std::string &extra = "") {
    return "<sdf version='1.9'><world name='unit'><physics name='p' type='ignored'><max_step_size>.002</max_step_size><real_time_factor>.7</real_time_factor></physics>" + extra + contents + "</world></sdf>";
  }
  static std::string robotXml() { return "<model name='robot'><link name='base'/></model>"; }
  struct EntryResult { int status; std::string output, error; };
  EntryResult entry(const std::vector<std::string> &args, const std::function<void(Importer &)> &validate = [](Importer &value) {
      value.description["unit_validator"] = true;
    }) {
    std::vector<std::string> values = args; std::vector<char *> argv;
    for (auto &value : values) argv.push_back(value.data());
    testing::internal::CaptureStdout(); testing::internal::CaptureStderr();
    const auto status = importEntry(static_cast<int>(argv.size()), argv.data(), validate, fakeModel);
    auto error = testing::internal::GetCapturedStderr(); auto output = testing::internal::GetCapturedStdout();
    return {status, output, error};
  }
  std::vector<std::string> arguments(const std::string &sdf = "", const std::string &runtime = "") {
    auto source = write("entry.sdf", sdf.empty() ? world(robotXml()) : sdf);
    auto bindings = write("bindings.yaml", runtime.empty() ? "robot: {id: robot, base_link: base}\nsimulation: {}\n" : runtime);
    return {"sdf_import", source.string(), bindings.string(), (dir / "entry-output").string(), "false"};
  }
  void resetOutput() { fs::remove_all(dir / "entry-output"); manifest = J::object(); auditedMeshes.clear(); }
};

void rejects(const std::function<void()> &operation, const std::string &message) {
  try { operation(); FAIL() << "Expected rejection containing: " << message; }
  catch (const std::exception &error) { EXPECT_NE(std::string(error.what()).find(message), std::string::npos) << error.what(); }
}

TEST_F(SdfImportUnits, ResourceResolutionUsesCanonicalUniqueLocalRootsAndAuditsFiles) {
  env("LITESIM_IMPORT_UNIT_PATH", std::nullopt); EXPECT_TRUE(paths("LITESIM_IMPORT_UNIT_PATH").empty());
  env("LITESIM_IMPORT_UNIT_PATH", ":one::two:"); EXPECT_EQ(paths("LITESIM_IMPORT_UNIT_PATH"), (std::vector<fs::path>{"one", "two"}));
  const auto file = write("model-root/item/mesh.obj", "triangle");
  EXPECT_EQ(resource(file.string(), dir), fs::canonical(file));
  EXPECT_EQ(resource("file://" + file.string(), dir), fs::canonical(file));
  EXPECT_EQ(resource("model-root/item/mesh.obj", dir), fs::canonical(file));
  EXPECT_EQ(manifest.at(fs::canonical(file).string()), litesim3d::sha256(file));
  env("GZ_SIM_RESOURCE_PATH", (dir / "absent").string() + ":" + (dir / "model-root").string());
  EXPECT_EQ(resource("model://item/mesh.obj", dir), fs::canonical(file));
  fs::create_directory_symlink(dir / "model-root", dir / "same-root");
  env("GZ_SIM_RESOURCE_PATH", (dir / "model-root").string() + ":" + (dir / "same-root").string());
  EXPECT_EQ(resource("model://item/mesh.obj", dir), fs::canonical(file));
  write("second/item/mesh.obj", "other"); env("GZ_SIM_RESOURCE_PATH", (dir / "model-root").string() + ":" + (dir / "second").string());
  rejects([&] { resource("model://item/mesh.obj", dir); }, "ambiguous resource");
  const auto packaged = write("prefix/share/pkg/image.png", "image");
  env("AMENT_PREFIX_PATH", (dir / "missing").string() + ":" + (dir / "prefix").string());
  EXPECT_EQ(resource("package://pkg/image.png", dir), fs::canonical(packaged));
  EXPECT_EQ(resource("model-root/item", dir, true), fs::canonical(dir / "model-root/item"));
  rejects([&] { resource("model-root/item", dir); }, "expected file");
  rejects([&] { resource("http://remote/asset", dir); }, "only local resources");
  rejects([&] { resource("missing", dir); }, "missing resource");
  env("GZ_SIM_RESOURCE_PATH", std::nullopt); rejects([&] { resource("model://item", dir); }, "missing resource");
  env("AMENT_PREFIX_PATH", std::nullopt); rejects([&] { resource("package://pkg", dir); }, "missing resource");
}

TEST_F(SdfImportUnits, BasicConversionsAndXmlHelpersPreserveCoordinatesAndPrecision) {
  EXPECT_EQ(xyz(gz::math::Vector3d(1, 2, 3)), J::array({1., 2., 3.}));
  EXPECT_EQ(pose(P(1, 2, 3, 0, 0, 0)), J::array({1., 2., 3., 0., 0., 0., 1.}));
  const auto rgba = color(gz::math::Color(.1, .2, .3, .4)); EXPECT_NEAR(rgba[2].get<double>(), .3, 1e-6);
  auto yp = poseYaml(P(1, 2, 3, .2, .3, .4)); EXPECT_DOUBLE_EQ(yp["xyz"][1].as<double>(), 2); EXPECT_NEAR(yp["rpy"][2].as<double>(), .4, 1e-10);
  EXPECT_EQ(triple(gz::math::Vector3d(1, 2, 3)), "1 2 3"); EXPECT_EQ(triple(J::array({1., 2., 3.})), "1 2 3");
  tinyxml2::XMLDocument d; auto *r = child(d, &d, "robot"); origin(d, r, P(1, 2, 3, 0, 0, .2));
  EXPECT_STREQ(r->FirstChildElement("origin")->Attribute("xyz"), "1 2 3");
  for (const auto &g : std::vector<J>{{{"type", "box"}, {"size", {1, 2, 3}}}, {{"type", "sphere"}, {"radius", .4}},
      {{"type", "cylinder"}, {"radius", .5}, {"length", 2.}}, {{"type", "mesh"}, {"uri", "unit.obj"}, {"scale", {2, 3, 4}}}}) {
    auto *n = child(d, r, "visual"); urdfGeometry(d, n, g);
    auto *shape = n->FirstChildElement("geometry")->FirstChildElement(); EXPECT_STREQ(shape->Name(), g["type"].get<std::string>().c_str());
    if (g["type"] == "box") EXPECT_STREQ(shape->Attribute("size"), "1 2 3");
    if (g["type"] == "sphere" || g["type"] == "cylinder") EXPECT_DOUBLE_EQ(shape->DoubleAttribute("radius"), g["radius"].get<double>());
    if (g["type"] == "cylinder") EXPECT_DOUBLE_EQ(shape->DoubleAttribute("length"), 2);
    if (g["type"] == "mesh") { EXPECT_STREQ(shape->Attribute("filename"), "unit.obj"); EXPECT_STREQ(shape->Attribute("scale"), "2 3 4"); }
  }
  urdf::Pose up; up.position.x = 4; up.rotation.setFromRPY(.1, .2, .3); EXPECT_NEAR(urdfPose(up).Rot().Euler().Y(), .2, 1e-10);
  EXPECT_DOUBLE_EQ(urdfPose(up).Pos().X(), 4);
}

TEST_F(SdfImportUnits, AuditAndPoseFailuresAreExplicitAndSourceFilesAreHashed) {
  EXPECT_NO_THROW(check({}, "empty"));
  rejects([] { check({sdf::Error(sdf::ErrorCode::FILE_READ, "first"), sdf::Error(sdf::ErrorCode::FILE_READ, "second")}, "context"); }, "context\nfirst\nsecond");
  EXPECT_EQ(baseOf(nullptr, dir), dir); audit(nullptr);
  auto e = std::make_shared<sdf::Element>(); e->SetName("model"); EXPECT_EQ(baseOf(e, dir), dir);
  e->SetFilePath((dir / "absent.sdf").string()); EXPECT_EQ(baseOf(e, dir), dir); EXPECT_NO_THROW(audit(e));
  const auto source = write("audit/source.sdf", "source"); e->SetFilePath(source.string());
  EXPECT_EQ(baseOf(e, dir), source.parent_path()); audit(e); EXPECT_EQ(manifest.at(source.string()), litesim3d::sha256(source));
  for (const auto &name : {"plugin", "actor", "heightmap", "polyline", "script", "projector", "particle_emitter"}) {
    auto c = std::make_shared<sdf::Element>(); c->SetName(name); e->InsertElement(c);
    rejects([&] { audit(e); }, "unsupported SDF element <" + std::string(name) + ">"); e->RemoveChild(c);
  }
  sdf::Visual unattached; rejects([&] { resolved(unattached.SemanticPose()); }, "SDF pose resolution failed");
  const auto &m = model("<link name='base'><pose>1 2 3 0 0 0</pose></link>");
  EXPECT_EQ(resolved(m.LinkByIndex(0)->SemanticPose()).Pos(), gz::math::Vector3d(1, 2, 3));
}

TEST_F(SdfImportUnits, MeshAuditAcceptsStaticObjAndColladaAndRejectsInvalidFiles) {
  const auto fixtures = fs::path(SDF_TEST_FIXTURES) / "assets/models/textured_crate";
  const auto obj = fixtures / "meshes/crate.obj"; auditMesh(obj); EXPECT_TRUE(auditedMeshes.count(obj.string()));
  EXPECT_TRUE(manifest.contains((fixtures / "meshes/crate.mtl").string()));
  EXPECT_TRUE(manifest.contains((fixtures / "materials/textures/checker.png").string()));
  auto before = manifest; auditMesh(obj); EXPECT_EQ(manifest, before);
  auditMesh(fixtures / "meshes/crate.dae");
  auto bad = write("invalid.obj", "not a mesh"); rejects([&] { auditMesh(bad); }, "invalid mesh");
  auto malformed = write("malformed.dae", "<COLLADA>"); rejects([&] { auditMesh(malformed); }, "invalid COLLADA XML");
  for (const auto &[name, text] : std::vector<std::pair<std::string, std::string>>{
      {"no_asset", "<COLLADA/>"}, {"no_axis", "<COLLADA><asset/></COLLADA>"},
      {"no_text", "<COLLADA><asset><up_axis/></asset></COLLADA>"}, {"wrong_axis", "<COLLADA><asset><up_axis>Y_UP</up_axis></asset></COLLADA>"}}) {
    auto file = write(name + ".dae", text); rejects([&] { auditMesh(file); }, "Z_UP");
  }
  for (const auto &profile : {"profile_GLSL", "profile_CG"}) {
    auto file = write(std::string(profile) + ".dae", "<COLLADA><asset><up_axis>Z_UP</up_axis></asset><effect><" + std::string(profile) + "/></effect></COLLADA>");
    rejects([&] { auditMesh(file); }, "shader profiles");
  }
  auto noLibrary = write("missing_mtl.obj", "mtllib absent.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
  rejects([&] { auditMesh(noLibrary); }, "missing resource");
}

TEST_F(SdfImportUnits, MeshMaterialBoundaryRejectsAnimationSkinEmbeddingAndUnsupportedMaps) {
  aiScene scene; EXPECT_NO_THROW(auditMeshMaterials(scene, dir / "unit.obj"));
  scene.mNumAnimations = 1; rejects([&] { auditMeshMaterials(scene, dir / "unit.obj"); }, "animated meshes"); scene.mNumAnimations = 0;
  scene.mNumMeshes = 1; scene.mMeshes = new aiMesh *[1]{new aiMesh};
  scene.mMeshes[0]->mNumBones = 1; scene.mMeshes[0]->mBones = new aiBone *[1]{new aiBone};
  rejects([&] { auditMeshMaterials(scene, dir / "unit.obj"); }, "skinned meshes");
  delete scene.mMeshes[0]->mBones[0]; delete[] scene.mMeshes[0]->mBones; scene.mMeshes[0]->mBones = nullptr; scene.mMeshes[0]->mNumBones = 0;
  scene.mNumTextures = 1; rejects([&] { auditMeshMaterials(scene, dir / "unit.obj"); }, "embedded mesh textures"); scene.mNumTextures = 0;
  scene.mNumMaterials = 1; scene.mMaterials = new aiMaterial *[1]{new aiMaterial};
  const auto texture = write("diffuse.png", "texture"); aiString name("diffuse.png");
  scene.mMaterials[0]->AddProperty(&name, AI_MATKEY_TEXTURE(aiTextureType_DIFFUSE, 0));
  auditMeshMaterials(scene, dir / "unit.obj"); EXPECT_TRUE(manifest.contains(texture.string()));
  scene.mMaterials[0]->AddProperty(&name, AI_MATKEY_TEXTURE(aiTextureType_NORMALS, 0));
  rejects([&] { auditMeshMaterials(scene, dir / "unit.obj"); }, "non-diffuse texture");
}

TEST_F(SdfImportUnits, GeometryAndRenderGeometryTranslateAllSupportedShapes) {
  for (const auto &[xml, expected] : std::vector<std::pair<std::string, J>>{
      {"<box><size>1 2 3</size></box>", {{"type", "box"}, {"size", {1., 2., 3.}}}},
      {"<sphere><radius>.4</radius></sphere>", {{"type", "sphere"}, {"radius", .4}}},
      {"<cylinder><radius>.2</radius><length>1.3</length></cylinder>", {{"type", "cylinder"}, {"radius", .2}, {"length", 1.3}}}}) {
    const auto &m = model("<link name='base'>" + visual(xml) + "</link>");
    EXPECT_EQ(geometry(*m.LinkByIndex(0)->VisualByIndex(0)->Geom(), dir), expected);
  }
  const auto meshFile = write("mesh.obj", "v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n");
  sdf::Mesh mesh; mesh.SetUri("mesh.obj"); mesh.SetScale({2, 3, 4}); sdf::Geometry sg; sg.SetMeshShape(mesh);
  EXPECT_EQ(geometry(sg, dir), (J{{"type", "mesh"}, {"uri", meshFile.string()}, {"scale", {2., 3., 4.}}}));
  mesh.SetSubmesh("part"); sg.SetMeshShape(mesh); rejects([&] { geometry(sg, dir); }, "submesh selection");
  rejects([&] { geometry(sdf::Geometry{}, dir); }, "unsupported geometry");
  rejects([&] { renderGeometry({}, dir); }, "missing URDF visual geometry");
  auto box = std::make_shared<urdf::Box>(); box->dim = urdf::Vector3(1, 2, 3);
  EXPECT_EQ(renderGeometry(box, dir)["size"], J::array({1., 2., 3.}));
  auto sphere = std::make_shared<urdf::Sphere>(); sphere->radius = .6; EXPECT_DOUBLE_EQ(renderGeometry(sphere, dir)["radius"], .6);
  auto cylinder = std::make_shared<urdf::Cylinder>(); cylinder->radius = .7; cylinder->length = 2; EXPECT_DOUBLE_EQ(renderGeometry(cylinder, dir)["length"], 2);
  auto um = std::make_shared<urdf::Mesh>(); um->filename = meshFile.string(); um->scale = urdf::Vector3(3, 4, 5);
  EXPECT_EQ(renderGeometry(um, dir)["scale"], J::array({3., 4., 5.}));
}

TEST_F(SdfImportUnits, MaterialsRetainMissingMaterialDefaultsAndTranslateMetalMaps) {
  sdf::Visual v; v.SetTransparency(.25); v.SetCastShadows(false);
  EXPECT_EQ(material(v, dir), (J{{"transparency", .25}, {"cast_shadows", false}}));
  sdf::Material m; m.SetAmbient({.1, .2, .3, 1}); m.SetDiffuse({.4, .5, .6, 1}); m.SetDoubleSided(true); v.SetMaterial(m);
  auto basic = material(v, dir); EXPECT_TRUE(basic["double_sided"]); EXPECT_NEAR(basic["diffuse"][0].get<double>(), .4, 1e-6); EXPECT_FALSE(basic.contains("roughness"));
  sdf::Pbr pbr; m.SetPbrMaterial(pbr); v.SetMaterial(m); EXPECT_FALSE(material(v, dir).contains("metalness"));
  sdf::PbrWorkflow workflow; workflow.SetType(sdf::PbrWorkflowType::METAL); workflow.SetRoughness(.3); workflow.SetMetalness(.8);
  pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, workflow); m.SetPbrMaterial(pbr); v.SetMaterial(m);
  EXPECT_FALSE(material(v, dir).contains("albedo_map"));
  workflow.SetAlbedoMap("albedo.png"); workflow.SetNormalMap("normal.png"); workflow.SetRoughnessMap("rough.png");
  workflow.SetMetalnessMap("metal.png"); workflow.SetEmissiveMap("emissive.png"); workflow.SetLightMap("light.png", 2);
  for (const auto &name : {"albedo", "normal", "rough", "metal", "emissive", "light"}) write(std::string(name) + ".png", name);
  pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, workflow); m.SetPbrMaterial(pbr); v.SetMaterial(m); auto maps = material(v, dir);
  EXPECT_DOUBLE_EQ(maps["roughness"], .3); EXPECT_DOUBLE_EQ(maps["metalness"], .8); EXPECT_EQ(maps["light_map_uv"], 2);
  for (const auto &key : {"albedo_map", "normal_map", "roughness_map", "metalness_map", "emissive_map", "light_map"}) EXPECT_TRUE(fs::is_regular_file(maps[key].get<std::string>()));
  workflow.SetAmbientOcclusionMap("ao.png"); pbr.SetWorkflow(sdf::PbrWorkflowType::METAL, workflow); m.SetPbrMaterial(pbr); v.SetMaterial(m);
  rejects([&] { material(v, dir); }, "ambient occlusion");
  sdf::PbrWorkflow specular; specular.SetType(sdf::PbrWorkflowType::SPECULAR); pbr.SetWorkflow(sdf::PbrWorkflowType::SPECULAR, specular); m.SetPbrMaterial(pbr); v.SetMaterial(m);
  rejects([&] { material(v, dir); }, "specular workflow");
}

TEST_F(SdfImportUnits, EntityRoleDefaultsBindingsAndIndicesAreIndependent) {
  auto value = importer(); const J g{{"type", "sphere"}, {"radius", .2}};
  value.entity(g, P(1, 2, 3, 0, 0, 0), "item", "visual", YAML::Node());
  value.entity(g, P::Zero, "item", "collision", YAML::Load("{support: true, reflectivity: .8}"));
  value.entity(g, P::Zero, "item", "sensor_geometry", YAML::Load("{support: true, reflectivity: .3}"));
  value.entity(g, P::Zero, "item", "collision", YAML::Load("{support: false}"));
  auto entries = value.runtime["entities"]; ASSERT_EQ(entries.size(), 4U);
  EXPECT_FALSE(entries[3]["support"].as<bool>());
  EXPECT_EQ(entries[0]["id"].as<std::string>(), "item/visual/0"); EXPECT_DOUBLE_EQ(entries[0]["reflectivity"].as<double>(), .5); EXPECT_FALSE(entries[0]["support"].as<bool>());
  EXPECT_EQ(entries[1]["id"].as<std::string>(), "item/collision/1"); EXPECT_DOUBLE_EQ(entries[1]["reflectivity"].as<double>(), .8); EXPECT_TRUE(entries[1]["support"].as<bool>());
  EXPECT_FALSE(entries[2]["support"].as<bool>()); EXPECT_DOUBLE_EQ(entries[0]["pose"]["xyz"][2].as<double>(), 3);
}

TEST_F(SdfImportUnits, SceneryUsesSemanticFramesCollisionFallbackAndNestedScope) {
  const auto &m = model("<link name='base'><pose>1 0 0 0 0 0</pose>" + visual() + "</link>"
      "<model name='inner'><pose>0 2 0 0 0 0</pose><link name='part'>" + visual("<sphere><radius>.3</radius></sphere>") + collision() + "</link></model>", "scenery", "<pose>4 0 0 0 0 0</pose>");
  auto value = importer(); value.runtime["entity_bindings"]["scenery"]["support"] = true; value.runtime["entity_bindings"]["scenery"]["reflectivity"] = .75;
  value.model(m, P(0, 0, 3, 0, 0, 0), "");
  EXPECT_EQ(value.modelNames, (std::set<std::string>{"scenery", "scenery::inner"}));
  ASSERT_EQ(value.description["visuals"].size(), 2U); EXPECT_EQ(value.description["visuals"][0]["pose"][0], 5.); EXPECT_EQ(value.description["visuals"][0]["pose"][2], 3.);
  EXPECT_EQ(value.description["visuals"][1]["id"], "scenery::inner/part/visible");
  const auto entries = value.runtime["entities"]; ASSERT_EQ(entries.size(), 6U);
  EXPECT_TRUE(entries[2]["support"].as<bool>()); EXPECT_EQ(entries[2]["collision"]["type"].as<std::string>(), "box");
  EXPECT_EQ(entries[5]["collision"]["type"].as<std::string>(), "sphere");
  const auto &fixed = model("<link name='base'/><link name='tip'/>" + joint("fixed", "fixed", "tip"), "fixed");
  auto other = importer(); EXPECT_NO_THROW(other.model(fixed, P::Zero, "")); EXPECT_TRUE(other.runtime["entities"].IsSequence()); EXPECT_EQ(other.runtime["entities"].size(), 0U);
}

TEST_F(SdfImportUnits, SceneryRejectsUnknownBindingsMotionSensorsAndUnsupportedSupport) {
  const auto &simple = model("<link name='base'/>", "item");
  auto value = importer(); value.runtime["entity_bindings"]["item"]["unexpected"] = true;
  rejects([&] { value.model(simple, P::Zero, ""); }, "unknown entity binding field");
  value = importer(); value.runtime["entity_bindings"]["item"]["support"] = true;
  rejects([&] { value.model(simple, P::Zero, ""); }, "support model has no collision");
  value = importer(); value.runtime["entity_bindings"]["item"]["support"] = false; EXPECT_NO_THROW(value.model(simple, P::Zero, ""));
  const auto &sensors = model("<link name='base'><sensor name='native' type='imu'/></link>", "sensed"); value = importer();
  rejects([&] { value.model(sensors, P::Zero, ""); }, "native scenery sensors");
  const auto &articulated = model("<link name='base'/><link name='tip'/>" + joint("moving", "revolute", "tip", "<axis><xyz>0 0 1</xyz></axis>"), "moving"); value = importer();
  rejects([&] { value.model(articulated, P::Zero, ""); }, "articulated scenery");
}

TEST_F(SdfImportUnits, RobotLegacyConversionPreservesJointTypesAxesAuxiliaryFramesAndMimics) {
  const std::string axis = "<axis><xyz>0 1 0</xyz><limit><lower>-1</lower><upper>2</upper><velocity>3</velocity></limit></axis>";
  const auto &m = model("<link name='base'>" + visual() + collision() + "</link><link name='fixed'/><link name='rev'><pose>1 0 0 0 0 0</pose></link><link name='cont'/><link name='slide'/><link name='partial'/>" +
      joint("fixed_j", "fixed", "fixed") + joint("rev_j", "revolute", "rev", "<pose relative_to='base'>0 0 0 0 0 0</pose>" + axis) +
      joint("cont_j", "revolute", "cont", "<axis><xyz>1 0 0</xyz><limit><lower>-1e16</lower><upper>1e16</upper><velocity>0</velocity></limit></axis>") +
      joint("slide_j", "prismatic", "slide", axis) + joint("partial_j", "revolute", "partial", "<axis><xyz>0 0 1</xyz><limit><lower>-1e16</lower><upper>2</upper></limit></axis>"));
  auto value = importer(); value.runtime["mimic_joints"] = YAML::Load("{rev_j: {joint: slide_j}, slide_j: {joint: cont_j, multiplier: -2, offset: .4}}");
  value.model(m, P(2, 3, 4, 0, 0, 0), ""); EXPECT_TRUE(value.foundRobot); ASSERT_EQ(value.description["visuals"].size(), 1U);
  EXPECT_EQ(value.description["visuals"][0]["binding"], "base"); EXPECT_DOUBLE_EQ(value.runtime["robot"]["initial_pose"]["xyz"][0].as<double>(), 2);
  tinyxml2::XMLDocument xml; ASSERT_EQ(xml.LoadFile((value.output / "robot.urdf").c_str()), tinyxml2::XML_SUCCESS);
  auto *robot = xml.FirstChildElement("robot"); ASSERT_NE(robot, nullptr); EXPECT_STREQ(robot->Attribute("name"), "robot");
  std::map<std::string, tinyxml2::XMLElement *> joints;
  for (auto *j = robot->FirstChildElement("joint"); j; j = j->NextSiblingElement("joint")) joints[j->Attribute("name")] = j;
  ASSERT_EQ(joints.size(), 6U); EXPECT_STREQ(joints.at("fixed_j")->Attribute("type"), "fixed");
  EXPECT_STREQ(joints.at("rev_j")->Attribute("type"), "revolute"); EXPECT_STREQ(joints.at("cont_j")->Attribute("type"), "continuous");
  EXPECT_STREQ(joints.at("slide_j")->Attribute("type"), "prismatic"); EXPECT_STREQ(joints.at("partial_j")->Attribute("type"), "revolute");
  EXPECT_STREQ(joints.at("rev_j")->FirstChildElement("child")->Attribute("link"), "__litesim_joint_rev_j");
  EXPECT_STREQ(joints.at("rev_j")->FirstChildElement("axis")->Attribute("xyz"), "0 1 0");
  EXPECT_DOUBLE_EQ(joints.at("rev_j")->FirstChildElement("limit")->DoubleAttribute("velocity"), 3);
  EXPECT_DOUBLE_EQ(joints.at("cont_j")->FirstChildElement("limit")->DoubleAttribute("velocity"), 1e9);
  EXPECT_DOUBLE_EQ(joints.at("rev_j")->FirstChildElement("mimic")->DoubleAttribute("multiplier"), 1);
  EXPECT_DOUBLE_EQ(joints.at("rev_j")->FirstChildElement("mimic")->DoubleAttribute("offset"), 0);
  EXPECT_DOUBLE_EQ(joints.at("slide_j")->FirstChildElement("mimic")->DoubleAttribute("multiplier"), -2);
  EXPECT_DOUBLE_EQ(joints.at("slide_j")->FirstChildElement("mimic")->DoubleAttribute("offset"), .4);
}

TEST_F(SdfImportUnits, RobotLegacyRejectsInvalidArticulationAndOutput) {
  const auto &simple = model("<link name='base'/>"); auto value = importer(); value.foundRobot = true;
  rejects([&] { value.robot(simple, P::Zero, dir); }, "duplicate robot model");
  const auto &nested = model("<link name='base'/><model name='child'><link name='part'/></model>"); value = importer(); rejects([&] { value.robot(nested, P::Zero, dir); }, "nested articulated");
  const auto &fixed = model("<link name='base'/>", "robot", "<static>true</static>"); value = importer(); rejects([&] { value.robot(fixed, P::Zero, dir); }, "must not be static");
  const auto &sensor = model("<link name='base'><sensor name='native' type='imu'/></link>"); value = importer(); rejects([&] { value.robot(sensor, P::Zero, dir); }, "native SDF sensors");
  value = importer(); value.runtime["robot"]["base_link"] = "missing"; rejects([&] { value.robot(simple, P::Zero, dir); }, "base_link not found");
  value = importer(); value.runtime["mimic_joints"]["missing"]["joint"] = "absent"; rejects([&] { value.robot(simple, P::Zero, dir); }, "unknown mimic");
  value = importer(); value.output = dir / "not-created" / "nested"; rejects([&] { value.robot(simple, P::Zero, dir); }, "cannot write generated URDF");
  const auto &ball = model("<link name='base'/><link name='tip'/>" + joint("ball", "ball", "tip")); value = importer(); rejects([&] { value.robot(ball, P::Zero, dir); }, "unsupported joint type");
  const auto &external = model("<link name='base'/><joint name='world_joint' type='fixed'><parent>world</parent><child>base</child></joint>"); value = importer(); rejects([&] { value.robot(external, P::Zero, dir); }, "joint must connect links");
  const auto &reserved = model("<link name='base'/><link name='tip'><pose>1 0 0 0 0 0</pose></link><link name='__litesim_joint_offset'/>" + joint("offset", "fixed", "tip", "<pose relative_to='base'>0 0 0 0 0 0</pose>"));
  value = importer(); rejects([&] { value.robot(reserved, P::Zero, dir); }, "reserved generated joint link");
}

TEST_F(SdfImportUnits, ComposedRobotUsesInjectedGraphValidationAndKeepsUrdfVisualMaterials) {
  const auto &m = model("<link name='base'/>");
  const auto texture = write("paint.png", "pixels");
  auto urdf = write("composed.urdf", "<robot name='robot'><link name='base'><visual><origin xyz='1 2 3'/><geometry><box size='1 2 3'/></geometry><material name='paint'><color rgba='.1 .2 .3 .4'/><texture filename='paint.png'/></material></visual><visual><geometry><sphere radius='.2'/></geometry></visual><visual><geometry><cylinder radius='.2' length='.4'/></geometry><material name='plain'><color rgba='.8 .7 .6 1'/></material></visual></link><link name='tip'/><joint name='ignored' type='prismatic'><parent link='base'/><child link='tip'/><axis xyz='1 0 0'/><limit lower='-1' upper='1' effort='1' velocity='1'/></joint></robot>");
  auto value = importer(); value.runtime["robot"]["urdf"] = urdf.string(); value.runtime["robot"]["initial_joints"]["ignored"] = .5; value.runtime["robot"]["base_link"] = "tip";
  unsigned calls = 0; value.loadModel = [&](const auto &file, const auto &options) { ++calls; EXPECT_EQ(file, urdf); auto graph = fakeModel(file, options); graph.links.insert("tip"); litesim3d::Joint joint; joint.name = "ignored"; joint.type = "prismatic"; joint.parent = "base"; joint.child = "tip"; joint.axis = litesim3d::Vec3::UnitX(); graph.joints[joint.name] = joint; graph.order.push_back(joint.name); return graph; };
  value.robot(m, P(4, 5, 6, 0, 0, 0), dir); EXPECT_EQ(calls, 1U); ASSERT_EQ(value.description["visuals"].size(), 3U);
  auto first = value.description["visuals"][0]; EXPECT_EQ(first["binding"], "base"); EXPECT_EQ(first["id"], "robot/base/visual_0"); EXPECT_EQ(first["pose"][2], 3.);
  EXPECT_EQ(first["material"]["albedo_map"], texture.string()); EXPECT_NEAR(first["material"]["diffuse"][3].get<double>(), .4, 1e-6);
  EXPECT_FALSE(value.description["visuals"][2]["material"].contains("albedo_map"));
  EXPECT_TRUE(value.description["visuals"][2]["material"].contains("diffuse"));
  EXPECT_FALSE(value.description["visuals"][1]["material"].contains("diffuse")); EXPECT_EQ(value.description["robot_model"]["root"], "base");
  EXPECT_EQ(value.description["robot_model"]["sha256"], litesim3d::sha256(urdf)); EXPECT_DOUBLE_EQ(value.runtime["robot"]["initial_pose"]["xyz"][0].as<double>(), 4.5);
  EXPECT_FALSE(fs::exists(value.output / "robot.urdf"));
}

TEST_F(SdfImportUnits, ComposedRobotRejectsMimicBindingsXacroInvalidXmlAndMissingBase) {
  const auto &m = model("<link name='base'/>"); auto value = importer(); value.runtime["robot"]["urdf"] = "robot.urdf";
  value.runtime["mimic_joints"]["any"] = "joint"; rejects([&] { value.robot(m, P::Zero, dir); }, "owns mimic joints");
  value = importer(); value.runtime["robot"]["urdf"] = "robot.xacro"; rejects([&] { value.robot(m, P::Zero, dir); }, "expand Xacro");
  auto invalid = write("invalid.urdf", "<not-a-robot/>"); value = importer(); value.loadModel = fakeModel; value.runtime["robot"]["urdf"] = invalid.string();
  rejects([&] { value.robot(m, P::Zero, dir); }, "invalid composed robot URDF");
  auto valid = write("valid.urdf", "<robot name='unit'><link name='base'/></robot>"); value = importer(); value.loadModel = fakeModel; value.runtime["robot"]["urdf"] = valid.string(); value.runtime["robot"]["base_link"] = "missing";
  rejects([&] { value.robot(m, P::Zero, dir); }, "base_link not found in composed URDF");
}

TEST_F(SdfImportUnits, ValidatedAnnotationHashesPluginFilesAndExpressesInitialLinksInWorld) {
  auto value = importer(); litesim3d::Config cfg; cfg.base_link = "base";
  litesim3d::plugins::Declaration declaration; declaration.parameter_files = {write("plugin.yaml", "scale: 1")}; cfg.plugins.push_back(declaration);
  litesim3d::CameraConfig camera; camera.name = "front"; camera.link = "base"; cfg.cameras.push_back(camera);
  litesim3d::Model graph; graph.root = "root"; graph.links = {"root", "base"}; litesim3d::Joint j; j.name = "offset"; j.type = "fixed"; j.parent = "root"; j.child = "base"; j.origin = litesim3d::pose({1, 0, 0}); graph.joints[j.name] = j; graph.order = {j.name};
  litesim3d::Snapshot snapshot; snapshot.world_base = litesim3d::pose({5, 2, 3}); appendValidatedRuntime(value, cfg, graph, snapshot);
  EXPECT_EQ(manifest.at(declaration.parameter_files[0].string()), litesim3d::sha256(declaration.parameter_files[0]));
  ASSERT_EQ(value.description["cameras"].size(), 1U); EXPECT_EQ(value.description["cameras"][0]["name"], "front");
  EXPECT_EQ(value.description["initial_link_poses"]["root"][0], 4.); EXPECT_EQ(value.description["initial_link_poses"]["base"][0], 5.); EXPECT_EQ(value.description["initial_link_poses"]["base"][2], 3.);
  cfg.plugins.clear(); cfg.cameras.clear(); appendValidatedRuntime(value, cfg, graph, snapshot); EXPECT_TRUE(value.description["cameras"].empty());
}

TEST_F(SdfImportUnits, EntryWritesAuditableAdapterWithTimingSceneCameraLightsAndPatternPaths) {
  const std::string extra = "<scene><ambient>.1 .2 .3 1</ambient><background>.4 .5 .6 1</background></scene><gui><camera name='observer'><pose>1 2 3 0 0 .2</pose></camera></gui>"
      "<light name='sun' type='directional'><direction>0 0 -1</direction></light><light name='point' type='point'/><light name='spot' type='spot'/>";
  const auto pattern = write("pattern.csv", "Time/s,Zenith/deg,Azimuth/deg\n0,0,0\n");
  auto args = arguments(world(robotXml(), extra), "robot: {id: robot, base_link: base}\nsimulation: {}\nlidars: [{pattern_file: pattern.csv}, {name: unpatterned}]\nentity_bindings: {robot: {}}\n"); args[4] = "true"; args.push_back(".9");
  unsigned calls = 0;
  auto result = entry(args, [&](Importer &value) { ++calls; EXPECT_TRUE(fs::exists(value.output / "runtime.yaml")); auto bindings = YAML::LoadFile((value.output / "runtime.yaml").string());
    EXPECT_DOUBLE_EQ(bindings["simulation"]["step_size"].as<double>(), .002); EXPECT_DOUBLE_EQ(bindings["simulation"]["real_time_factor"].as<double>(), .9);
    EXPECT_TRUE(bindings["simulation"]["collision_budget_enabled"].as<bool>()); EXPECT_EQ(bindings["lidars"][0]["pattern_file"].as<std::string>(), pattern.string());
    EXPECT_FALSE(bindings["entity_bindings"]); EXPECT_FALSE(bindings["mimic_joints"]); EXPECT_TRUE(bindings["robot"]["allow_visual_only"].as<bool>());
    value.description["cameras"] = J::array();
  });
  ASSERT_EQ(result.status, 0) << result.error; EXPECT_EQ(calls, 1U); auto report = J::parse(result.output); EXPECT_TRUE(report["collision_budget_enabled"]); EXPECT_EQ(report["visuals"], 0);
  auto output = J::parse(litesim3d::readText(dir / "entry-output/scene.json")); EXPECT_EQ(output["robot_id"], "robot"); EXPECT_TRUE(output["obstacles_fixed"]); EXPECT_EQ(output["source"], args[1]);
  ASSERT_EQ(output["lights"].size(), 3U); EXPECT_EQ(output["lights"][0]["name"], "sun"); EXPECT_EQ(output["view_pose"][0], 1.); EXPECT_NEAR(output["ambient"][2].get<double>(), .3, 1e-6);
  EXPECT_TRUE(output["resources"].contains(args[2])); EXPECT_TRUE(output["resources"].contains(pattern.string()));
  auto expected = output; expected.erase("scene_id"); EXPECT_EQ(output["scene_id"], litesim3d::sha256Text(expected.dump())); EXPECT_EQ(report["scene_id"], output["scene_id"]);
}

TEST_F(SdfImportUnits, EntryResolvesIncludesAndComposedUrdfThroughLocalBoundaries) {
  write("models/included/model.config", "<model><name>included</name><version>1</version><sdf version='1.9'>model.sdf</sdf></model>");
  write("models/included/model.sdf", "<sdf version='1.9'><model name='robot'><link name='base'/></model></sdf>");
  env("GZ_SIM_RESOURCE_PATH", (dir / "models").string());
  write("composed.urdf", "<robot name='robot'><link name='base'/></robot>");
  auto args = arguments(world("<include><uri>model://included</uri></include>", "<gui/>"), "robot: {id: robot, base_link: base, urdf: composed.urdf}\nsimulation: {}\n");
  fs::create_directories(dir / "entry-output");
  auto result = entry(args); ASSERT_EQ(result.status, 0) << result.error;
  const auto generated = YAML::LoadFile((dir / "entry-output/runtime.yaml").string()); EXPECT_DOUBLE_EQ(generated["simulation"]["real_time_factor"].as<double>(), .7);
  EXPECT_FALSE(generated["simulation"]["collision_budget_enabled"].as<bool>()); EXPECT_FALSE(generated["robot"]["allow_visual_only"]);
  auto output = J::parse(litesim3d::readText(dir / "entry-output/scene.json")); EXPECT_FALSE(output.contains("view_pose")); EXPECT_EQ(output["ambient"].size(), 4U); EXPECT_TRUE(output["lights"].empty());
  EXPECT_EQ(output["robot_model"]["source"], (dir / "composed.urdf").string());
}

TEST_F(SdfImportUnits, PublicEntryRejectsUsageWithoutCallingExternalValidation) {
  char executable[] = "sdf_import"; char *argv[] = {executable};
  testing::internal::CaptureStderr();
  EXPECT_EQ(litesimSdfImportMain(1, argv), 1);
  const auto error = testing::internal::GetCapturedStderr();
  EXPECT_NE(error.find("SDF import: usage:"), std::string::npos);
  EXPECT_TRUE(fs::is_empty(dir));
}

TEST_F(SdfImportUnits, EntryRejectsBadArgumentsDuplicateOwnershipUnknownModelsAndValidationErrors) {
  for (auto size : {0U, 4U, 7U}) { std::vector<std::string> args(size, "ignored"); auto result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("usage:"), std::string::npos); }
  auto args = arguments(); args[4] = "yes"; auto result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("collision budget"), std::string::npos);
  args[4] = "false"; write("entry-output/occupied", "not empty"); result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("must be empty"), std::string::npos); resetOutput();
  for (const auto &runtime : {"robot: {id: robot, base_link: base}\nentities: []\n", "robot: {id: robot, base_link: base, initial_pose: {xyz: [0,0,0]}}\n"}) {
    args = arguments("", runtime); result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("must not duplicate SDF geometry"), std::string::npos); resetOutput();
  }
  for (const auto &field : {"step_size: .1", "real_time_factor: 1"}) {
    args = arguments("", "robot: {id: robot, base_link: base}\nsimulation: {" + std::string(field) + "}\n"); result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("duplicate SDF physics"), std::string::npos); resetOutput();
  }
  args = arguments("", "robot: {id: absent, base_link: base}\nsimulation: {}\n"); result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("does not match a world model"), std::string::npos); resetOutput();
  args = arguments("", "robot: {id: robot, base_link: base}\nsimulation: {}\nentity_bindings: {missing: {}}\n"); result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("unknown entity binding"), std::string::npos); resetOutput();
  args = arguments("<sdf version='1.9'><model name='not_world'><link name='base'/></model></sdf>"); result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("exactly one world"), std::string::npos); resetOutput();
  args = arguments("not-xml"); result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("SDF import failed"), std::string::npos); resetOutput();
  args = arguments(); args.push_back("not-a-number"); result = entry(args); EXPECT_EQ(result.status, 1); EXPECT_FALSE(fs::exists(dir / "entry-output/scene.json")); resetOutput();
  args = arguments(); result = entry(args, [](Importer &) { throw std::runtime_error("injected validation rejection"); });
  EXPECT_EQ(result.status, 1); EXPECT_NE(result.error.find("injected validation rejection"), std::string::npos); EXPECT_TRUE(fs::exists(dir / "entry-output/runtime.yaml")); EXPECT_FALSE(fs::exists(dir / "entry-output/scene.json"));
}
} // namespace
