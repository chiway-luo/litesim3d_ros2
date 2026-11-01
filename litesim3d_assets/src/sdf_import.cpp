#include <litesim3d/sim.hpp>
#include <sdf/sdf.hh>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <tinyxml2.h>
#include <yaml-cpp/yaml.h>
#include <urdf_parser/urdf_parser.h>
#include <fstream>
#include <iostream>
#include <sstream>

// The output is an internal, disposable adapter for the existing deterministic
// kernel. SDF owns scenery and spawn poses. An explicit composed URDF owns the
// complete robot; without one, the legacy SDF robot is converted as before.
namespace {
using J = litesim3d::Json;
using P = gz::math::Pose3d;
namespace fs = std::filesystem;
using litesim3d::require;
J manifest = J::object();
std::vector<fs::path> paths(const char *name) {
  std::vector<fs::path> out;
  std::istringstream in(std::getenv(name) ? std::getenv(name) : "");
  for (std::string p; std::getline(in, p, ':');) if (!p.empty()) out.emplace_back(p);
  return out;
}
fs::path resource(const std::string &uri, const fs::path &base, bool directory = false) {
  std::vector<fs::path> candidates;
  if (uri.rfind("model://", 0) == 0) {
    for (auto &p : paths("GZ_SIM_RESOURCE_PATH")) candidates.push_back(p / uri.substr(8));
  } else if (uri.rfind("package://", 0) == 0) {
    for (auto &p : paths("AMENT_PREFIX_PATH")) candidates.push_back(p / "share" / uri.substr(10));
  } else {
    require(uri.find("://") == std::string::npos || uri.rfind("file://", 0) == 0,
            "only local resources are supported: " + uri);
    candidates.push_back(base / (uri.rfind("file://", 0) == 0 ? uri.substr(7) : uri));
  }
  std::set<fs::path> matches;
  for (auto &p : candidates) if (fs::exists(p)) matches.insert(fs::canonical(p));
  require(matches.size()<=1,"ambiguous resource (multiple search roots): "+uri);
  if(!matches.empty()) {
    const auto p=*matches.begin();
    require(directory || fs::is_regular_file(p), "expected file: " + p.string());
    if (fs::is_regular_file(p)) manifest[p.string()] = litesim3d::sha256(p);
    return p;
  }
  throw std::runtime_error("missing resource: " + uri + " (relative to " + base.string() + ")");
}
void check(const sdf::Errors &errors, const std::string &context) {
  if (errors.empty()) return;
  std::string text = context;
  for (auto &e : errors) text += "\n" + e.Message();
  throw std::runtime_error(text);
}
P resolved(const sdf::SemanticPose &s, const std::string &frame = "") {
  P p; check(s.Resolve(p, frame), "SDF pose resolution failed"); return p;
}
J xyz(const gz::math::Vector3d &v) { return J::array({v.X(), v.Y(), v.Z()}); }
J pose(const P &p) { return {p.Pos().X(), p.Pos().Y(), p.Pos().Z(), p.Rot().X(), p.Rot().Y(), p.Rot().Z(), p.Rot().W()}; }
J color(const gz::math::Color &c) { return {c.R(), c.G(), c.B(), c.A()}; }
YAML::Node yaml(const J &j) { return YAML::Load(j.dump()); }
YAML::Node poseYaml(const P &p) { return yaml({{"xyz", xyz(p.Pos())}, {"rpy", xyz(p.Rot().Euler())}}); }
fs::path baseOf(const sdf::ElementPtr &e, const fs::path &fallback) {
  if (e && !e->FilePath().empty() && fs::is_regular_file(e->FilePath()))
    return fs::canonical(e->FilePath()).parent_path();
  return fallback;
}
void audit(const sdf::ElementPtr &e) {
  if (!e) return;
  if (!e->FilePath().empty() && fs::is_regular_file(e->FilePath()))
    manifest[fs::canonical(e->FilePath()).string()] = litesim3d::sha256(e->FilePath());
  // Ignoring these would produce plausible-looking but materially different scenes.
  static const std::set<std::string> unsupported = {"plugin", "actor", "heightmap", "polyline", "script", "projector", "particle_emitter"};
  require(!unsupported.count(e->GetName()), "unsupported SDF element <" + e->GetName() + "> in " + e->FilePath());
  for (auto c = e->GetFirstElement(); c; c = c->GetNextElement()) audit(c);
}
std::set<std::string> auditedMeshes;
void auditMeshMaterials(const aiScene &scene, const fs::path &file) {
  require(scene.mNumAnimations==0,"animated meshes are unsupported: "+file.string());
  for(unsigned i=0;i<scene.mNumMeshes;++i)require(!scene.mMeshes[i]->HasBones(),"skinned meshes are unsupported: "+file.string());
  require(scene.mNumTextures == 0, "embedded mesh textures require export to external texture files: " + file.string());
  for (unsigned i = 0; i < scene.mNumMaterials; ++i)
    for (int t = aiTextureType_DIFFUSE; t <= aiTextureType_UNKNOWN; ++t)
      for (unsigned n = 0; n < scene.mMaterials[i]->GetTextureCount(aiTextureType(t)); ++n) {
        require(t==aiTextureType_DIFFUSE,
          "mesh material uses a non-diffuse texture; declare that map in SDF PBR material: "+file.string());
        aiString name; scene.mMaterials[i]->GetTexture(aiTextureType(t), n, &name);
        resource(name.C_Str(), file.parent_path());
      }
}
void auditMesh(const fs::path &file) {
  if (!auditedMeshes.insert(file.string()).second) return;
  if(file.extension()==".dae") {
    tinyxml2::XMLDocument xml;require(xml.LoadFile(file.c_str())==tinyxml2::XML_SUCCESS,"invalid COLLADA XML");
    auto *asset=xml.RootElement()->FirstChildElement("asset");
    auto *up=asset?asset->FirstChildElement("up_axis"):nullptr;
    require(up&&up->GetText()&&std::string(up->GetText())=="Z_UP",
      "COLLADA must be exported with Z_UP for matching visual/ray/collision coordinates: "+file.string());
    std::function<void(tinyxml2::XMLElement*)> validate=[&](auto *element){
      const std::string name=element->Name();
      require(name!="profile_GLSL"&&name!="profile_CG","COLLADA shader profiles are unsupported: "+file.string());
      for(auto *c=element->FirstChildElement();c;c=c->NextSiblingElement())validate(c);
    };validate(xml.RootElement());
  }
  Assimp::Importer importer;
  const auto *scene = importer.ReadFile(file.string(), aiProcess_Triangulate | aiProcess_ValidateDataStructure);
  require(scene && scene->HasMeshes(), "invalid mesh: " + file.string() + ": " + importer.GetErrorString());
  auditMeshMaterials(*scene,file);
  if (file.extension() == ".obj") {
    std::ifstream stream(file);
    for (std::string line; std::getline(stream, line);) {
      std::istringstream input(line); std::string key, name; input >> key;
      if (key == "mtllib") while (input >> name) resource(name, file.parent_path());
    }
  }
}
J geometry(const sdf::Geometry &g, const fs::path &base) {
  if (auto b = g.BoxShape()) return {{"type", "box"}, {"size", xyz(b->Size())}};
  if (auto s = g.SphereShape()) return {{"type", "sphere"}, {"radius", s->Radius()}};
  if (auto c = g.CylinderShape()) return {{"type", "cylinder"}, {"radius", c->Radius()}, {"length", c->Length()}};
  if (auto m = g.MeshShape()) {
    require(m->Submesh().empty(), "mesh submesh selection is not supported by the ray/collision importer");
    auto file = resource(m->Uri(), baseOf(m->Element(), base));
    auditMesh(file);
    return {{"type", "mesh"}, {"uri", file.string()}, {"scale", xyz(m->Scale())}};
  }
  throw std::runtime_error("unsupported geometry; use box, sphere, cylinder or mesh (export planes to a thin box)");
}
J material(const sdf::Visual &v, const fs::path &base) {
  J out = {{"transparency", v.Transparency()}, {"cast_shadows", v.CastShadows()}};
  const auto *m = v.Material();
  if (!m) return out; // Retain mesh sub-materials, UVs and texture assignments.
  auto mb = baseOf(m->Element(), base);
  out["ambient"] = color(m->Ambient()); out["diffuse"] = color(m->Diffuse());
  out["specular"] = color(m->Specular()); out["emissive"] = color(m->Emissive());
  out["double_sided"] = m->DoubleSided();
  if (auto p = m->PbrMaterial()) {
    require(!p->Workflow(sdf::PbrWorkflowType::SPECULAR), "PBR specular workflow is unsupported; use metal workflow");
    if (auto w = p->Workflow(sdf::PbrWorkflowType::METAL)) {
      require(w->AmbientOcclusionMap().empty(), "ambient occlusion maps are not supported by this rendering backend");
      out["roughness"] = w->Roughness(); out["metalness"] = w->Metalness();
      for (const auto &pair : std::vector<std::pair<std::string, std::string>>{
        {"albedo_map",w->AlbedoMap()}, {"normal_map",w->NormalMap()},
        {"roughness_map",w->RoughnessMap()}, {"metalness_map",w->MetalnessMap()},
        {"emissive_map",w->EmissiveMap()}, {"light_map",w->LightMap()}})
        if (!pair.second.empty()) out[pair.first] = resource(pair.second, mb).string();
      out["light_map_uv"] = w->LightMapTexCoordSet();
    }
  }
  return out;
}
std::string triple(const gz::math::Vector3d &v) {
  std::ostringstream s; s.precision(17); s << v.X() << ' ' << v.Y() << ' ' << v.Z(); return s.str();
}
std::string triple(const J &v) {
  return triple(gz::math::Vector3d(v[0].get<double>(), v[1].get<double>(), v[2].get<double>()));
}
tinyxml2::XMLElement *child(tinyxml2::XMLDocument &d, tinyxml2::XMLNode *p, const char *tag) {
  auto *e = d.NewElement(tag); p->InsertEndChild(e); return e;
}
void origin(tinyxml2::XMLDocument &d, tinyxml2::XMLNode *n, const P &p) {
  auto *o = child(d,n,"origin"); o->SetAttribute("xyz",triple(p.Pos()).c_str()); o->SetAttribute("rpy",triple(p.Rot().Euler()).c_str());
}
void urdfGeometry(tinyxml2::XMLDocument &d, tinyxml2::XMLNode *n, const J &g) {
  const auto type = g.at("type").get<std::string>();
  auto *e = child(d,child(d,n,"geometry"),type.c_str());
  if (type == "box") e->SetAttribute("size",triple(g.at("size")).c_str());
  if (type == "sphere" || type == "cylinder") e->SetAttribute("radius",g.at("radius").get<double>());
  if (type == "cylinder") e->SetAttribute("length",g.at("length").get<double>());
  if (type == "mesh") { e->SetAttribute("filename",g.at("uri").get<std::string>().c_str()); e->SetAttribute("scale",triple(g.at("scale")).c_str()); }
}
P urdfPose(const urdf::Pose &p) {
  return P(gz::math::Vector3d(p.position.x,p.position.y,p.position.z),
           gz::math::Quaterniond(p.rotation.w,p.rotation.x,p.rotation.y,p.rotation.z));
}
J renderGeometry(const urdf::GeometrySharedPtr &g, const fs::path &base) {
  require(bool(g), "missing URDF visual geometry");
  if(g->type==urdf::Geometry::BOX) {
    auto b=std::dynamic_pointer_cast<urdf::Box>(g);
    return {{"type","box"},{"size",{b->dim.x,b->dim.y,b->dim.z}}};
  }
  if(g->type==urdf::Geometry::SPHERE)
    return {{"type","sphere"},{"radius",std::dynamic_pointer_cast<urdf::Sphere>(g)->radius}};
  if(g->type==urdf::Geometry::CYLINDER) {
    auto c=std::dynamic_pointer_cast<urdf::Cylinder>(g);
    return {{"type","cylinder"},{"radius",c->radius},{"length",c->length}};
  }
  require(g->type==urdf::Geometry::MESH,"unsupported URDF visual geometry");
  auto mesh=std::dynamic_pointer_cast<urdf::Mesh>(g);
  auto file=resource(mesh->filename,base);auditMesh(file);
  return {{"type","mesh"},{"uri",file.string()},{"scale",{mesh->scale.x,mesh->scale.y,mesh->scale.z}}};
}
struct Importer {
  // The composed-model graph loader and final simulator validation are external
  // boundaries. Tests replace them while exercising this adapter in-process.
  using ModelLoader = std::function<litesim3d::Model(const fs::path &, const litesim3d::ModelOptions &)>;
  ModelLoader loadModel = litesim3d::Model::load;
  fs::path source, output;
  YAML::Node runtime;
  J description = {{"schema_version",1}, {"visuals",J::array()}, {"lights",J::array()}};
  std::string robotName;
  std::set<std::string> modelNames;
  bool foundRobot = false;
  uint64_t entityIndex = 0;
  void visual(const sdf::Visual &v, const P &parent, const std::string &binding, const std::string &name, const fs::path &base) {
    auto g = geometry(*v.Geom(),base);
    description["visuals"].push_back({{"id",name}, {"binding",binding}, {"pose",pose(parent*resolved(v.SemanticPose()))},
      {"geometry",g}, {"material",material(v,base)}});
  }
  void entity(const J &g, const P &p, const std::string &name, const std::string &role, const YAML::Node &binding) {
    YAML::Node e; e["id"] = name + "/" + role + "/" + std::to_string(entityIndex++);
    e["pose"] = poseYaml(p); e[role] = yaml(g);
    e["reflectivity"] = binding["reflectivity"] ? binding["reflectivity"].as<double>() : .5;
    e["support"] = role == "collision" && binding["support"] && binding["support"].as<bool>();
    runtime["entities"].push_back(e);
  }
  void robot(const sdf::Model &m, const P &world, const fs::path &base) {
    require(!foundRobot, "duplicate robot model"); foundRobot = true;
    require(m.ModelCount() == 0, "nested articulated robot models require flattening before import");
    require(!m.Static(), "driven robot must not be static");
    if(runtime["robot"]["urdf"]) {
      require(!runtime["mimic_joints"],"composed URDF owns mimic joints; remove runtime mimic_joints");
      const fs::path file=runtime["robot"]["urdf"].as<std::string>();
      require(file.extension()!=".xacro","expand Xacro with litesim3d_bringup prepare_config before sdf_import");
      const auto xml=litesim3d::readText(file);
      litesim3d::ModelOptions options;options.allow_visual_only=true;
      auto model=loadModel(file,options); // validate graph before urdfdom traversal
      auto parsed=urdf::parseURDF(xml);require(bool(parsed),"invalid composed robot URDF");
      for(const auto &[name,link]:parsed->links_) {
        size_t index=0;
        for(const auto &v:link->visual_array) {
          J mat={{"transparency",0.0},{"cast_shadows",true}};
          if(v->material) {
            auto &c=v->material->color;
            mat["diffuse"]=mat["ambient"]={c.r,c.g,c.b,c.a};
            if(!v->material->texture_filename.empty())mat["albedo_map"]=resource(v->material->texture_filename,file.parent_path()).string();
          }
          description["visuals"].push_back({{"id",robotName+"/"+name+"/visual_"+std::to_string(index++)},
            {"binding",name},{"pose",pose(urdfPose(v->origin))},{"geometry",renderGeometry(v->geometry,file.parent_path())},{"material",mat}});
        }
      }
      std::map<std::string,double> joints;
      if(auto initial=runtime["robot"]["initial_joints"])for(auto pair:initial)joints[pair.first.as<std::string>()]=pair.second.as<double>();
      const auto frames=model.fk(joints);
      const auto baseLink=runtime["robot"]["base_link"].as<std::string>();
      require(frames.count(baseLink),"robot base_link not found in composed URDF");
      const auto &bp=frames.at(baseLink);Eigen::Quaterniond q(bp.linear());
      P rootBase(gz::math::Vector3d(bp.translation().x(),bp.translation().y(),bp.translation().z()),
                 gz::math::Quaterniond(q.w(),q.x(),q.y(),q.z()));
      runtime["robot"]["initial_pose"]=poseYaml(world*rootBase);
      description["robot_model"]={{"source",file.string()},{"sha256",litesim3d::sha256(file)},{"root",model.root}};
      return;
    }
    tinyxml2::XMLDocument d; auto *r=child(d,&d,"robot"); r->SetAttribute("name",robotName.c_str());
    std::map<std::string,P> links;
    for (uint64_t i=0;i<m.LinkCount();++i) {
      const auto &l=*m.LinkByIndex(i); links[l.Name()]=resolved(l.SemanticPose());
      require(l.SensorCount()==0,"declare LiteSim sensor bindings in runtime.yaml; native SDF sensors are unsupported");
      auto *link=child(d,r,"link");link->SetAttribute("name",l.Name().c_str());
      for(uint64_t v=0;v<l.VisualCount();++v) {
        auto &s=*l.VisualByIndex(v);auto g=geometry(*s.Geom(),base);
        auto *n=child(d,link,"visual");origin(d,n,resolved(s.SemanticPose()));urdfGeometry(d,n,g);
        visual(s,P::Zero,l.Name(),m.Name()+"/"+l.Name()+"/"+s.Name(),base);
      }
      for(uint64_t c=0;c<l.CollisionCount();++c) {
        auto &s=*l.CollisionByIndex(c);auto *n=child(d,link,"collision");
        origin(d,n,resolved(s.SemanticPose()));urdfGeometry(d,n,geometry(*s.Geom(),base));
      }
    }
    const auto baseLink=runtime["robot"]["base_link"].as<std::string>();
    require(links.count(baseLink),"robot base_link not found in SDF");
    runtime["robot"]["initial_pose"]=poseYaml(world*links.at(baseLink));
    std::set<std::string> jointNames;
    for(uint64_t i=0;i<m.JointCount();++i) {
      auto &j=*m.JointByIndex(i);jointNames.insert(j.Name());
      std::string parent,body;check(j.ResolveParentLink(parent),j.Name());check(j.ResolveChildLink(body),j.Name());
      require(links.count(parent)&&links.count(body),"joint must connect links in this robot: "+j.Name());
      require(j.Type()==sdf::JointType::FIXED||j.Type()==sdf::JointType::REVOLUTE||j.Type()==sdf::JointType::PRISMATIC,
        "unsupported joint type: "+j.Name());
      const auto jp=resolved(j.SemanticPose(),"__model__");
      const auto offset=jp.Inverse()*links.at(body);
      const bool auxiliary=offset!=P::Zero;
      auto target=auxiliary?"__litesim_joint_"+j.Name():body;
      if(auxiliary) {
        require(!links.count(target),"reserved generated joint link name: "+target);
        child(d,r,"link")->SetAttribute("name",target.c_str());
        auto *fixed=child(d,r,"joint");fixed->SetAttribute("name",(target+"_fixed").c_str());fixed->SetAttribute("type","fixed");
        child(d,fixed,"parent")->SetAttribute("link",target.c_str());child(d,fixed,"child")->SetAttribute("link",body.c_str());origin(d,fixed,offset);
      }
      auto *joint=child(d,r,"joint");joint->SetAttribute("name",j.Name().c_str());
      std::string type="fixed";
      if(j.Type()!=sdf::JointType::FIXED) {
        const auto *axis=j.Axis();require(axis,"missing joint axis");
        type=j.Type()==sdf::JointType::PRISMATIC?"prismatic":
          (axis->Lower()<=-1e15&&axis->Upper()>=1e15?"continuous":"revolute");
        gz::math::Vector3d v;check(axis->ResolveXyz(v),j.Name());
        child(d,joint,"axis")->SetAttribute("xyz",triple(v).c_str());
        auto *limit=child(d,joint,"limit");limit->SetAttribute("lower",axis->Lower());limit->SetAttribute("upper",axis->Upper());
        limit->SetAttribute("effort",1e9);limit->SetAttribute("velocity",axis->MaxVelocity()>0?axis->MaxVelocity():1e9);
      }
      joint->SetAttribute("type",type.c_str());
      child(d,joint,"parent")->SetAttribute("link",parent.c_str());child(d,joint,"child")->SetAttribute("link",target.c_str());
      origin(d,joint,links.at(parent).Inverse()*jp);
      if(auto mimic=runtime["mimic_joints"][j.Name()]) {
        auto *e=child(d,joint,"mimic");e->SetAttribute("joint",mimic["joint"].as<std::string>().c_str());
        e->SetAttribute("multiplier",mimic["multiplier"]?mimic["multiplier"].as<double>():1.0);
        e->SetAttribute("offset",mimic["offset"]?mimic["offset"].as<double>():0.0);
      }
    }
    if(auto mimics=runtime["mimic_joints"]) for(auto p:mimics)
      require(jointNames.count(p.first.as<std::string>()),"unknown mimic joint binding");
    auto file=output/"robot.urdf"; require(d.SaveFile(file.c_str())==tinyxml2::XML_SUCCESS,"cannot write generated URDF");
    runtime["robot"]["urdf"]=file.string();
  }
  void model(const sdf::Model &m,const P &parent,const std::string &scope) {
    const auto base=baseOf(m.Element(),source.parent_path());
    const auto world=parent*resolved(m.SemanticPose());
    const auto name=scope.empty()?m.Name():scope+"::"+m.Name();
    modelNames.insert(name);
    if(name==robotName) {robot(m,world,base);return;}
    // Every environment model is fixed at its declared initial pose, including
    // SDF models whose <static> flag is absent or false.
    for(uint64_t j=0;j<m.JointCount();++j)
      require(m.JointByIndex(j)->Type()==sdf::JointType::FIXED,"articulated scenery is unsupported: "+name);
    const auto binding=runtime["entity_bindings"][name];
    if(binding)for(auto p:binding)require(p.first.as<std::string>()=="support"||p.first.as<std::string>()=="reflectivity",
      "unknown entity binding field: "+name+"/"+p.first.as<std::string>());
    bool hasCollision=false;
    for(uint64_t i=0;i<m.LinkCount();++i) {
      const auto &l=*m.LinkByIndex(i);const auto lp=world*resolved(l.SemanticPose());
      require(l.SensorCount()==0,"native scenery sensors are unsupported: "+name);
      for(uint64_t v=0;v<l.VisualCount();++v) {
        const auto &s=*l.VisualByIndex(v);visual(s,lp,"",name+"/"+l.Name()+"/"+s.Name(),base);
        entity(geometry(*s.Geom(),base),lp*resolved(s.SemanticPose()),name,"visual",binding);
        entity(geometry(*s.Geom(),base),lp*resolved(s.SemanticPose()),name,"sensor_geometry",binding);
        if(l.CollisionCount()==0) {
          hasCollision=true;
          entity(geometry(*s.Geom(),base),lp*resolved(s.SemanticPose()),name,"collision",binding);
        }
      }
      for(uint64_t c=0;c<l.CollisionCount();++c) {
        hasCollision=true;
        const auto &s=*l.CollisionByIndex(c);
        entity(geometry(*s.Geom(),base),lp*resolved(s.SemanticPose()),name,"collision",binding);
      }
    }
    require(!binding["support"]||!binding["support"].as<bool>()||hasCollision,
      "support model has no collision: "+name);
    for(uint64_t i=0;i<m.ModelCount();++i)model(*m.ModelByIndex(i),world,name);
  }
};
void appendValidatedRuntime(Importer &importer, const litesim3d::Config &cfg,
                            const litesim3d::Model &model, const litesim3d::Snapshot &snapshot) {
  for(const auto &plugin:cfg.plugins)
    for(const auto &file:plugin.parameter_files)manifest[file.string()]=litesim3d::sha256(file);
  importer.description["cameras"]=J::array();
  for(const auto &c:cfg.cameras)importer.description["cameras"].push_back(c.json());
  auto frames=model.fk(snapshot.joints);
  const auto wr=snapshot.world_base*frames.at(cfg.base_link).inverse();
  for(const auto &[name,local]:frames) {
    litesim3d::Pose p=wr*local;Eigen::Quaterniond q(p.linear());
    importer.description["initial_link_poses"][name]={p.translation().x(),p.translation().y(),p.translation().z(),q.x(),q.y(),q.z(),q.w()};
  }
}
void validateImportedRuntime(Importer &importer) {
  litesim3d::Scene scene;auto cfg=litesim3d::Config::load(importer.output/"runtime.yaml",scene);
  auto model=litesim3d::Model::load(cfg.urdf,cfg.model_options);
  litesim3d::Simulator validation(cfg,model,scene);
  appendValidatedRuntime(importer,validation.config(),model,validation.snapshot());
}
int importEntry(int argc,char **argv, const std::function<void(Importer &)> &validate,
                const Importer::ModelLoader &loadModel=litesim3d::Model::load) {
  try {
    require(argc==5||argc==6,"usage: sdf_import WORLD.sdf RUNTIME.yaml OUTPUT_DIR COLLISION_BUDGET [TARGET_RTF]");
    Importer importer;importer.source=fs::canonical(argv[1]);importer.output=fs::absolute(argv[3]);
    importer.loadModel=loadModel;
    require(std::string(argv[4])=="true"||std::string(argv[4])=="false","collision budget must be true or false");
    require(!fs::exists(importer.output)||fs::is_empty(importer.output),"output directory must be empty");
    fs::create_directories(importer.output);
    importer.runtime=YAML::LoadFile(argv[2]);
    require(!importer.runtime["entities"]&&!importer.runtime["robot"]["initial_pose"],
      "runtime YAML must not duplicate SDF geometry or initial pose");
    if(importer.runtime["robot"]["urdf"])
      importer.runtime["robot"]["urdf"]=resource(importer.runtime["robot"]["urdf"].as<std::string>(),fs::canonical(argv[2]).parent_path()).string();
    importer.robotName=importer.runtime["robot"]["id"].as<std::string>();
    importer.runtime["entities"]=YAML::Node(YAML::NodeType::Sequence);
    importer.runtime["scene_format"]="role_entries";
    if(!importer.runtime["robot"]["urdf"])importer.runtime["robot"]["allow_visual_only"]=true;
    importer.runtime["simulation"]["collision_budget_enabled"]=std::string(argv[4])=="true";
    sdf::ParserConfig config;
    config.SetFindCallback([&](const std::string &uri){return resource(uri,importer.source.parent_path(),true).string();});
    sdf::Root root;check(root.Load(importer.source.string(),config),"SDF import failed");
    audit(root.Element());
    require(root.WorldCount()==1,"SDF entry point must contain exactly one world");
    const auto *world=root.WorldByIndex(0);
    require(!importer.runtime["simulation"]["step_size"]&&!importer.runtime["simulation"]["real_time_factor"],
      "runtime must not duplicate SDF physics step_size or real_time_factor");
    const auto *physics=world->PhysicsDefault();
    require(physics,"SDF world must define physics timing");
    importer.runtime["simulation"]["step_size"]=physics->MaxStepSize();
    importer.runtime["simulation"]["real_time_factor"]=argc==6?std::stod(argv[5]):physics->RealTimeFactor();
    if(const auto *scene=world->Scene()) {
      importer.description["ambient"]=color(scene->Ambient());
      importer.description["background"]=color(scene->Background());
    }
    // The SDF GUI camera supplies only the initial observation pose. It does
    // not enable any Gazebo controls, plugins, selection or simulation actions.
    if(auto gui=world->Element()->FindElement("gui"))
      if(auto camera=gui->FindElement("camera")) {
        auto value=camera->Get<P>("pose",P::Zero);
        require(value.second,"invalid SDF GUI camera pose");
        importer.description["view_pose"]=pose(value.first);
      }
    require(world->ActorCount()==0,"animated actors are unsupported");
    for(uint64_t i=0;i<world->ModelCount();++i)importer.model(*world->ModelByIndex(i),P::Zero,"");
    require(importer.foundRobot,"runtime robot.id does not match a world model: "+importer.robotName);
    for(auto p:importer.runtime["entity_bindings"])
      require(importer.modelNames.count(p.first.as<std::string>()),"unknown entity binding: "+p.first.as<std::string>());
    for(uint64_t i=0;i<world->LightCount();++i) {
      auto &l=*world->LightByIndex(i);
      require(l.Type()==sdf::LightType::DIRECTIONAL||l.Type()==sdf::LightType::POINT||l.Type()==sdf::LightType::SPOT,"unsupported light");
      importer.description["lights"].push_back({{"name",l.Name()}, {"type",int(l.Type())}, {"pose",pose(resolved(l.SemanticPose()))},
        {"diffuse",color(l.Diffuse())}, {"specular",color(l.Specular())}, {"direction",xyz(l.Direction())},
        {"cast_shadows",l.CastShadows()}, {"range",l.AttenuationRange()}, {"constant",l.ConstantAttenuationFactor()},
        {"linear",l.LinearAttenuationFactor()}, {"quadratic",l.QuadraticAttenuationFactor()},
        {"inner",l.SpotInnerAngle().Radian()}, {"outer",l.SpotOuterAngle().Radian()}, {"falloff",l.SpotFalloff()}});
    }
    importer.runtime.remove("mimic_joints");importer.runtime.remove("entity_bindings");
    // Resolve sensor table paths before moving the runtime adapter to its temporary directory.
    for(auto l:importer.runtime["lidars"])if(l["pattern_file"])
      l["pattern_file"]=resource(l["pattern_file"].as<std::string>(),fs::canonical(argv[2]).parent_path()).string();
    manifest[fs::canonical(argv[2]).string()]=litesim3d::sha256(argv[2]);
    importer.description["robot_id"]=importer.robotName;
    importer.description["obstacles_fixed"]=true;
    importer.description["source"]=importer.source.string();
    std::ofstream(importer.output/"runtime.yaml")<<importer.runtime;
    // Validate all kinematics, ray/collision geometry and budgets before launch.
    validate(importer);
    importer.description["resources"]=manifest;
    importer.description["scene_id"]=litesim3d::sha256Text(importer.description.dump());
    std::ofstream(importer.output/"scene.json")<<importer.description.dump(2)<<'\n';
    std::cout<<J({{"scene_id",importer.description["scene_id"]},{"visuals",importer.description["visuals"].size()},
      {"resources",manifest.size()},{"collision_budget_enabled",std::string(argv[4])=="true"}}).dump()<<'\n';
    return 0;
  }catch(const std::exception &e){std::cerr<<"SDF import: "<<e.what()<<'\n';return 1;}
}
} // namespace
#ifndef LITESIM_SDF_IMPORT_NO_MAIN
int main(int argc,char **argv) { return importEntry(argc,argv,validateImportedRuntime); }
#endif
