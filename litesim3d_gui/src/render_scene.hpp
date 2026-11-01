#pragma once
#include <ignition/rendering.hh>
#include <ignition/rendering/config.hh>
#include <ignition/rendering/OrbitViewController.hh>
#include <ignition/common/Console.hh>
#include <ignition/common/MeshManager.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/common/Material.hh>
#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <QApplication>
#include <QIcon>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QStandardPaths>
#include <QDir>
#include <QFile>
#include <QCryptographicHash>
#include <QImageReader>
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <filesystem>
#include <algorithm>
#include <cmath>


namespace litesim3d_render {
using J=nlohmann::json;
namespace render=ignition::rendering;
namespace math=ignition::math;
using Clock=std::chrono::steady_clock;
math::Vector3d vec(const J &v){return {v.at(0).get<double>(),v.at(1).get<double>(),v.at(2).get<double>()};}
math::Color color(const J &v){return {v.at(0).get<float>(),v.at(1).get<float>(),v.at(2).get<float>(),v.at(3).get<float>()};}
math::Pose3d pose(const J &p){return {{p.at(0).get<double>(),p.at(1).get<double>(),p.at(2).get<double>()},
  {p.at(6).get<double>(),p.at(3).get<double>(),p.at(4).get<double>(),p.at(5).get<double>()}};}
void require(bool test,const std::string &message){if(!test)throw std::runtime_error(message);}
void completeNormals(const ignition::common::Mesh *mesh) {
  for (unsigned k=0;k<mesh->SubMeshCount();++k) {
    auto sub=mesh->SubMeshByIndex(k).lock();
    require(sub&&sub->SubMeshPrimitiveType()==ignition::common::SubMesh::TRIANGLES,
      "visual meshes must contain triangles");
    bool missing=sub->NormalCount()!=sub->VertexCount();
    for(unsigned n=0;n<sub->NormalCount();++n)
      missing=missing||!sub->Normal(n).IsFinite()||sub->Normal(n).Length()<1e-9;
    if(!missing)continue;
    require(sub->IndexCount()%3==0,"invalid triangle index count");
    std::vector<math::Vector3d> normals(sub->VertexCount(),math::Vector3d::Zero);
    for(unsigned i=0;i<sub->IndexCount();i+=3) {
      unsigned a=sub->Index(i),b=sub->Index(i+1),c=sub->Index(i+2);
      require(a<normals.size()&&b<normals.size()&&c<normals.size(),"invalid mesh index");
      auto n=(sub->Vertex(b)-sub->Vertex(a)).Cross(sub->Vertex(c)-sub->Vertex(a));
      normals[a]+=n;normals[b]+=n;normals[c]+=n;
    }
    // common::SubMesh::RecalculateNormals is a no-op when no normals exist.
    // Generate them in linear time; preserve supplied normals on valid meshes.
    for(unsigned i=0;i<normals.size();++i) {
      auto n=normals[i].Length()>1e-12?normals[i].Normalized():math::Vector3d::UnitZ;
      if(i<sub->NormalCount())sub->SetNormal(i,n);else sub->AddNormal(n);
    }
  }
}
struct Item{render::VisualPtr visual;std::string binding;math::Pose3d local;};
class RenderScene {
public:
  explicit RenderScene(J description):description_(std::move(description)) {
    require(description_.at("schema_version")==1,"unsupported scene schema");
    for(auto it=description_.at("resources").begin();it!=description_.at("resources").end();++it) {
      QFile file(QString::fromStdString(it.key()));require(file.open(QIODevice::ReadOnly),"missing scene resource: "+it.key());
      QCryptographicHash hash(QCryptographicHash::Sha256);require(hash.addData(&file),"cannot hash resource: "+it.key());
      require(hash.result().toHex().toStdString()==it.value().get<std::string>(),"scene resource changed after import: "+it.key());
    }
    const auto log=QStandardPaths::writableLocation(QStandardPaths::TempLocation)+"/litesim3d-rendering";
    QDir().mkpath(log);ignition::common::Console::log.Init(log.toStdString(),"viewer.log");
    // Conda relocates binary string literals with NUL padding. Ogre concatenates
    // its compiled-in resource directory, retaining that padding and silently
    // missing material scripts. The installed SDK header has a normal path.
    // Set only this process's default; preserve an explicit caller override.
    auto resourceRoot=qgetenv("IGN_RENDERING_RESOURCE_PATH");
    if(resourceRoot.isEmpty())resourceRoot=QByteArray(IGN_RENDERING_RESOURCE_PATH);
    require(std::filesystem::is_regular_file(std::filesystem::path(resourceRoot.constData()) /
        "ogre2/media/materials/scripts/depth_camera.material"),
        "Ogre material resources missing; set IGN_RENDERING_RESOURCE_PATH to the installed rendering share directory");
    qputenv("IGN_RENDERING_RESOURCE_PATH",resourceRoot);
#ifdef __APPLE__
    engine_=render::engine("ogre2",{{"metal","1"}});
#else
    engine_=render::engine("ogre2");
#endif
    require(engine_,"Ogre2 rendering backend is unavailable");
    try { initializeScene(); }
    catch (...) {
      // A throwing constructor never runs ~RenderScene. Release partially
      // imported visuals and the engine before the caller retries a scene.
      release();
      throw;
    }
  }
private:
  void initializeScene() {
    scene_=engine_->CreateScene("litesim_viewer");require(bool(scene_),"cannot create rendering scene");
    scene_->SetAmbientLight(color(description_.value("ambient",J::array({.35,.35,.35,1}))));
    scene_->SetBackgroundColor(color(description_.value("background",J::array({.075,.09,.12,1}))));
    auto root=scene_->RootVisual();
    for(const auto &l:description_.at("lights")) {
      render::LightPtr light;
      const int type=l.at("type");
      if(type==3){auto d=scene_->CreateDirectionalLight();d->SetDirection(vec(l.at("direction")));light=d;}
      else if(type==2){auto d=scene_->CreateSpotLight();d->SetDirection(vec(l.at("direction")));d->SetInnerAngle(l.at("inner").get<double>());d->SetOuterAngle(l.at("outer").get<double>());d->SetFalloff(l.at("falloff"));light=d;}
      else light=scene_->CreatePointLight();
      light->SetLocalPose(pose(l.at("pose")));light->SetDiffuseColor(color(l.at("diffuse")));light->SetSpecularColor(color(l.at("specular")));
      light->SetCastShadows(l.at("cast_shadows"));light->SetAttenuationRange(l.at("range"));light->SetAttenuationConstant(l.at("constant"));
      light->SetAttenuationLinear(l.at("linear"));light->SetAttenuationQuadratic(l.at("quadratic"));root->AddChild(light);
    }
    if(description_.at("lights").empty()){auto l=scene_->CreateDirectionalLight();l->SetDirection(-.6,-.4,-1);l->SetDiffuseColor(1,1,1);root->AddChild(l);}
    for(const auto &item:description_.at("visuals")) {
      auto v=scene_->CreateVisual();const auto &g=item.at("geometry");const auto type=g.at("type").get<std::string>();
      render::GeometryPtr geometry;
      if(type=="box"){geometry=scene_->CreateBox();v->SetLocalScale(vec(g.at("size")));}
      else if(type=="sphere"){geometry=scene_->CreateSphere();v->SetLocalScale(2*g.at("radius").get<double>());}
      else if(type=="cylinder"){geometry=scene_->CreateCylinder();v->SetLocalScale(2*g.at("radius").get<double>(),2*g.at("radius").get<double>(),g.at("length"));}
      else if(type=="mesh"){auto mesh=loadMesh(g.at("uri").get<std::string>());require(mesh,"mesh loader failed: "+g.at("uri").get<std::string>());completeNormals(mesh);geometry=scene_->CreateMesh(mesh);v->SetLocalScale(vec(g.at("scale")));}
      require(bool(geometry),"rendering geometry failed: "+item.at("id").get<std::string>());v->AddGeometry(geometry);
      const auto &m=item.at("material");
      auto apply=[&](render::MaterialPtr mat){
        require(bool(mat),"missing mesh material");
        if(m.contains("ambient"))mat->SetAmbient(color(m.at("ambient")));
        if(m.contains("diffuse"))mat->SetDiffuse(color(m.at("diffuse")));
        if(m.contains("specular"))mat->SetSpecular(color(m.at("specular")));
        if(m.contains("emissive"))mat->SetEmissive(color(m.at("emissive")));
        mat->SetTransparency(1-(1-mat->Transparency())*(1-m.at("transparency").get<double>()));mat->SetCastShadows(m.at("cast_shadows"));
        mat->SetReceiveShadows(true);
        if(m.contains("roughness"))mat->SetRoughness(m.at("roughness"));
        if(m.contains("metalness"))mat->SetMetalness(m.at("metalness"));
        if(m.contains("albedo_map")){mat->SetTexture(m.at("albedo_map"));mat->SetAlphaFromTexture(true);}
        if(m.contains("normal_map"))mat->SetNormalMap(m.at("normal_map"));
        if(m.contains("roughness_map"))mat->SetRoughnessMap(m.at("roughness_map"));
        if(m.contains("metalness_map"))mat->SetMetalnessMap(m.at("metalness_map"));
        if(m.contains("emissive_map"))mat->SetEmissiveMap(m.at("emissive_map"));
        if(m.contains("light_map"))mat->SetLightMap(m.at("light_map"),m.at("light_map_uv"));
        mat->SetAlphaFromTexture(mat->TextureAlphaEnabled(),mat->AlphaThreshold(),m.value("double_sided",mat->TwoSidedEnabled()));
        for(const auto &file:{mat->Texture(),mat->NormalMap(),mat->RoughnessMap(),mat->MetalnessMap(),mat->EmissiveMap(),mat->LightMap()})
          if(!file.empty()){QImageReader reader(QString::fromStdString(file));require(!reader.read().isNull(),"cannot decode texture: "+file);}
      };
      if(auto mesh=std::dynamic_pointer_cast<render::Mesh>(geometry)) {
        require(mesh->SubMeshCount()>0,"mesh has no rendered submeshes");
        auto neutral=scene_->CreateMaterial();neutral->SetEmissive(0,0,0);v->SetMaterial(neutral,false);
        // Preserve each sub-material separately: a uniform replacement would lose
        // texture slots and turn a multi-material model into a single color.
        for(unsigned i=0;i<mesh->SubMeshCount();++i){
          auto sub=mesh->SubMeshByIndex(i);auto mat=scene_->CreateMaterial();
          mat->SetDiffuse(.7,.7,.7);mat->SetAmbient(.7,.7,.7);mat->SetEmissive(0,0,0);mat->SetRoughness(.65);mat->SetMetalness(0);
          auto source=mesh->Descriptor().mesh;
          if(type=="mesh"&&source&&i<source->SubMeshCount()) {
            auto piece=source->SubMeshByIndex(i).lock();
            if(piece&&piece->MaterialIndex()<source->MaterialCount()) {
              auto original=source->MaterialByIndex(piece->MaterialIndex());
              mat->SetAmbient(original->Ambient());mat->SetDiffuse(original->Diffuse());mat->SetSpecular(original->Specular());
              mat->SetEmissive(original->Emissive());mat->SetTransparency(original->Transparency());
              if(!original->TextureImage().empty()){mat->SetTexture(original->TextureImage());mat->SetAlphaFromTexture(true);}
            }
          }
          apply(mat);sub->SetMaterial(mat,false);
        }
      }else{auto mat=scene_->CreateMaterial();apply(mat);v->SetMaterial(mat,false);}
      const auto local=pose(item.at("pose"));v->SetLocalPose(local);root->AddChild(v);
      items_.push_back({v,item.at("binding").get<std::string>(),local});
    }
    applyPoses(description_.at("initial_link_poses"));
  }
public:
  ~RenderScene() { release(); }
  RenderScene(const RenderScene &)=delete;
  RenderScene &operator=(const RenderScene &)=delete;
  render::ScenePtr scene() const { return scene_; }
  std::vector<math::Pose3d> modelPoses() const {
    std::vector<math::Pose3d> result;for(const auto &i:items_)result.push_back(i.visual->WorldPose());return result;
  }
  const ignition::common::Mesh *loadMesh(const std::string &file) {
    return loadMesh(file,collada_);
  }
  using MeshCache=std::map<std::string,std::unique_ptr<ignition::common::Mesh>>;
  // File decoding/cache ownership does not depend on a rendering engine.
  static const ignition::common::Mesh *loadMesh(const std::string &file,MeshCache &cache) {
    if(std::filesystem::path(file).extension()!=".dae")
      return ignition::common::MeshManager::Instance()->Load(file);
    auto found=cache.find(file);if(found!=cache.end())return found->second.get();
    // Use the same COLLADA decoder / node transforms as the collision kernel.
    // Older gz-common loaders misread legal shared-index polylist inputs.
    Assimp::Importer loader;loader.SetPropertyBool(AI_CONFIG_IMPORT_COLLADA_IGNORE_UP_DIRECTION,true);
    auto *source=loader.ReadFile(file,aiProcess_Triangulate|aiProcess_PreTransformVertices|aiProcess_FlipUVs|aiProcess_ValidateDataStructure);
    require(source&&source->HasMeshes(),"cannot decode COLLADA: "+file);
    auto mesh=std::make_unique<ignition::common::Mesh>();mesh->SetName(file);mesh->SetPath(std::filesystem::path(file).parent_path().string());
    for(unsigned i=0;i<source->mNumMaterials;++i) {
      auto *input=source->mMaterials[i];auto material=std::make_shared<ignition::common::Material>();
      aiColor4D c;auto cv=[](const aiColor4D &v){return math::Color(v.r,v.g,v.b,v.a);};
      if(input->Get(AI_MATKEY_COLOR_DIFFUSE,c)==AI_SUCCESS)material->SetDiffuse(cv(c));
      if(input->Get(AI_MATKEY_COLOR_AMBIENT,c)==AI_SUCCESS)material->SetAmbient(cv(c));
      if(input->Get(AI_MATKEY_COLOR_SPECULAR,c)==AI_SUCCESS)material->SetSpecular(cv(c));
      if(input->Get(AI_MATKEY_COLOR_EMISSIVE,c)==AI_SUCCESS)material->SetEmissive(cv(c));
      float opacity=1;input->Get(AI_MATKEY_OPACITY,opacity);material->SetTransparency(1-opacity);
      aiString texture;if(input->GetTexture(aiTextureType_DIFFUSE,0,&texture)==AI_SUCCESS)
        material->SetTextureImage(texture.C_Str(),std::filesystem::path(file).parent_path().string());
      mesh->AddMaterial(material);
    }
    for(unsigned i=0;i<source->mNumMeshes;++i) {
      auto *input=source->mMeshes[i];auto sub=std::make_unique<ignition::common::SubMesh>();
      sub->SetName("part_"+std::to_string(i));sub->SetMaterialIndex(input->mMaterialIndex);
      for(unsigned v=0;v<input->mNumVertices;++v) {
        const auto p=input->mVertices[v];sub->AddVertex(p.x,p.y,p.z);
        if(input->HasNormals()){const auto n=input->mNormals[v];sub->AddNormal(n.x,n.y,n.z);}
        for(unsigned uv=0;uv<AI_MAX_NUMBER_OF_TEXTURECOORDS;++uv)if(input->HasTextureCoords(uv)) {
          const auto t=input->mTextureCoords[uv][v];sub->AddTexCoordBySet(t.x,t.y,uv);
        }
      }
      for(unsigned f=0;f<input->mNumFaces;++f){const auto &face=input->mFaces[f];require(face.mNumIndices==3,"non-triangle COLLADA primitive");for(unsigned j=0;j<3;++j)sub->AddIndex(face.mIndices[j]);}
      mesh->AddSubMesh(std::move(sub));
    }
    auto result=mesh.get();cache[file]=std::move(mesh);return result;
  }
  void applyPoses(const J &poses){for(auto &i:items_)if(!i.binding.empty()&&poses.contains(i.binding))i.visual->SetLocalPose(pose(poses.at(i.binding))*i.local);}
private:
  void release() {
    items_.clear();
    if(engine_&&scene_){engine_->DestroyScene(scene_);scene_.reset();}
    if(engine_){render::unloadEngine("ogre2");engine_=nullptr;}
  }
  J description_;
  render::RenderEngine *engine_=nullptr;render::ScenePtr scene_;
  std::vector<Item> items_;
  MeshCache collada_;
};
} // namespace litesim3d_render
