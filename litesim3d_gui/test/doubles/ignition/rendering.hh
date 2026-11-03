#pragma once
// Recording rendering boundary: no Ogre, GPU, window or render thread.
// Math, resource decoding, material policy and frame serialization stay real.
#include <ignition/math/Pose3.hh>
#include <ignition/math/Color.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/Event.hh>
#include <algorithm>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>
namespace ignition::rendering {
namespace unit {
inline bool fail_engine=false,fail_scene=false,fail_camera=false,fail_depth=false,fail_material=false;
inline int destroyed=0,unloaded=0,mesh_count=-1;
inline bool missing_source=false,emit_depth=true;
inline unsigned depth_channels=1,depth_width=0,depth_height=0;
inline std::vector<float> depth_pixels;
inline void reset(){
  fail_engine=fail_scene=fail_camera=fail_depth=fail_material=missing_source=false;
  destroyed=unloaded=0;mesh_count=-1;emit_depth=true;
  depth_channels=1;depth_width=depth_height=0;depth_pixels.clear();
}
}
class Material {
public:
  math::Color ambient,diffuse,specular,emissive;
  double transparency=0,roughness=0,metalness=0,threshold=.5;
  bool cast=false,receive=false,alpha=false,two_sided=false;
  std::string texture,normal,rough,metal,emit,light;unsigned light_uv=0;
  void SetAmbient(math::Color v){ambient=v;} void SetAmbient(double r,double g,double b){ambient={float(r),float(g),float(b),1};}
  void SetDiffuse(math::Color v){diffuse=v;} void SetDiffuse(double r,double g,double b){diffuse={float(r),float(g),float(b),1};}
  void SetSpecular(math::Color v){specular=v;}
  void SetEmissive(math::Color v){emissive=v;}void SetEmissive(double r,double g,double b){emissive={float(r),float(g),float(b),1};}
  void SetTransparency(double v){transparency=v;}double Transparency()const{return transparency;}
  void SetCastShadows(bool v){cast=v;}void SetReceiveShadows(bool v){receive=v;}
  void SetRoughness(double v){roughness=v;}void SetMetalness(double v){metalness=v;}
  void SetTexture(std::string v){texture=v;}void SetNormalMap(std::string v){normal=v;}
  void SetRoughnessMap(std::string v){rough=v;}void SetMetalnessMap(std::string v){metal=v;}
  void SetEmissiveMap(std::string v){emit=v;}void SetLightMap(std::string v,unsigned uv){light=v;light_uv=uv;}
  void SetAlphaFromTexture(bool v,double t=.5,bool two=false){alpha=v;threshold=t;two_sided=two;}
  bool TextureAlphaEnabled()const{return alpha;}double AlphaThreshold()const{return threshold;}
  bool TwoSidedEnabled()const{return two_sided;}
  std::string Texture()const{return texture;}std::string NormalMap()const{return normal;}
  std::string RoughnessMap()const{return rough;}std::string MetalnessMap()const{return metal;}
  std::string EmissiveMap()const{return emit;}std::string LightMap()const{return light;}
};
using MaterialPtr=std::shared_ptr<Material>;
class Geometry {public:virtual ~Geometry()=default;};
using GeometryPtr=std::shared_ptr<Geometry>;
struct SubMesh {MaterialPtr material;void SetMaterial(MaterialPtr m,bool){material=m;}};
struct MeshDescriptor {const common::Mesh*mesh=nullptr;};
class Mesh:public Geometry {
public:
  MeshDescriptor descriptor;std::vector<std::shared_ptr<SubMesh>> pieces;
  unsigned SubMeshCount()const{return pieces.size();}
  std::shared_ptr<SubMesh> SubMeshByIndex(unsigned i){return pieces.at(i);}
  MeshDescriptor Descriptor()const{return descriptor;}
};
class Node {
public:
  virtual ~Node()=default;math::Pose3d pose;math::Vector3d scale{1,1,1};std::vector<std::shared_ptr<Node>> children;
  void SetLocalPose(math::Pose3d p){pose=p;}math::Pose3d WorldPose()const{return pose;}
  math::Vector3d WorldPosition()const{return pose.Pos();}
  void SetLocalScale(math::Vector3d s){scale=s;}void SetLocalScale(double s){scale={s,s,s};}
  void SetLocalScale(double x,double y,double z){scale={x,y,z};}
  void AddChild(std::shared_ptr<Node> n){children.push_back(n);}
};
class Visual:public Node {
public:GeometryPtr geometry;MaterialPtr material;
  void AddGeometry(GeometryPtr g){geometry=g;}void SetMaterial(MaterialPtr m,bool){material=m;}
};
using VisualPtr=std::shared_ptr<Visual>;
class Light:public Node {
public:
  int type=0;math::Vector3d direction;math::Color diffuse,specular;bool shadows=false;
  double inner=0,outer=0,falloff=0,range=0,constant=0,linear=0,quadratic=0;
  void SetDirection(math::Vector3d v){direction=v;}void SetDirection(double x,double y,double z){direction={x,y,z};}
  void SetInnerAngle(double v){inner=v;}void SetOuterAngle(double v){outer=v;}void SetFalloff(double v){falloff=v;}
  void SetDiffuseColor(math::Color v){diffuse=v;}void SetDiffuseColor(double r,double g,double b){diffuse={float(r),float(g),float(b),1};}
  void SetSpecularColor(math::Color v){specular=v;}void SetCastShadows(bool v){shadows=v;}
  void SetAttenuationRange(double v){range=v;}void SetAttenuationConstant(double v){constant=v;}
  void SetAttenuationLinear(double v){linear=v;}void SetAttenuationQuadratic(double v){quadratic=v;}
};
using LightPtr=std::shared_ptr<Light>;
enum PixelFormat { PF_R8G8B8,PF_FLOAT32_RGBA };
struct Image {std::vector<unsigned char> bytes;void* Data(){return bytes.data();}};
class Camera:public Node {
public:
  unsigned width=0,height=0,aa=0;double aspect=0,hfov=0,near=0,far=0;PixelFormat format=PF_R8G8B8;
  unsigned captures=0;
  void SetImageWidth(unsigned v){width=v;}void SetImageHeight(unsigned v){height=v;}
  unsigned ImageWidth()const{return width;}unsigned ImageHeight()const{return height;}
  void SetAspectRatio(double v){aspect=v;}void SetHFOV(double v){hfov=v;}
  void SetNearClipPlane(double v){near=v;}void SetFarClipPlane(double v){far=v;}
  void SetAntiAliasing(unsigned v){aa=v;}void SetImageFormat(PixelFormat v){format=v;}
  Image CreateImage(){return Image{std::vector<unsigned char>(size_t(width)*height*3,42)};}
  void Capture(Image&){++captures;}
};
using CameraPtr=std::shared_ptr<Camera>;
class DepthCamera:public Camera {
public:
  std::function<void(const float*,unsigned,unsigned,unsigned,const std::string&)> callback;
  bool texture=false;
  template<class F>common::ConnectionPtr ConnectNewDepthFrame(F f){callback=f;return {};}
  void CreateDepthTexture(){texture=true;}
  void Update(){
    if(!unit::emit_depth)return;
    auto data=unit::depth_pixels;if(data.empty())data.assign(size_t(width)*height,2);
    callback(data.data(),unit::depth_width?unit::depth_width:width,
             unit::depth_height?unit::depth_height:height,unit::depth_channels,"depth");
  }
};
using DepthCameraPtr=std::shared_ptr<DepthCamera>;
class Scene {
public:
  VisualPtr root=std::make_shared<Visual>();math::Color ambient,background;
  std::vector<VisualPtr>visuals;std::vector<LightPtr>lights;std::vector<CameraPtr>cameras;
  unsigned pre=0,post=0;
  VisualPtr RootVisual(){return root;}
  void SetAmbientLight(math::Color v){ambient=v;}void SetBackgroundColor(math::Color v){background=v;}
  LightPtr light(int type){auto p=std::make_shared<Light>();p->type=type;lights.push_back(p);return p;}
  LightPtr CreateDirectionalLight(){return light(3);}LightPtr CreateSpotLight(){return light(2);}LightPtr CreatePointLight(){return light(1);}
  VisualPtr CreateVisual(){auto p=std::make_shared<Visual>();visuals.push_back(p);return p;}
  GeometryPtr CreateBox(){return std::make_shared<Geometry>();}
  GeometryPtr CreateSphere(){return std::make_shared<Geometry>();}
  GeometryPtr CreateCylinder(){return std::make_shared<Geometry>();}
  GeometryPtr CreateMesh(const common::Mesh*m){
    auto p=std::make_shared<Mesh>();p->descriptor.mesh=unit::missing_source?nullptr:m;
    int count=unit::mesh_count<0?int(m->SubMeshCount()):unit::mesh_count;
    for(int i=0;i<count;++i)p->pieces.push_back(std::make_shared<SubMesh>());return p;
  }
  MaterialPtr CreateMaterial(){return unit::fail_material?nullptr:std::make_shared<Material>();}
  CameraPtr CreateCamera(){
    if(unit::fail_camera)return {};auto p=std::make_shared<Camera>();cameras.push_back(p);return p;
  }
  DepthCameraPtr CreateDepthCamera(){
    if(unit::fail_depth)return {};auto p=std::make_shared<DepthCamera>();cameras.push_back(p);return p;
  }
  void PreRender(){++pre;}void PostRender(){++post;}
};
using ScenePtr=std::shared_ptr<Scene>;
class RenderEngine {
public:
  ScenePtr current;
  ScenePtr CreateScene(const std::string&){
    current=unit::fail_scene?nullptr:std::make_shared<Scene>();return current;
  }
  void DestroyScene(ScenePtr){++unit::destroyed;current.reset();}
};
inline RenderEngine test_engine;
inline RenderEngine*engine(const std::string&,const std::map<std::string,std::string>& = {}){
  return unit::fail_engine?nullptr:&test_engine;
}
inline bool unloadEngine(const std::string&){++unit::unloaded;return true;}
class OrbitViewController {
public:
  CameraPtr camera;math::Vector3d target;math::Vector2d pan,orbit;double zoom=0;
  explicit OrbitViewController(CameraPtr c):camera(c){}
  void SetTarget(math::Vector3d t){target=t;}math::Vector3d Target()const{return target;}
  void Pan(math::Vector2d v){pan=v;}void Orbit(math::Vector2d v){orbit=v;}void Zoom(double v){zoom=v;}
};
}
