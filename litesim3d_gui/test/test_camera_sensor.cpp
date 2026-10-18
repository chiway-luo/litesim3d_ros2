#include "../src/camera_renderer.hpp"
#include <fstream>
using namespace litesim3d_render;
float pixel(const sensor_msgs::msg::Image &image,unsigned x,unsigned y) {
  float value;std::memcpy(&value,image.data.data()+y*image.step+x*4,4);return value;
}
int main(int argc,char **argv) {
  if(argc!=2)return 2;
  QApplication app(argc,argv);int status=0;
  try {
    std::filesystem::create_directories(argv[1]);
    J description={{"schema_version",1},{"resources",J::object()},{"lights",J::array()},
      {"ambient",{1,1,1,1}},{"background",{0,0,0,1}}, {"visuals",J::array()},
      {"initial_link_poses",{{"optical",{0,0,0,-.5,.5,-.5,.5}}}}, {"cameras",J::array()}};
    description["visuals"].push_back({{"id","wall"},{"binding",""},{"pose",{3,0,0,0,0,0,1}},
      {"geometry",{{"type","box"},{"size",{.2,20,20}}}},
      {"material",{{"diffuse",{.8,.1,.05,1}},{"ambient",{.8,.1,.05,1}},{"transparency",0},{"cast_shadows",false}}}});
    description["visuals"].push_back({{"id","blue_target"},{"binding",""},{"pose",{2,.6,.3,0,0,0,1}},
      {"geometry",{{"type","box"},{"size",{.2,.2,.2}}}},
      {"material",{{"diffuse",{.02,.05,.9,1}},{"ambient",{.02,.05,.9,1}},{"transparency",0},{"cast_shadows",false}}}});
    for(const auto &type:{"rgbd","rgb","depth"})description["cameras"].push_back({
      {"name",type},{"link","optical"},{"topic",type},{"type",type},{"enabled",true},
      {"width",160},{"height",120},{"rate_hz",15.0},{"horizontal_fov_deg",90.0},{"near_clip_m",.1},{"far_clip_m",10.0}});
    CameraRenderer renderer(description);auto poses=description.at("initial_link_poses");
    std_msgs::msg::Header header;header.stamp.sec=1;
    auto frames=renderer.capture({"rgbd","rgb","depth"},poses,header);
    for(const auto &frame:frames) {
      require(frame.info.header.stamp==header.stamp&&frame.info.header.frame_id=="optical","bad camera metadata");
      require(std::abs(frame.info.k[0]-80)<1e-8&&frame.info.k[2]==79.5,"bad camera intrinsics");
      if(frame.camera_name!="rgb")for(auto x:{20U,80U,140U}) {
        float z=pixel(frame.depth,x,60);
        require(std::abs(z-2.9)<.003,"depth must be axial 2.9 m at all columns; got "+std::to_string(z));
      }
      if(frame.camera_name!="depth") {
        const auto *p=frame.color.data.data()+60*frame.color.step+80*3;
        require(p[0]>p[1]*2&&p[0]>p[2]*2&&p[0]>80,"RGB texture/material output is wrong or black");
      }
    }
    require(frames[1].depth.data.empty()&&frames[2].color.data.empty(),"wrong camera mode outputs");
    // An off-axis target tests optical handedness, projection and RGB/depth registration.
    const auto &registered=frames[0];unsigned bluePixels=0;
    for(unsigned y=42;y<=51;++y)for(unsigned x=50;x<=59;++x) {
      const auto *p=registered.color.data.data()+y*registered.color.step+x*3;
      if(p[2]>p[0]*3&&p[2]>p[1]*3&&p[2]>80) {
        const double z=pixel(registered.depth,x,y);
        // The off-axis view includes the cube's side as well as its front face.
        require(z>=1.897&&z<=2.103,"RGB/depth pixels are not aligned at "+std::to_string(x)+","+std::to_string(y)+": "+std::to_string(z));
        const double opticalX=(x-registered.info.k[2])*z/registered.info.k[0];
        const double opticalY=(y-registered.info.k[5])*z/registered.info.k[4];
        require(opticalX>=-.71&&opticalX<=-.49&&opticalY>=-.41&&opticalY<=-.19,
                "camera_info projection does not reconstruct the target");
        ++bluePixels;
      }
    }
    require(bluePixels>30,"camera optical orientation/projection is wrong");
    auto &color=frames[0].color;
    QImage image(color.data.data(),color.width,color.height,color.step,QImage::Format_RGB888);
    require(image.save(QString::fromStdString(std::string(argv[1])+"/rgb.png")),"save failed");
    std::ofstream raw(std::string(argv[1])+"/depth.f32",std::ios::binary);
    raw.write(reinterpret_cast<const char *>(frames[0].depth.data.data()),frames[0].depth.data.size());raw.close();
    poses["optical"][0]=.5;header.stamp.sec=2;
    auto moved=renderer.capture({"rgbd"},poses,header);
    require(std::abs(pixel(moved[0].depth,80,60)-2.4)<.003,"camera pose update did not affect depth");
    poses["optical"][0]=-20;header.stamp.sec=3;
    auto outside=renderer.capture({"depth"},poses,header);
    require(std::isnan(pixel(outside[0].depth,80,60)),"out-of-range depth must be NaN");
    std::cout<<J({{"status","PASS"},{"axial_depth_m",pixel(frames[0].depth,140,60)},
      {"moved_depth_m",pixel(moved[0].depth,80,60)},{"modes",{"rgb","depth","rgbd"}},
      {"registered_target_pixels",bluePixels},
      {"invalid_depth","NaN"},{"width",160},{"height",120}}).dump(2)<<'\n';
  }catch(const std::exception &e){std::cerr<<e.what()<<'\n';status=1;}
  return status;
}
