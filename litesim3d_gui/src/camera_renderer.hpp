#pragma once
#include "render_scene.hpp"
#include <ignition/rendering/DepthCamera.hh>
#include <litesim3d_interfaces/msg/camera_frame.hpp>
#include <cstring>
#include <limits>
#include <set>

namespace litesim3d_render {
class CameraRenderer {
  struct Sensor {
    J config;
    render::CameraPtr color;
    render::DepthCameraPtr depth;
    ignition::common::ConnectionPtr connection;
    std::vector<float> pixels;
  };
public:
  explicit CameraRenderer(const J &description):world_(description) {
    for(const auto &config:description.value("cameras",J::array())) {
      if(!config.at("enabled").get<bool>())continue;
      auto sensor=std::make_unique<Sensor>();sensor->config=config;
      const auto type=config.at("type").get<std::string>();
      auto configure=[&](const render::CameraPtr &camera) {
        require(bool(camera),"cannot create camera renderer");
        camera->SetImageWidth(config.at("width"));camera->SetImageHeight(config.at("height"));
        camera->SetAspectRatio(config.at("width").get<double>()/config.at("height").get<double>());
        camera->SetHFOV(config.at("horizontal_fov_deg").get<double>()*M_PI/180);
        camera->SetNearClipPlane(config.at("near_clip_m"));camera->SetFarClipPlane(config.at("far_clip_m"));
        camera->SetAntiAliasing(0);world_.scene()->RootVisual()->AddChild(camera);
      };
      if(type=="rgb"||type=="rgbd") {
        sensor->color=world_.scene()->CreateCamera();configure(sensor->color);
        sensor->color->SetImageFormat(render::PF_R8G8B8);
      }
      if(type=="depth"||type=="rgbd") {
        sensor->depth=world_.scene()->CreateDepthCamera();configure(sensor->depth);
        sensor->depth->SetImageFormat(render::PF_FLOAT32_RGBA);
        auto *raw=sensor.get();
        sensor->connection=sensor->depth->ConnectNewDepthFrame(
          [raw](const float *data,unsigned width,unsigned height,unsigned channels,const std::string &) {
            require(width==raw->config.at("width")&&height==raw->config.at("height")&&channels==1,
                    "unexpected depth buffer dimensions");
            raw->pixels.assign(data,data+size_t(width)*height);
          });
        sensor->depth->CreateDepthTexture();
      }
      require(sensors_.emplace(config.at("name").get<std::string>(),std::move(sensor)).second,"duplicate camera");
    }
  }
  std::vector<litesim3d_interfaces::msg::CameraFrame> capture(
      const std::vector<std::string> &names,const J &poses,const std_msgs::msg::Header &header) {
    require(!names.empty()&&names.size()<=sensors_.size(),"invalid camera batch");
    std::set<std::string> unique;
    for(const auto &name:names)require(sensors_.count(name)&&unique.insert(name).second,"unknown or repeated camera: "+name);
    world_.applyPoses(poses);
    std::vector<litesim3d_interfaces::msg::CameraFrame> result;
    for(const auto &name:names) {
      auto &s=*sensors_.at(name);const auto &c=s.config;
      const unsigned w=c.at("width"),h=c.at("height");
      // Gazebo rendering uses +x forward, +y left, +z up; ROS images use optical axes.
      const math::Pose3d opticalInBody(0,0,0,-M_PI/2,0,-M_PI/2);
      const auto worldCamera=pose(poses.at(c.at("link").get<std::string>()))*opticalInBody.Inverse();
      litesim3d_interfaces::msg::CameraFrame frame;frame.camera_name=name;
      frame.info.header=header;frame.info.header.frame_id=c.at("link");
      frame.info.width=w;frame.info.height=h;frame.info.distortion_model="plumb_bob";
      frame.info.d.assign(5,0);
      const double f=w/(2*std::tan(c.at("horizontal_fov_deg").get<double>()*M_PI/360));
      const double cx=(w-1)*.5,cy=(h-1)*.5;
      frame.info.k={f,0,cx,0,f,cy,0,0,1};frame.info.r={1,0,0,0,1,0,0,0,1};
      frame.info.p={f,0,cx,0,0,f,cy,0,0,0,1,0};
      auto initialize=[&](sensor_msgs::msg::Image &image,const std::string &encoding,unsigned bytes) {
        image.header=frame.info.header;image.width=w;image.height=h;image.encoding=encoding;
        image.is_bigendian=false;image.step=w*bytes;image.data.resize(size_t(image.step)*h);
      };
      if(s.color) {
        s.color->SetLocalPose(worldCamera);world_.scene()->PreRender();
        auto image=s.color->CreateImage();s.color->Capture(image);world_.scene()->PostRender();
        initialize(frame.color,"rgb8",3);std::memcpy(frame.color.data.data(),image.Data(),frame.color.data.size());
      }
      if(s.depth) {
        s.depth->SetLocalPose(worldCamera);s.pixels.clear();s.depth->Update();
        require(s.pixels.size()==size_t(w)*h,"renderer did not produce a depth frame");
        initialize(frame.depth,"32FC1",4);
        const float near=c.at("near_clip_m"),far=c.at("far_clip_m");
        for(size_t i=0;i<s.pixels.size();++i) {
          float z=s.pixels[i];
          // Missing/clipped depth is invalid, never a zero-distance obstacle.
          if(!std::isfinite(z)||z<near||z>=far)z=std::numeric_limits<float>::quiet_NaN();
          uint32_t bits;std::memcpy(&bits,&z,4);
          for(unsigned b=0;b<4;++b)frame.depth.data[i*4+b]=uint8_t(bits>>(8*b));
        }
      }
      result.push_back(std::move(frame));
    }
    return result;
  }
private:
  RenderScene world_;
  std::map<std::string,std::unique_ptr<Sensor>> sensors_;
};
} // namespace litesim3d_render
