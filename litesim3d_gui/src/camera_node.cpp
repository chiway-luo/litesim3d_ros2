#include "camera_renderer.hpp"
#include <litesim3d_interfaces/srv/render_cameras.hpp>

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);QApplication app(argc,argv);app.setQuitOnLastWindowClosed(false);
  int result=0;
  try {
    using namespace litesim3d_render;
    auto node=std::make_shared<rclcpp::Node>("litesim_camera_renderer");
    const auto file=node->declare_parameter<std::string>("scene_description","");
    J description;std::ifstream input(file);require(bool(input),"missing camera scene description");input>>description;
    CameraRenderer renderer(description);
    const auto initial=description.at("initial_link_poses");
    // Warm up every configured target before advertising readiness.
    std::vector<std::string> names;
    for(const auto &c:description.at("cameras"))if(c.at("enabled").get<bool>())names.push_back(c.at("name"));
    require(!names.empty(),"no cameras enabled");
    renderer.capture(names,initial,std_msgs::msg::Header{});
    using Service=litesim3d_interfaces::srv::RenderCameras;
    std::string epoch;int64_t last=-1;
    auto service=node->create_service<Service>("/sim/render_cameras",
      [&](Service::Request::SharedPtr request,Service::Response::SharedPtr response) {
        try {
          require(request->scene_id==description.at("scene_id"),"camera scene mismatch");
          require(!request->run_id.empty(),"missing camera run_id");
          require(request->link_names.size()==request->link_poses.size(),"invalid pose array");
          const int64_t stamp=int64_t(request->header.stamp.sec)*1000000000+request->header.stamp.nanosec;
          require(stamp>0&&(epoch!=request->run_id||stamp>last),"non-increasing camera timestamp");
          J poses=J::object();
          for(size_t i=0;i<request->link_names.size();++i) {
            const auto &p=request->link_poses[i];
            require(!poses.contains(request->link_names[i]),"duplicate pose link");
            poses[request->link_names[i]]={p.position.x,p.position.y,p.position.z,
              p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w};
          }
          require(poses.size()==initial.size(),"incomplete rendering pose snapshot");
          for(auto it=initial.begin();it!=initial.end();++it)require(poses.contains(it.key()),"missing link pose");
          response->frames=renderer.capture(request->camera_names,poses,request->header);
          epoch=request->run_id;last=stamp;response->success=true;
        } catch(const std::exception &e) {response->message=e.what();}
      });
    QTimer timer;QObject::connect(&timer,&QTimer::timeout,[&] {
      if(!rclcpp::ok()){app.quit();return;}rclcpp::spin_some(node);
    });timer.start(1);
    RCLCPP_INFO(node->get_logger(),"Camera renderer ready (%zu cameras)",names.size());
    result=app.exec();
  }catch(const std::exception &e){std::cerr<<"camera renderer: "<<e.what()<<'\n';result=1;}
  if(rclcpp::ok())rclcpp::shutdown();return result;
}
