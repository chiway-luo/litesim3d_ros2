#include "render_scene.hpp"
#include "ros_event_loop.hpp"
#include <ignition/rendering.hh>
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

namespace {
using namespace litesim3d_render;
class Viewer final:public QWidget {
public:
  explicit Viewer(std::shared_ptr<rclcpp::Node> node):node_(std::move(node)),
      executor_(ros_executor_options(node_->get_node_base_interface()->get_context())) {
    executor_.add_node(node_);
    setWindowTitle("LiteSim 3D");resize(1100,740);setMinimumSize(480,320);
    setFocusPolicy(Qt::NoFocus);setContextMenuPolicy(Qt::NoContextMenu);
    const auto filename=node_->declare_parameter<std::string>("scene_description","");
    if(!filename.empty()) {
      std::ifstream input(filename);require(bool(input),"cannot open scene description: "+filename);input>>description_;
    } else {
      // A restarted viewer retrieves the latched description without changing
      // simulation state or relying on launch's temporary file path.
      auto description=node_->create_subscription<std_msgs::msg::String>("/sim/scene_description",
        rclcpp::QoS(1).reliable().transient_local(),[this](std_msgs::msg::String::ConstSharedPtr msg){description_=J::parse(msg->data);});
      auto deadline=Clock::now()+std::chrono::seconds(10);
      while(description_.is_null()&&Clock::now()<deadline) {
        if(!process_ros_events(node_->get_node_base_interface()->get_context(),
            [this]{executor_.spin_some();}))break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
      require(!description_.is_null(),"timed out waiting for /sim/scene_description");
    }
    rendering_=std::make_unique<RenderScene>(description_);scene_=rendering_->scene();
    auto root=scene_->RootVisual();
    camera_=scene_->CreateCamera();camera_->SetImageWidth(1100);camera_->SetImageHeight(700);camera_->SetImageFormat(render::PF_R8G8B8);
    camera_->SetHFOV(1.05);camera_->SetNearClipPlane(.02);camera_->SetFarClipPlane(1000);camera_->SetAntiAliasing(0);
    camera_->SetLocalPose({-4,-6,4.2,0,.47,.98});root->AddChild(camera_);
    orbit_=std::make_unique<render::OrbitViewController>(camera_);orbit_->SetTarget({1,0,.4});
    if(description_.contains("view_pose")) {
      const auto view=pose(description_.at("view_pose"));
      camera_->SetLocalPose(view);
      const auto forward=view.Rot().RotateVector(math::Vector3d::UnitX);
      const double distance=forward.Z() < -1e-6 && view.Pos().Z() > 0
          ? view.Pos().Z() / -forward.Z() : 10.0;
      orbit_->SetTarget(view.Pos()+forward*distance);
    }
    // No publishers, service clients, simulation actions, key shortcuts or
    // picking handlers exist in this process. Only camera transforms change.
    states_=node_->create_subscription<std_msgs::msg::String>("/sim/scene_state",rclcpp::QoS(1).best_effort(),[this](std_msgs::msg::String::ConstSharedPtr msg){
      const auto state=J::parse(msg->data);if(state.at("scene_id")!=description_.at("scene_id"))return;
      if(!epoch_.empty()&&epoch_!=state.at("run_id").get<std::string>())statistics_=J();
      epoch_=state.at("run_id").get<std::string>();rendering_->applyPoses(state.at("link_poses"));
    });
    stats_=node_->create_subscription<std_msgs::msg::String>("/sim/stats",rclcpp::QoS(1).best_effort(),[this](std_msgs::msg::String::ConstSharedPtr msg){
      auto value=J::parse(msg->data);if(!epoch_.empty()&&value.at("run_id")!=epoch_)return;statistics_=value;lastStats_=Clock::now();
    });
    snapshot_=node_->declare_parameter<std::string>("capture_path","");
    exitAfter_=node_->declare_parameter<int>("exit_after_frames",0);
    readyFile_=node_->declare_parameter<std::string>("ready_file","");
    started_=fpsStart_=Clock::now();
    connect(&timer_,&QTimer::timeout,this,[this]{
      try{
        if(!litesim3d_render::process_ros_events(node_->get_node_base_interface()->get_context(),
            [this]{executor_.spin_some();})){close();return;}
        frame();
      }catch(const std::exception &e){std::cerr<<"viewer: "<<e.what()<<'\n';QApplication::exit(1);}
    });timer_.start(33);
  }
  ~Viewer() override {
    timer_.stop();orbit_.reset();camera_.reset();scene_.reset();rendering_.reset();
  }
#ifdef LITESIM_VIEWER_TEST
  math::Pose3d cameraPose()const{return camera_->WorldPose();}
  std::vector<math::Pose3d> modelPoses()const{return rendering_->modelPoses();}
  void renderForTest(){frame();}
#endif
protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);p.fillRect(rect(),QColor(19,23,31));if(!image_.isNull())p.drawImage(QRect(0,0,width(),height()-36),image_);
    p.fillRect(QRect(0,height()-36,width(),36),QColor(25,30,39));p.setPen(QColor(224,230,240));
    QString status="Waiting for simulation";
    if(!statistics_.is_null()){
      auto actual=statistics_.at("actual_real_time_factor");const double target=statistics_.at("target_real_time_factor");
      status=QString("RTF %1   Target %2   Sim %3 s   %4").arg(actual.is_null()?QString("warming up"):QString::number(actual.get<double>()*100,'f',1)+"%")
        .arg(target==0?QString("unlimited"):QString::number(target*100,'f',0)+"%")
        .arg(statistics_.at("stamp_ns").get<double>()*1e-9,0,'f',2).arg(QString::fromStdString(statistics_.at("state")));
      status+=statistics_.value("collision_budget_enabled",false)?"   Collision budget ON":"   Collision budget OFF";
      if(Clock::now()-lastStats_>std::chrono::seconds(2))status="Disconnected — "+status;
    }
    p.drawText(14,height()-12,status+QString("   FPS %1").arg(fps_,0,'f',1));
  }
  void mousePressEvent(QMouseEvent *e) override{lastMouse_=e->pos();}
  void mouseMoveEvent(QMouseEvent *e) override{
    const auto delta=e->pos()-lastMouse_;lastMouse_=e->pos();
    if((e->modifiers()&Qt::ShiftModifier)&&(e->buttons()&(Qt::LeftButton|Qt::MiddleButton|Qt::RightButton)))
      orbit_->Pan({double(delta.x()),double(delta.y())});
    else if(e->buttons()&Qt::LeftButton)orbit_->Orbit({double(delta.x()),double(delta.y())});
  }
  void wheelEvent(QWheelEvent *e) override{
    const double distance=(camera_->WorldPosition()-orbit_->Target()).Length();
    const double steps=e->angleDelta().y()/120.0;
    const double target=std::clamp(distance*std::exp(-steps*.12),.08,500.0);orbit_->Zoom(distance-target);e->accept();
  }
private:
  void frame(){
    const unsigned w=unsigned(width()),h=unsigned(height()-36);
    if(camera_->ImageWidth()!=w||camera_->ImageHeight()!=h){camera_->SetImageWidth(w);camera_->SetImageHeight(h);}
    camera_->SetAspectRatio(double(w)/h);scene_->PreRender();
    auto pixels=camera_->CreateImage();camera_->Capture(pixels);scene_->PostRender();
    image_=QImage(static_cast<const uchar *>(pixels.Data()),int(w),int(h),int(w*3),QImage::Format_RGB888).copy();
    ++frames_;++fpsFrames_;auto now=Clock::now();const double span=std::chrono::duration<double>(now-fpsStart_).count();
    if(span>=1){fps_=fpsFrames_/span;fpsFrames_=0;fpsStart_=now;}
    if(frames_==3&&!readyFile_.empty()){std::ofstream(readyFile_)<<description_.at("scene_id").get<std::string>();}
    update();
    if(exitAfter_>0&&frames_>=exitAfter_){if(!snapshot_.empty())require(grab().save(QString::fromStdString(snapshot_)),"capture failed");close();}
  }
  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr states_,stats_;
  std::unique_ptr<RenderScene> rendering_;render::ScenePtr scene_;render::CameraPtr camera_;
  std::unique_ptr<render::OrbitViewController> orbit_;
  QTimer timer_;QPoint lastMouse_;QImage image_;J description_,statistics_;
  std::string epoch_,snapshot_,readyFile_;int exitAfter_=0,frames_=0,fpsFrames_=0;double fps_=0;
  Clock::time_point started_,fpsStart_,lastStats_;
};
}
#ifndef LITESIM_VIEWER_TEST
int main(int argc,char **argv){
  QApplication::setAttribute(Qt::AA_UseHighDpiPixmaps);
  rclcpp::init(argc,argv);QApplication app(argc,argv);app.setApplicationName("LiteSim 3D");
  // Qt's SVG icon engine renders the embedded vector at the requested size/DPI.
  app.setWindowIcon(QIcon(":/litesim3d/icons/litesim3d.svg"));
  int result=0;try{auto node=std::make_shared<rclcpp::Node>("litesim_viewer");Viewer viewer(node);viewer.show();result=app.exec();}
  catch(const std::exception &e){std::cerr<<"viewer: "<<e.what()<<'\n';result=1;}
  if(rclcpp::ok())rclcpp::shutdown();return result;
}
#endif
