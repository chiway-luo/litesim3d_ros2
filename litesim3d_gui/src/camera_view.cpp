#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "ros_event_loop.hpp"
#include <std_msgs/msg/string.hpp>
#include <nlohmann/json.hpp>
#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTimer>
#include <QImage>
#include <chrono>
#include <cmath>
#include <cstring>
#include <deque>
#include <map>
#include <algorithm>
#include <iostream>

class CameraView final:public QWidget {
  using Image=sensor_msgs::msg::Image;
  using Clock=std::chrono::steady_clock;
  struct Pair {Image::ConstSharedPtr color,depth;};
public:
  explicit CameraView(rclcpp::Node::SharedPtr node):node_(std::move(node)),
      executor_(litesim3d_render::ros_executor_options(node_->get_node_base_interface()->get_context())) {
    executor_.add_node(node_);
    topic_=node_->declare_parameter<std::string>("topic","camera/front");
    type_=node_->declare_parameter<std::string>("type","rgb");
    rate_=node_->declare_parameter<double>("rate_hz",15);
    near_=node_->declare_parameter<double>("near_clip_m",.1);far_=node_->declare_parameter<double>("far_clip_m",20);
    capture_=node_->declare_parameter<std::string>("capture_path","");
    exitAfter_=node_->declare_parameter<int>("exit_after_frames",0);
    if((type_!="rgb"&&type_!="depth"&&type_!="rgbd")||!(far_>near_&&near_>0))throw std::runtime_error("invalid camera view settings");
    setWindowTitle(QString::fromStdString("LiteSim Camera — "+topic_));resize(type_=="rgbd"?1100:640,500);
    setMinimumSize(480,280);setFocusPolicy(Qt::NoFocus);setContextMenuPolicy(Qt::NoContextMenu);
    auto subscribe=[&](const std::string &channel) {
      return node_->create_subscription<Image>(topic_+"/"+channel+"/image_raw",rclcpp::QoS(5).best_effort(),
        [this,channel](Image::ConstSharedPtr image) {
          const int64_t stamp=int64_t(image->header.stamp.sec)*1000000000+image->header.stamp.nanosec;
          auto &pair=pending_[stamp];if(channel=="color")pair.color=image;else pair.depth=image;
          if((type_=="depth"||pair.color)&&(type_=="rgb"||pair.depth)) {
            if(pair.color)color_=decode(*pair.color,false);
            if(pair.depth)depth_=decode(*pair.depth,true);
            width_=image->width;height_=image->height;stamp_=stamp;
            if(!samples_.empty()&&stamp<=samples_.back().second)samples_.clear();
            samples_.push_back({Clock::now(),stamp});while(samples_.size()>30)samples_.pop_front();
            ++frames_;pending_.erase(pending_.begin(),pending_.upper_bound(stamp));update();
          }
          while(pending_.size()>5)pending_.erase(pending_.begin());
        });
    };
    if(type_!="depth")colorSub_=subscribe("color");if(type_!="rgb")depthSub_=subscribe("depth");
    stats_=node_->create_subscription<std_msgs::msg::String>("/sim/stats",rclcpp::QoS(1).best_effort(),
      [this](std_msgs::msg::String::ConstSharedPtr msg) {state_=nlohmann::json::parse(msg->data).value("state","");});
    connect(&timer_,&QTimer::timeout,[this] {
      try {
        if(!litesim3d_render::process_ros_events(node_->get_node_base_interface()->get_context(),
            [this]{executor_.spin_some();})){close();return;}
        update();
        if(exitAfter_>0&&frames_>=exitAfter_){if(!capture_.empty()&&!grab().save(QString::fromStdString(capture_)))throw std::runtime_error("preview capture failed");close();}
      }catch(const std::exception &e){RCLCPP_ERROR(node_->get_logger(),"%s",e.what());QApplication::exit(1);}
    });timer_.start(30);
  }
protected:
  void paintEvent(QPaintEvent *) override {
    QPainter p(this);p.fillRect(rect(),QColor(19,23,31));p.setPen(QColor(224,230,240));
    const int panels=type_=="rgbd"?2:1,span=width()/panels;
    auto draw=[&](const QImage &image,int panel,const QString &title) {
      p.drawText(12+panel*span,25,title);
      const QRect area(panel*span+8,40,span-16,height()-110);
      if(!image.isNull()){auto size=image.size();size.scale(area.size(),Qt::KeepAspectRatio);p.drawImage(QRect(area.center()-QPoint(size.width()/2,size.height()/2),size),image);}
      else p.drawText(area,Qt::AlignCenter,"Waiting for camera data");
    };
    if(type_!="depth")draw(color_,0,"RGB  ·  rgb8");
    if(type_!="rgb")draw(depth_,type_=="rgbd"?1:0,QString("Depth  ·  32FC1 metres  ·  %1–%2 m  ·  invalid = black").arg(near_).arg(far_));
    double wall=0,sim=0;
    if(samples_.size()>1){double dt=std::chrono::duration<double>(samples_.back().first-samples_.front().first).count();
      if(dt>0)wall=(samples_.size()-1)/dt;double ds=(samples_.back().second-samples_.front().second)*1e-9;if(ds>0)sim=(samples_.size()-1)/ds;}
    bool stale=samples_.empty()||Clock::now()-samples_.back().first>std::chrono::seconds(2);
    p.drawText(12,height()-40,QString("%1 × %2   Configured %3 Hz   Received: sim %4 Hz / wall %5 FPS")
      .arg(width_).arg(height_).arg(rate_).arg(sim,0,'f',1).arg(stale?0:wall,0,'f',1));
    p.drawText(12,height()-16,QString("Stamp %1 s   %2%3").arg(stamp_*1e-9,0,'f',3)
      .arg(QString::fromStdString(state_)).arg(stale&&state_!="paused"?"   Waiting / disconnected":""));
  }
private:
  QImage decode(const Image &image,bool depth) {
    const unsigned bytes=depth?4:3;
    if(image.width<1||image.height<1||image.width>4096||image.height>4096||
       image.step<image.width*bytes||image.data.size()<size_t(image.step)*image.height||
       image.encoding!=(depth?"32FC1":"rgb8"))throw std::runtime_error("unsupported camera image");
    if(!depth)return QImage(image.data.data(),image.width,image.height,image.step,QImage::Format_RGB888).copy();
    QImage result(image.width,image.height,QImage::Format_RGB32);
    for(unsigned y=0;y<image.height;++y)for(unsigned x=0;x<image.width;++x) {
      const auto *b=image.data.data()+y*image.step+x*4;uint32_t bits=0;
      for(unsigned k=0;k<4;++k)bits|=uint32_t(b[k])<<(8*(image.is_bigendian?3-k:k));
      float z;std::memcpy(&z,&bits,4);
      result.setPixelColor(x,y,std::isfinite(z)&&z>0?QColor::fromHsvF(.67*std::clamp((z-near_)/(far_-near_),0.0,1.0),1,1):QColor(0,0,0));
    }
    return result;
  }
  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<Image>::SharedPtr colorSub_,depthSub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stats_;
  QTimer timer_;QImage color_,depth_;std::map<int64_t,Pair> pending_;
  std::deque<std::pair<Clock::time_point,int64_t>> samples_;
  std::string topic_,type_,state_,capture_;double rate_,near_,far_;int exitAfter_=0,frames_=0;
  unsigned width_=0,height_=0;int64_t stamp_=0;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);QApplication app(argc,argv);int result=0;
  try{auto node=std::make_shared<rclcpp::Node>("litesim_camera_view");CameraView view(node);view.show();result=app.exec();}
  catch(const std::exception &e){std::cerr<<e.what()<<'\n';result=1;}
  if(rclcpp::ok())rclcpp::shutdown();return result;
}
