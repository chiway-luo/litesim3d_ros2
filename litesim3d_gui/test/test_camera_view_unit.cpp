// Pure QWidget/model units on Qt's official in-memory offscreen platform.
// Image/status callbacks are dispatched directly. No publisher peer, camera
// renderer, display server, ROS executor loop or simulated physics runs.
#define LITESIM_CAMERA_VIEW_TEST
#include "../src/camera_view.cpp"
#include <gtest/gtest.h>
#include <QTemporaryDir>
#include <QFileInfo>

struct CameraViewUnitAccess {
  static void stop(CameraView &v){v.timer_.stop();}
  static void tick(CameraView &v){v.tick();}
  static void signal(CameraView &v){QMetaObject::invokeMethod(&v.timer_,"timeout",Qt::DirectConnection);}
  static unsigned frames(const CameraView &v){return v.frames_;}
  static size_t pending(const CameraView &v){return v.pending_.size();}
  static int64_t oldest(const CameraView &v){return v.pending_.begin()->first;}
  static size_t samples(const CameraView &v){return v.samples_.size();}
  static int64_t stamp(const CameraView &v){return v.stamp_;}
  static std::string state(const CameraView &v){return v.state_;}
  static bool color(const CameraView &v){return bool(v.colorSub_);}
  static bool depth(const CameraView &v){return bool(v.depthSub_);}
  static const QImage &pixels(const CameraView &v,bool depth){return depth?v.depth_:v.color_;}
  static auto status(const CameraView &v){return v.statusText();}
  static void image(CameraView &v,bool depth,const sensor_msgs::msg::Image &message){
    auto subscription=depth?v.depthSub_:v.colorSub_;
    ASSERT_NE(subscription,nullptr);
    std::shared_ptr<void> erased=std::make_shared<sensor_msgs::msg::Image>(message);
    subscription->handle_message(erased,rclcpp::MessageInfo{});
  }
  static void stats(CameraView &v,const std::string &text){
    auto message=std::make_shared<std_msgs::msg::String>();message->data=text;
    std::shared_ptr<void> erased=message;v.stats_->handle_message(erased,rclcpp::MessageInfo{});
  }
  static void rates(CameraView &v,bool stale,bool equal,bool reverse=false){
    const auto last=CameraView::Clock::now()-std::chrono::seconds(stale?3:0);
    v.samples_={{last-(equal?std::chrono::seconds(0):std::chrono::seconds(1)),1000000000},
                {last,reverse?0:equal?1000000000:2000000000}};
    v.width_=2;v.height_=2;v.stamp_=2000000000;
  }
};
namespace {
using Access=CameraViewUnitAccess;
using Image=sensor_msgs::msg::Image;
class CameraViewUnits:public testing::Test {
  struct Context: rclcpp::Context {void finalize(){clean_up();}};
protected:
  std::shared_ptr<Context> context;
  std::vector<rclcpp::Node::SharedPtr> nodes;
  unsigned sequence=0;
  void SetUp() override {context=std::make_shared<Context>();context->init(0,nullptr);}
  void TearDown() override {
    nodes.clear();context->shutdown("unit complete");context->finalize();
    std::weak_ptr<Context> weak=context;context.reset();EXPECT_TRUE(weak.expired());
  }
  std::unique_ptr<CameraView> view(const std::string &type="rgb",
      std::vector<rclcpp::Parameter> parameters={},CameraView::Hooks hooks={}){
    parameters.emplace_back("type",type);
    auto node=std::make_shared<rclcpp::Node>("camera_view_unit_"+std::to_string(sequence++),
        rclcpp::NodeOptions().context(context).parameter_overrides(parameters));
    nodes.push_back(node);
    auto result=std::make_unique<CameraView>(node,std::move(hooks));Access::stop(*result);return result;
  }
  static Image image(bool depth=false,int64_t stamp=1000000000){
    Image result;result.header.stamp.sec=stamp/1000000000;result.header.stamp.nanosec=stamp%1000000000;
    result.width=result.height=2;result.step=depth?8:6;result.encoding=depth?"32FC1":"rgb8";
    result.data.resize(result.step*result.height);
    for(unsigned i=0;i<4;++i){
      if(depth){float z=1;uint32_t bits;std::memcpy(&bits,&z,4);for(unsigned k=0;k<4;++k)result.data[i*4+k]=uint8_t(bits>>(8*k));}
      else{result.data[i*3]=0;result.data[i*3+1]=255;result.data[i*3+2]=0;}
    }
    return result;
  }
  static QImage paint(CameraView &view){view.show();QApplication::processEvents();return view.grab().toImage();}
};

TEST_F(CameraViewUnits, ParameterValidationAndChannelSelectionRemainExplicit){
  EXPECT_THROW(view("stereo"),std::runtime_error);
  EXPECT_THROW(view("rgb",{rclcpp::Parameter("near_clip_m",0.)}),std::runtime_error);
  EXPECT_THROW(view("rgb",{rclcpp::Parameter("far_clip_m",.1)}),std::runtime_error);
  for(const auto &type:{"rgb","depth","rgbd"}){
    auto v=view(type);EXPECT_EQ(Access::color(*v),std::string(type)!="depth");
    EXPECT_EQ(Access::depth(*v),std::string(type)!="rgb");
    EXPECT_EQ(v->width(),std::string(type)=="rgbd"?1100:640);
    EXPECT_EQ(v->height(),500);EXPECT_TRUE(v->windowTitle().contains("camera/front"));
    auto rendered=paint(*v);EXPECT_FALSE(rendered.isNull());
    EXPECT_EQ(rendered.pixelColor(2,2),QColor(19,23,31));
  }
}

TEST_F(CameraViewUnits, RgbAndDepthDispatchDecodeOwnedPixelsWithoutAnyPeer){
  for(bool depth:{false,true}){
    auto v=view(depth?"depth":"rgb");auto message=image(depth);
    Access::image(*v,depth,message);EXPECT_EQ(Access::frames(*v),1u);EXPECT_EQ(Access::pending(*v),0u);
    EXPECT_EQ(Access::stamp(*v),1000000000);EXPECT_EQ(Access::samples(*v),1u);
    const auto expected=CameraView::decode(message,depth,.1,20);
    EXPECT_EQ(Access::pixels(*v,depth),expected);
    message.data.assign(message.data.size(),0);EXPECT_EQ(Access::pixels(*v,depth),expected);
    auto rendered=paint(*v);EXPECT_EQ(rendered.pixelColor(rendered.width()/2,rendered.height()/2),expected.pixelColor(0,0));
    Access::stats(*v,R"({"state":"paused"})");EXPECT_EQ(Access::state(*v),"paused");
    Access::stats(*v,"{}");EXPECT_TRUE(Access::state(*v).empty());
    EXPECT_THROW(Access::stats(*v,"{"),nlohmann::json::parse_error);
    Access::signal(*v);EXPECT_EQ(Access::frames(*v),1u);
  }
}

TEST_F(CameraViewUnits, RgbdPairsExactStampsAndBoundsIncompleteFrames){
  auto v=view("rgbd");
  Access::image(*v,false,image(false,1));EXPECT_EQ(Access::frames(*v),0u);
  Access::image(*v,true,image(true,2));EXPECT_EQ(Access::pending(*v),2u);
  Access::image(*v,false,image(false,2));EXPECT_EQ(Access::frames(*v),1u);
  EXPECT_EQ(Access::pending(*v),0u);EXPECT_EQ(Access::stamp(*v),2);
  auto rendered=paint(*v);
  EXPECT_EQ(rendered.pixelColor(rendered.width()/4,rendered.height()/2),QColor(0,255,0));
  EXPECT_EQ(rendered.pixelColor(3*rendered.width()/4,rendered.height()/2),CameraView::decode(image(true),true,.1,20).pixelColor(0,0));
  for(int64_t stamp=3;stamp<=9;++stamp)Access::image(*v,false,image(false,stamp));
  EXPECT_EQ(Access::pending(*v),5u);EXPECT_EQ(Access::oldest(*v),5);
  Access::image(*v,true,image(true,9));EXPECT_EQ(Access::pending(*v),0u);EXPECT_EQ(Access::frames(*v),2u);
  // Depth-first and color-first completion must both work.
  Access::image(*v,true,image(true,10));Access::image(*v,false,image(false,10));
  EXPECT_EQ(Access::frames(*v),3u);
}

TEST_F(CameraViewUnits, RepeatedOrRewoundTimestampsResetRateWindowAndHistoryIsBounded){
  auto v=view();
  for(int64_t stamp=1;stamp<=40;++stamp)Access::image(*v,false,image(false,stamp*1000000000));
  EXPECT_EQ(Access::frames(*v),40u);EXPECT_EQ(Access::samples(*v),30u);
  Access::image(*v,false,image(false,40000000000));EXPECT_EQ(Access::samples(*v),1u);
  Access::image(*v,false,image(false,1000000000));EXPECT_EQ(Access::samples(*v),1u);
  auto malformed=image();malformed.width=0;
  EXPECT_THROW(Access::image(*v,false,malformed),std::runtime_error);
  EXPECT_EQ(Access::frames(*v),42u);
}

TEST_F(CameraViewUnits, StatusDistinguishesActiveStalePausedAndNonIncreasingRateSamples){
  auto v=view();auto text=Access::status(*v);
  EXPECT_TRUE(text.first.contains("sim 0.0 Hz / wall 0.0 FPS"));
  EXPECT_TRUE(text.second.contains("Waiting / disconnected"));
  Access::rates(*v,false,false);text=Access::status(*v);
  EXPECT_TRUE(text.first.contains("sim 1.0 Hz / wall 1.0 FPS"));
  EXPECT_FALSE(text.second.contains("disconnected"));EXPECT_TRUE(text.second.contains("2.000 s"));
  Access::rates(*v,true,false);Access::stats(*v,R"({"state":"running"})");text=Access::status(*v);
  EXPECT_TRUE(text.first.contains("wall 0.0 FPS"));EXPECT_TRUE(text.second.contains("disconnected"));
  Access::stats(*v,R"({"state":"paused"})");EXPECT_FALSE(Access::status(*v).second.contains("disconnected"));
  Access::rates(*v,false,true);EXPECT_TRUE(Access::status(*v).first.contains("sim 0.0 Hz / wall 0.0 FPS"));
  Access::rates(*v,false,false,true);EXPECT_TRUE(Access::status(*v).first.contains("sim 0.0 Hz"));
}

TEST_F(CameraViewUnits, TickClosesOnShutdownAndDoesNotHideUnexpectedWorkErrors){
  int exits=0;auto error=view("rgb",{}, {[]{throw std::runtime_error("unit spin failure");},[&](int code){EXPECT_EQ(code,1);++exits;}});
  error->show();Access::tick(*error);EXPECT_EQ(exits,1);EXPECT_TRUE(error->isVisible());error.reset();
  auto stopped=view();stopped->show();context->shutdown("stopped before tick");Access::tick(*stopped);
  EXPECT_FALSE(stopped->isVisible());
}

TEST_F(CameraViewUnits, ShutdownRacingWithWorkClosesWithoutReportingAnError){
  int exits=0;
  auto v=view("rgb",{}, {[&]{context->shutdown("between guard and work");throw std::runtime_error("shutdown race");},[&](int){++exits;}});
  v->show();Access::tick(*v);EXPECT_EQ(exits,0);EXPECT_FALSE(v->isVisible());
}

TEST_F(CameraViewUnits, CaptureExitThresholdKeepsExactSnapshotOrReportsSaveFailure){
  QTemporaryDir directory;ASSERT_TRUE(directory.isValid());
  const auto path=directory.filePath("preview.png");int exits=0;
  auto v=view("rgb",{rclcpp::Parameter("capture_path",path.toStdString()),rclcpp::Parameter("exit_after_frames",1)},
      {{},[&](int){++exits;}});
  v->show();Access::tick(*v);EXPECT_TRUE(v->isVisible());EXPECT_FALSE(QFileInfo::exists(path));
  Access::image(*v,false,image());Access::tick(*v);EXPECT_FALSE(v->isVisible());EXPECT_EQ(exits,0);
  QImage saved(path);ASSERT_FALSE(saved.isNull());EXPECT_EQ(saved.pixelColor(saved.width()/2,saved.height()/2),QColor(0,255,0));
  v.reset();
  auto empty=view("rgb",{rclcpp::Parameter("exit_after_frames",1)});empty->show();
  Access::image(*empty,false,image());Access::tick(*empty);EXPECT_FALSE(empty->isVisible());empty.reset();
  auto bad=view("rgb",{rclcpp::Parameter("capture_path",directory.path().toStdString()),rclcpp::Parameter("exit_after_frames",1)},
      {{},[&](int code){EXPECT_EQ(code,1);++exits;}});
  bad->show();Access::image(*bad,false,image());Access::tick(*bad);EXPECT_EQ(exits,1);EXPECT_TRUE(bad->isVisible());
}
}  // namespace
int main(int argc,char **argv){
  qputenv("QT_QPA_PLATFORM","offscreen");
  QTemporaryDir runtime;
  qputenv("XDG_RUNTIME_DIR",runtime.path().toUtf8());
  QApplication app(argc,argv);testing::InitGoogleTest(&argc,argv);return RUN_ALL_TESTS();
}
