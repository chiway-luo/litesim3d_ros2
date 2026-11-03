// Real offscreen QWidget policy runs against test-only ROS and rendering
// boundaries. No DDS context, subscription or executor can be constructed.
#include "../src/render_scene.hpp"
#include "../src/ros_event_loop.hpp"
#ifndef LITESIM_UNIT_ROS_TRANSPORT
#error viewer_unit requires the test-only ROS transport boundary
#endif
#include <gtest/gtest.h>
#include <QTemporaryDir>
#include <sstream>
#ifdef LITESIM_VIEWER_UNIT_FONTCONFIG
#include <fontconfig/fontconfig.h>
#endif
namespace litesim3d_render {
struct ViewerClock {
  using time_point=std::chrono::steady_clock::time_point;
  inline static time_point value{};
  inline static std::chrono::steady_clock::duration step=std::chrono::milliseconds(10);
  static time_point now(){auto result=value;value+=step;return result;}
};
}
#define LITESIM_VIEWER_TEST
#define private public
#define protected public
#define Clock ViewerClock
#include "../src/viewer.cpp"
#undef Clock
#undef protected
#undef private
namespace {
namespace backend=ignition::rendering;
class ViewerUnits:public testing::Test {
protected:
  void SetUp()override{
    rclcpp::unit::reset();context=std::make_shared<rclcpp::Context>();context->init(0,nullptr);
    backend::unit::reset();ViewerClock::value={};ViewerClock::step=std::chrono::milliseconds(10);
    previous=qgetenv("IGN_RENDERING_RESOURCE_PATH");had=qEnvironmentVariableIsSet("IGN_RENDERING_RESOURCE_PATH");
    ASSERT_TRUE(temp.isValid());auto root=temp.path()+"/resources";
    ASSERT_TRUE(QDir().mkpath(root+"/ogre2/media/materials/scripts"));
    QFile f(root+"/ogre2/media/materials/scripts/depth_camera.material");ASSERT_TRUE(f.open(QIODevice::WriteOnly));f.write("unit");f.close();
    qputenv("IGN_RENDERING_RESOURCE_PATH",root.toUtf8());
    description={{"schema_version",1},{"scene_id","scene"},{"resources",J::object()},
                 {"lights",J::array()},{"visuals",J::array()},{"initial_link_poses",J::object()}};
  }
  void TearDown()override{
    viewer.reset();node.reset();backend::test_engine.current.reset();
    rclcpp::unit::reset();
    std::weak_ptr<rclcpp::Context> released=context;
    if(context->is_valid())context->shutdown("unit teardown");context.reset();
    EXPECT_TRUE(released.expired());
    if(had)qputenv("IGN_RENDERING_RESOURCE_PATH",previous);else qunsetenv("IGN_RENDERING_RESOURCE_PATH");
  }
  void create(std::vector<rclcpp::Parameter> parameters={}){
    auto file=temp.filePath("scene.json").toStdString();std::ofstream(file)<<description.dump();
    parameters.emplace_back("scene_description",file);rclcpp::NodeOptions options;options.context(context).parameter_overrides(parameters);
    node=std::make_shared<rclcpp::Node>("viewer_unit_"+std::to_string(++serial),options);viewer=std::make_unique<Viewer>(node);
    viewer->resize(480,320);viewer->timer_.stop();
  }
  void deliver(const rclcpp::Subscription<std_msgs::msg::String>::SharedPtr&sub,J data){
    auto message=std::make_shared<std_msgs::msg::String>();message->data=data.dump();
    std::shared_ptr<void> payload=message;sub->handle_message(payload,rclcpp::MessageInfo{});
  }
  J state(std::string run="a",std::string scene="scene"){return {{"scene_id",scene},{"run_id",run},{"link_poses",J::object()}};}
  J stats(std::string run="a"){return {{"run_id",run},{"actual_real_time_factor",.5},{"target_real_time_factor",1},{"stamp_ns",1000000000},{"state","running"}};}
  QImage paint(){
    QImage image(viewer->size(),QImage::Format_ARGB32);image.fill(Qt::transparent);viewer->render(&image);return image;
  }
  QTemporaryDir temp;QByteArray previous;bool had=false;J description;
  std::shared_ptr<rclcpp::Context>context;
  std::shared_ptr<rclcpp::Node>node;std::unique_ptr<Viewer>viewer;inline static unsigned serial=0;
};
TEST_F(ViewerUnits,DefaultAndConfiguredViewPosesSetCameraAndOrbitTarget){
  create();EXPECT_EQ(viewer->cameraPose(),math::Pose3d(-4,-6,4.2,0,.47,.98));EXPECT_EQ(viewer->orbit_->Target(),math::Vector3d(1,0,.4));
  viewer.reset();node.reset();
  for(const auto&p:std::vector<J>{J::array({1,2,3,0,0,0,1}),J::array({1,2,3,0,std::sin(.3),0,std::cos(.3)}),J::array({1,2,-1,0,std::sin(.3),0,std::cos(.3)})}){
    description["view_pose"]=p;create();const auto expected=pose(p);EXPECT_EQ(viewer->cameraPose(),expected);
    const auto forward=expected.Rot().RotateVector(math::Vector3d::UnitX);
    double distance=expected.Pos().Z()>0&&forward.Z()<-1e-6?expected.Pos().Z()/-forward.Z():10;
    EXPECT_EQ(viewer->orbit_->Target(),expected.Pos()+forward*distance);viewer.reset();node.reset();
  }
}
TEST_F(ViewerUnits,StateAndStatisticsRejectForeignSceneAndStaleEpoch){
  create();deliver(viewer->stats_,stats("unbound"));EXPECT_EQ(viewer->statistics_["run_id"],"unbound");
  deliver(viewer->states_,state("a","foreign"));EXPECT_TRUE(viewer->epoch_.empty());
  deliver(viewer->states_,state());deliver(viewer->stats_,stats());EXPECT_EQ(viewer->statistics_["run_id"],"a");
  deliver(viewer->stats_,stats("stale"));EXPECT_EQ(viewer->statistics_["run_id"],"a");
  deliver(viewer->states_,state("b"));EXPECT_TRUE(viewer->statistics_.is_null());EXPECT_EQ(viewer->epoch_,"b");
  deliver(viewer->states_,state("b"));EXPECT_EQ(viewer->epoch_,"b");
}
TEST_F(ViewerUnits,MouseAndWheelDispatchOnlyCameraControls){
  create();QMouseEvent press(QEvent::MouseButtonPress,QPointF(10,10),Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);
  viewer->mousePressEvent(&press);EXPECT_EQ(viewer->lastMouse_,QPoint(10,10));
  QMouseEvent pan(QEvent::MouseMove,QPointF(15,18),Qt::NoButton,Qt::MiddleButton,Qt::ShiftModifier);
  viewer->mouseMoveEvent(&pan);EXPECT_EQ(viewer->orbit_->pan,math::Vector2d(5,8));
  QMouseEvent orbit(QEvent::MouseMove,QPointF(18,20),Qt::NoButton,Qt::LeftButton,Qt::NoModifier);
  viewer->mouseMoveEvent(&orbit);EXPECT_EQ(viewer->orbit_->orbit,math::Vector2d(3,2));
  QMouseEvent none(QEvent::MouseMove,QPointF(20,21),Qt::NoButton,Qt::NoButton,Qt::ShiftModifier);
  viewer->mouseMoveEvent(&none);EXPECT_EQ(viewer->orbit_->pan,math::Vector2d(5,8));
  const double distance=(viewer->camera_->WorldPosition()-viewer->orbit_->Target()).Length();
  QWheelEvent wheel(QPointF(1,1),QPointF(1,1),QPoint(),QPoint(0,120),Qt::NoButton,Qt::NoModifier,Qt::NoScrollPhase,false);
  viewer->wheelEvent(&wheel);EXPECT_NEAR(viewer->orbit_->zoom,distance-distance*std::exp(-.12),1e-12);EXPECT_TRUE(wheel.isAccepted());
  EXPECT_TRUE(viewer->modelPoses().empty());
}
TEST_F(ViewerUnits,FramesResizeBuffersPublishReadinessAndCalculateFps){
  auto ready=temp.filePath("ready").toStdString();create({rclcpp::Parameter("ready_file",ready)});
  viewer->fpsStart_=ViewerClock::value-std::chrono::seconds(2);
  viewer->renderForTest();EXPECT_GT(viewer->fps_,0);EXPECT_EQ(viewer->image_.size(),QSize(480,284));
  EXPECT_EQ(viewer->image_.pixelColor(0,0),QColor(42,42,42));
  viewer->renderForTest();viewer->renderForTest();std::ifstream file(ready);std::string id;file>>id;EXPECT_EQ(id,"scene");
  EXPECT_EQ(viewer->scene_->pre,3u);EXPECT_EQ(viewer->scene_->post,3u);
  viewer->resize(480,330);viewer->renderForTest();EXPECT_EQ(viewer->camera_->height,294u);
  viewer->resize(500,330);viewer->renderForTest();EXPECT_EQ(viewer->camera_->width,500u);
}
TEST_F(ViewerUnits,StatusPaintChangesForWarmupTargetBudgetAndDisconnection){
  create();auto waiting=paint();viewer->statistics_=stats();auto running=paint();EXPECT_NE(waiting,running);
  viewer->statistics_["actual_real_time_factor"]=nullptr;viewer->statistics_["target_real_time_factor"]=0;
  viewer->statistics_["collision_budget_enabled"]=true;auto warmup=paint();EXPECT_NE(warmup,running);
  viewer->lastStats_=ViewerClock::value-std::chrono::seconds(3);auto disconnected=paint();EXPECT_NE(disconnected,warmup);
  viewer->frame();EXPECT_FALSE(paint().isNull());
}
TEST_F(ViewerUnits,SnapshotExitAndTimerFailureAreObservable){
  auto path=temp.filePath("snapshot.png").toStdString();create({
    rclcpp::Parameter("capture_path",path),rclcpp::Parameter("exit_after_frames",1)});
  viewer->show();viewer->frame();EXPECT_FALSE(viewer->isVisible());EXPECT_FALSE(QImage(QString::fromStdString(path)).isNull());
  viewer.reset();node.reset();create({rclcpp::Parameter("exit_after_frames",1)});viewer->show();viewer->frame();EXPECT_FALSE(viewer->isVisible());
  viewer.reset();node.reset();create({rclcpp::Parameter("capture_path",temp.path().toStdString()),rclcpp::Parameter("exit_after_frames",1)});
  std::ostringstream error;auto*old=std::cerr.rdbuf(error.rdbuf());
  EXPECT_TRUE(QMetaObject::invokeMethod(&viewer->timer_,"timeout",Qt::DirectConnection));std::cerr.rdbuf(old);
  EXPECT_NE(error.str().find("capture failed"),std::string::npos);
}
TEST_F(ViewerUnits,DescriptionErrorsFailWithoutStartingRenderer){
  rclcpp::NodeOptions options;options.context(context).parameter_overrides({rclcpp::Parameter("scene_description","/missing/unit-scene")});
  node=std::make_shared<rclcpp::Node>("viewer_missing",options);EXPECT_THROW(Viewer value(node),std::runtime_error);node.reset();
  ViewerClock::step=std::chrono::seconds(6);node=std::make_shared<rclcpp::Node>("viewer_timeout",rclcpp::NodeOptions().context(context));
  EXPECT_THROW(Viewer value(node),std::runtime_error);EXPECT_EQ(backend::unit::destroyed,0);
}
TEST_F(ViewerUnits,ShutdownTimerClosesWithoutCapturingAnotherFrame){
  create();viewer->show();context->shutdown("unit shutdown");
  EXPECT_TRUE(QMetaObject::invokeMethod(&viewer->timer_,"timeout",Qt::DirectConnection));
  EXPECT_FALSE(viewer->isVisible());EXPECT_EQ(viewer->frames_,0);
}
}
int main(int argc,char**argv){
  qputenv("QT_QPA_PLATFORM","offscreen");QTemporaryDir runtime;qputenv("XDG_RUNTIME_DIR",runtime.path().toUtf8());int result;
  {QApplication app(argc,argv);testing::InitGoogleTest(&argc,argv);result=RUN_ALL_TESTS();}
#ifdef LITESIM_VIEWER_UNIT_FONTCONFIG
  FcFini();
#endif
  return result;
}
