// Business-orchestration units: production SimNode consumes an inert backend,
// deterministic time/context/counts and recording publication/render adapters.
// No real Simulator, physics advancement, DDS peer, process or renderer is used.
#define main litesim_sim_node_process_entrypoint
#include "../src/sim_node.cpp"
#undef main
#include <gtest/gtest.h>
#include <any>
#include <fstream>

namespace litesim3d_ros { struct JointControlUnitAccess {
 static void deliver(JointControl &control, const JointControl::Command &message) {
  std::shared_ptr<void> p=std::make_shared<JointControl::Command>(message);control.command_->handle_message(p,rclcpp::MessageInfo{});
 }
}; }
struct SimNodeUnitAccess {
  static void update(SimNode &n) { n.update(); }
  static void events(SimNode &n, const std::vector<litesim3d::Event> &v) { n.publish(v); }
  static void state(SimNode &n, const litesim3d::Snapshot &v) { n.publishState(v); }
  static void status(SimNode &n) { n.publishStatus(); }
  static void periodic(SimNode &n) { n.publishPeriodicStatus(); }
  static void scene(SimNode &n, bool initial=true) { n.publishScene(initial); }
  static void sceneState(SimNode &n) { n.publishSceneState(); }
  static void cameras(SimNode &n, const litesim3d::Event &v) { n.publishCameras(v); }
  static void manifest(SimNode &n, const std::string &reason) { n.saveManifest(reason); }
  static void ready(SimNode &n, bool v) { n.ready_=v; }
  static bool ready(const SimNode &n) { return n.ready_; }
  static void counters(SimNode &n, uint64_t qos, uint64_t drops) { n.qos_conflicts_=qos;n.dropped_frames_=drops; }
  static void path(SimNode &n, size_t count) { n.trajectory_.poses.resize(count); }
  static void timing(SimNode &n,size_t count) { n.compute_ms_.assign(count,1);n.bridge_ms_.assign(count,2); }
  static SimNodeHooks &hooks(SimNode &n) { return n.hooks_; }
  static auto now(SimNode &n){return n.wallNow();}
  static bool ok(SimNode &n){return n.contextOk();}
  static bool cameraReady(SimNode &n){return n.cameraReady();}
  static auto publishers(SimNode &n,const std::string&t){return n.publisherCount(t);}
  template<class Publisher,class Message>static void emit(SimNode &n,const Publisher&p,const Message&m){n.emit(p,m);}
  template<class Publisher>static size_t subscriptions(SimNode &n,const Publisher&p){return n.subscriptionCount(p);}
  template<class Broadcaster>static void transform(SimNode &n,Broadcaster&b,const geometry_msgs::msg::TransformStamped&m){n.sendTransform(b,m);}
  static void qos(SimNode &n){std::shared_ptr<void> data=std::make_shared<rclcpp::QOSOfferedIncompatibleQoSInfo>();n.clouds_.begin()->second->get_event_handlers().at(RCL_PUBLISHER_OFFERED_INCOMPATIBLE_QOS)->execute(data);}
  static void joint(SimNode &n,const litesim3d_interfaces::msg::JointCommand &v){litesim3d_ros::JointControlUnitAccess::deliver(*n.joint_control_,v);}
  static auto takeBackend(SimNode &n){return std::move(n.sim_);}
  static void restoreBackend(SimNode &n,std::unique_ptr<litesim3d_ros::SimulatorBackend> backend){n.sim_=std::move(backend);}
  static void timer(SimNode &n) { n.timer_->execute_callback(); }
  static void statusTimer(SimNode &n) { n.status_timer_->execute_callback(); }
  static void sceneTimer(SimNode &n) { n.scene_timer_->execute_callback(); }
  template<class Message> static void command(SimNode &n,const Message &v) {
    std::shared_ptr<void> p=std::make_shared<Message>(v);
    if constexpr(std::is_same_v<Message,geometry_msgs::msg::Twist>) n.twist_->handle_message(p,rclcpp::MessageInfo{});
    else n.stamped_->handle_message(p,rclcpp::MessageInfo{});
  }
  static void tf(SimNode &n, const tf2_msgs::msg::TFMessage &v,unsigned id) {
    std::shared_ptr<void> p=std::make_shared<tf2_msgs::msg::TFMessage>(v);
    rmw_message_info_t info{};info.publisher_gid.data[0]=uint8_t(id);
    n.tf_watch_->handle_message(p,rclcpp::MessageInfo(info));
  }
  template<class Service> static void service(SimNode &n, typename Service::Request::SharedPtr v) {
    auto header=std::make_shared<rmw_request_id_t>();
    if constexpr(std::is_same_v<Service,std_srvs::srv::SetBool>) n.pause_service_->handle_request(header,v);
    else if constexpr(std::is_same_v<Service,litesim3d_interfaces::srv::Step>) n.step_service_->handle_request(header,v);
    else n.pose_service_->handle_request(header,v);
  }

};
namespace {
using namespace litesim3d;
using Render=litesim3d_interfaces::srv::RenderCameras;
struct FakeSimulation {
  Config cfg; Model mdl; Scene sc; Snapshot snap;
  State current=State::Ready;
  std::string run_id="unit-simulation", error;
  std::vector<Command> commands;
  std::vector<JointCommand> joint_commands;
  std::vector<Event> next;
  std::vector<Contact> contact_values;
  uint64_t contact_count=0, ticks=0;
  bool throw_pause=false,throw_step=false,throw_tick=false,throw_pose=false,throw_camera=false;
  std::function<void()> on_tick;
  unsigned exits=0,processed=0;
  FakeSimulation(Config c,Model m,Scene s):cfg(std::move(c)),mdl(std::move(m)),sc(std::move(s)) {
    snap.world_base=cfg.initial_pose;snap.odom_base=cfg.initial_pose;snap.odom_root=Pose::Identity();
    snap.joints={{"arm",.2},{"lidar_fixed",0},{"imu_fixed",0},{"optical_fixed",0}};
    snap.joint_velocities={{"arm",.3}};
  }
  const Config &config()const{return cfg;} const Model &model()const{return mdl;}
  const Scene &scene()const{return sc;} const Snapshot &snapshot()const{return snap;}
  State state()const{return current;} const std::vector<Contact>&contacts()const{return contact_values;}
  uint64_t contactCount()const{return contact_count;}
  Json diagnostics()const{return {{"state",current==State::Fault?"FAULT":current==State::Paused?"PAUSED":"RUNNING"},
    {"last_error",error},{"command_timeout",timeout},{"rejected_commands",rejected}};}
  bool timeout=false;uint64_t rejected=0;
  bool command(const Command&v){commands.push_back(v);return true;}
  bool jointCommand(const JointCommand&v){joint_commands.push_back(v);return true;}
  void pause(bool v){if(throw_pause)throw std::runtime_error("unit pause failure");current=v?State::Paused:State::Running;}
  std::vector<Event> tick(){++ticks;if(on_tick)on_tick();if(throw_tick)throw std::runtime_error("unit tick failure");return next;}
  StepResult singleStep(const std::string &id,uint32_t count,std::vector<Event>&out){
    if(throw_step)throw std::runtime_error("unit step failure");out=next;return{true,0,id+":"+std::to_string(count),snap.stamp};}
  void setPose(const Pose&p){if(throw_pose)throw std::runtime_error("unit pose failure");snap.world_base=p;}
  void fault(const std::string &why){current=State::Fault;error=why;}
  void exit(){++exits;current=State::Exited;}
  Json manifest(const std::string &why)const{return{{"reason",why},{"run_id",run_id}};}
  void processCamera(size_t,plugins::ImageFrame &frame){++processed;if(throw_camera)throw std::runtime_error("unit camera failure");if(!frame.color.empty())frame.color[0]=77;}
};
struct Publication{std::string topic;std::any value;};
std::function<rcl_ret_t(void*)> response_callback;
rcl_ret_t responseSender(const rcl_service_t*,rmw_request_id_t*,void*value){return response_callback?response_callback(value):RCL_RET_OK;}
std::shared_ptr<Shape> shape(Shape::Kind kind,Vec3 size){auto s=std::make_shared<Shape>();s->kind=kind;s->size=size;return s;}
class SimNodeUnits:public ::testing::Test {
protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  void SetUp()override{
    cfg.real_time_factor=0;cfg.urdf="unit-model";cfg.base_link="base_link";
    mdl.root="base_link";mdl.xml="<robot name='unit'/>";
    mdl.links={"base_link","arm_link","lidar_link","imu_link","optical"};
    for(auto pair:std::vector<std::pair<std::string,std::string>>{{"arm","arm_link"},{"lidar_fixed","lidar_link"},{"imu_fixed","imu_link"},{"optical_fixed","optical"}}){
      Joint j;j.name=pair.first;j.parent="base_link";j.child=pair.second;j.type=pair.first=="arm"?"revolute":"fixed";j.axis=Vec3::UnitZ();mdl.joints[j.name]=j;mdl.order.push_back(j.name);
    }
    sc.visual.push_back({"box","",shape(Shape::Kind::Box,{1,2,3})});
    clock=std::chrono::steady_clock::time_point(std::chrono::seconds(100));
    dir=std::filesystem::temp_directory_path()/("sim-node-unit-"+std::to_string(++serial));std::filesystem::create_directories(dir);
    response_callback={};
  }
  void TearDown()override{node.reset();engine.reset();response_callback={};std::filesystem::remove_all(dir);}
  SimNodeHooks hooks(){
    SimNodeHooks h;
    h.load_config=[this](const std::string&,Scene&out){out=sc;return cfg;};
    h.load_model=[this](const std::filesystem::path&,const ModelOptions&){return mdl;};
    h.make_backend=[this](Config c,Model m,Scene s){engine=std::make_shared<FakeSimulation>(std::move(c),std::move(m),std::move(s));if(initialize)initialize(*engine);return std::make_unique<litesim3d_ros::SimulatorBackendAdapter<FakeSimulation>>(engine);};
    h.publish=[this](const std::string&t,std::any v){published.push_back({t,std::move(v)});};
    h.now=[this]{return clock;};h.ok=[this]{return context_ok;};h.shutdown=[this]{++shutdowns;};
    h.publishers=[this](const std::string&t){return t=="/clock"?clock_publishers:t=="sim/joint_commands"?joint_publishers:1U;};
    h.subscriptions=[](const std::string&){return 1U;};h.camera_ready=[this]{return camera_ready;};
    h.sender=responseSender;h.lidar_factory=serializer_factory;
    h.render=[this](Render::Request::SharedPtr request,double timeout){render_request=request;render_timeout=timeout;return render_response;};
    return h;
  }
  void create(std::vector<rclcpp::Parameter> parameters={}){rclcpp::NodeOptions options;options.parameter_overrides(parameters);node=std::make_shared<SimNode>(options,hooks());}
  template<class Message> const Message&last(const std::string&topic)const{
    for(auto it=published.rbegin();it!=published.rend();++it)if(it->topic==topic)return std::any_cast<const Message&>(it->value);
    throw std::logic_error("unit publication missing: "+topic);
  }
  size_t count(const std::string&topic)const{return std::count_if(published.begin(),published.end(),[&](const auto&p){return p.topic==topic;});}
  void sensors(bool camera=false){LidarConfig l;l.name="lidar";l.link="lidar_link";l.topic="points_raw";l.vertical_angles={0};l.columns=2;l.publish_range_boundary=true;cfg.lidars={l};ImuConfig i;i.link="imu_link";cfg.imus={i};if(camera){CameraConfig c;c.name="camera";c.link="optical";c.type="rgbd";c.width=2;c.height=1;cfg.cameras={c};}}
  std::string sceneFile(){auto p=dir/"scene.json";std::ofstream(p)<<Json{{"scene_id","unit-scene"}};return p.string();}
  Event event(){Event e;e.snapshot=engine->snap;e.snapshot.stamp=1000000007;e.snapshot.v=.4;e.snapshot.lateral=.5;e.snapshot.vertical=.6;e.snapshot.w=.7;e.snapshot.odom_v=.1;e.snapshot.odom_lateral=.2;e.snapshot.odom_vertical=.3;e.snapshot.odom_w=.4;e.odom=true;return e;}
  void cameraResponse(const Event&e){render_response=std::make_shared<Render::Response>();render_response->success=true;litesim3d_interfaces::msg::CameraFrame f;f.camera_name="camera";f.info.header.stamp=stamp(e.snapshot.stamp);f.info.header.frame_id="optical";f.info.width=2;f.info.height=1;for(auto entry:{std::pair<sensor_msgs::msg::Image*,bool>{&f.color,true},{&f.depth,false}}){auto&i=*entry.first;i.header=f.info.header;i.width=2;i.height=1;i.encoding=entry.second?"rgb8":"32FC1";i.step=entry.second?6:8;i.data.resize(i.step,11);}render_response->frames={f};}
  Config cfg;Model mdl;Scene sc;std::shared_ptr<FakeSimulation>engine;std::shared_ptr<SimNode>node;
  std::vector<Publication>published;std::filesystem::path dir;inline static unsigned serial=0;
  std::chrono::steady_clock::time_point clock;bool context_ok=true,camera_ready=true;unsigned shutdowns=0;size_t clock_publishers=1,joint_publishers=1;
  std::function<void(FakeSimulation&)>initialize;
  litesim3d_ros::LidarOutputs::PluginFactory serializer_factory;
  Render::Request::SharedPtr render_request;Render::Response::SharedPtr render_response;double render_timeout=0;
};
}
TEST_F(SimNodeUnits, ConstructorPublishesMetadataAndCommandsAreTranslatedWithoutPhysics){
  create({rclcpp::Parameter("frame_prefix","robot/")});
  EXPECT_EQ(last<std_msgs::msg::String>("/robot_description").data,mdl.xml);
  EXPECT_EQ(last<rosgraph_msgs::msg::Clock>("/clock").clock.nanosec,1U);
  EXPECT_EQ(last<sensor_msgs::msg::JointState>("/joint_states").name,std::vector<std::string>{"arm"});
  geometry_msgs::msg::Twist command;command.linear.x=1;command.linear.y=2;command.linear.z=3;command.angular.x=4;command.angular.y=5;command.angular.z=6;
  SimNodeUnitAccess::command(*node,command);ASSERT_EQ(engine->commands.size(),1U);const auto&c=engine->commands[0];EXPECT_EQ(c.v,1);EXPECT_EQ(c.lateral,2);EXPECT_EQ(c.vertical,3);EXPECT_EQ(c.roll_rate,4);EXPECT_EQ(c.pitch_rate,5);EXPECT_EQ(c.w,6);EXPECT_FALSE(c.stamp);EXPECT_TRUE(c.frame.empty());EXPECT_EQ(engine->ticks,0U);
}
TEST_F(SimNodeUnits, TimestampedCommandFramesAndStartupValidation){
  create({rclcpp::Parameter("cmd_mode","stamped"),rclcpp::Parameter("frame_prefix","robot/")});
  for(const char*frame:{"","robot/base_link","other"}){geometry_msgs::msg::TwistStamped m;m.header.frame_id=frame;m.header.stamp.sec=2;m.header.stamp.nanosec=9;SimNodeUnitAccess::command(*node,m);EXPECT_EQ(engine->commands.back().stamp,2000000009U);EXPECT_EQ(engine->commands.back().frame,std::string(frame)=="robot/base_link"?"base_link":frame);}
  node.reset();
  for(const auto&p:std::vector<rclcpp::Parameter>{rclcpp::Parameter("frame_prefix","bad"),rclcpp::Parameter("frame_prefix","/bad/"),rclcpp::Parameter("external_odom_tf",true),rclcpp::Parameter("startup_timeout",0.),rclcpp::Parameter("startup_timeout",NAN),rclcpp::Parameter("camera_render_timeout",0.),rclcpp::Parameter("camera_render_timeout",INFINITY),rclcpp::Parameter("camera_render_timeout",3601.),rclcpp::Parameter("initial_pose_override",std::vector<double>{1,2}),rclcpp::Parameter("cmd_mode","bad"),rclcpp::Parameter("test_drop_frames",-1),rclcpp::Parameter("test_drop_frames",101)})EXPECT_THROW(create({p}),std::runtime_error)<<p.get_name();
  create({rclcpp::Parameter("initial_pose_override",std::vector<double>{1,2,3,0,0,0,1}),rclcpp::Parameter("publish_odom_tf",false),rclcpp::Parameter("sensor_reliable",true)});EXPECT_EQ(engine->cfg.initial_pose.translation(),Vec3(1,2,3));
}
TEST_F(SimNodeUnits, UpdateReadinessPacingAndFaultsUseDeterministicClock){
  create({rclcpp::Parameter("viewer_ready_file",(dir/"ready").string()),rclcpp::Parameter("startup_timeout",1.)});
  SimNodeUnitAccess::timer(*node);EXPECT_EQ(engine->ticks,0U);clock+=std::chrono::seconds(2);SimNodeUnitAccess::update(*node);EXPECT_TRUE(node->failed());EXPECT_NE(engine->error.find("viewer readiness"),std::string::npos);
  node.reset();create({rclcpp::Parameter("start_paused",true)});SimNodeUnitAccess::update(*node);EXPECT_TRUE(SimNodeUnitAccess::ready(*node));EXPECT_EQ(engine->current,State::Paused);EXPECT_EQ(engine->ticks,1U);
  context_ok=false;SimNodeUnitAccess::update(*node);EXPECT_EQ(engine->ticks,1U);context_ok=true;
  engine->throw_tick=true;SimNodeUnitAccess::update(*node);EXPECT_TRUE(node->failed());EXPECT_EQ(engine->error,"unit tick failure");
  node.reset();cfg.real_time_factor=1;create();SimNodeUnitAccess::update(*node);EXPECT_EQ(engine->ticks,0U);clock+=std::chrono::milliseconds(100);SimNodeUnitAccess::update(*node);EXPECT_EQ(engine->ticks,8U);
  node.reset();clock_publishers=2;create();SimNodeUnitAccess::update(*node);EXPECT_TRUE(node->failed());EXPECT_EQ(engine->ticks,0U);
}
TEST_F(SimNodeUnits, EventSerializationPreservesSensorOdometryContactsAndBoundedWindows){
  sensors();create({rclcpp::Parameter("frame_prefix","r/")});auto e=event();
  ImuSample imu;imu.stamp=e.snapshot.stamp;imu.frame="imu_link";imu.topic="imu/data_raw";imu.acceleration={1,2,3};imu.angular_velocity={4,5,6};e.imus={imu};
  Cloud cloud;cloud.topic="points_raw";cloud.frame="lidar_link";cloud.stamp=e.snapshot.stamp;cloud.width=2;cloud.height=1;cloud.row_step=48;cloud.data.resize(48,7);cloud.range_boundary.resize(48,8);cloud.compute_ms=2;cloud.compute_end_ns=1;e.clouds={cloud};
  engine->contact_values={{e.snapshot.stamp,"arm_link","wall",-.01}};engine->contact_count=1;
  SimNodeUnitAccess::path(*node,2000);SimNodeUnitAccess::timing(*node,4096);SimNodeUnitAccess::events(*node,{e});
  const auto&o=last<nav_msgs::msg::Odometry>("/odom");EXPECT_EQ(o.header.frame_id,"r/odom");EXPECT_EQ(o.child_frame_id,"r/base_link");EXPECT_EQ(o.twist.twist.linear.y,.2);
  const auto&truth=last<nav_msgs::msg::Odometry>("/sim/ground_truth/robot_0/odom");EXPECT_EQ(truth.header.frame_id,"world");EXPECT_EQ(truth.twist.twist.linear.z,.6);
  EXPECT_EQ(last<nav_msgs::msg::Path>("/trajectory").poses.size(),1001U);
  EXPECT_EQ(last<sensor_msgs::msg::Imu>("/imu/data_raw").angular_velocity.z,6);
  const auto&points=last<sensor_msgs::msg::PointCloud2>("/points_raw");EXPECT_EQ(points.data,cloud.data);EXPECT_EQ(points.fields.size(),6U);EXPECT_EQ(points.fields[4].datatype,4);
  EXPECT_EQ(last<sensor_msgs::msg::PointCloud2>("/points_raw/range_boundary").data,cloud.range_boundary);
  EXPECT_EQ(last<litesim3d_interfaces::msg::Contact>("/sim/ground_truth/robot_0/contacts").robot_link,"r/arm_link");
  SimNodeUnitAccess::status(*node);const auto stats=Json::parse(last<std_msgs::msg::String>("/sim/stats").data);EXPECT_EQ(stats["run_id"],engine->run_id);
  node.reset();create({rclcpp::Parameter("test_drop_frames",1),rclcpp::Parameter("publish_odom_tf",false)});SimNodeUnitAccess::events(*node,{e});EXPECT_EQ(count("/points_raw"),1U); // first node's publication only
  e.clouds[0].range_boundary.clear();SimNodeUnitAccess::events(*node,{e});EXPECT_EQ(count("/points_raw"),2U);
}
TEST_F(SimNodeUnits, SceneKindsStateThrottlingAndManifestAreSerialized){
  sc.visual.push_back({"sphere","",shape(Shape::Kind::Sphere,{.4,0,0})});sc.visual.push_back({"cylinder","",shape(Shape::Kind::Cylinder,{.2,0,1})});auto mesh=shape(Shape::Kind::Mesh,{1,1,1});mesh->triangles={{{0,0,0},{1,0,0},{0,1,0}}};sc.visual.push_back({"mesh","",mesh});
  create({rclcpp::Parameter("scene_description",sceneFile()),rclcpp::Parameter("manifest_path",(dir/"manifest.json").string())});
  const auto&m=last<visualization_msgs::msg::MarkerArray>("/scene").markers;ASSERT_EQ(m.size(),4U);EXPECT_EQ(m[0].type,m[0].CUBE);EXPECT_EQ(m[1].type,m[1].SPHERE);EXPECT_EQ(m[2].type,m[2].CYLINDER);EXPECT_EQ(m[3].points.size(),3U);
  SimNodeUnitAccess::sceneTimer(*node);auto n=count("/sim/scene_state");SimNodeUnitAccess::sceneState(*node);EXPECT_EQ(count("/sim/scene_state"),n);clock+=std::chrono::milliseconds(34);SimNodeUnitAccess::sceneState(*node);EXPECT_EQ(count("/sim/scene_state"),n+1);
  SimNodeUnitAccess::manifest(*node,"unit");Json data;std::ifstream(dir/"manifest.json")>>data;EXPECT_EQ(data["reason"],"unit");EXPECT_EQ(data["scene_assets"]["scene_id"],"unit-scene");EXPECT_EQ(data["ros_distro"],"humble");
  node->finish();EXPECT_EQ(engine->exits,1U);engine->fault("fault");node->finish();EXPECT_EQ(engine->exits,1U);
}
TEST_F(SimNodeUnits, CameraRequestAndImageValidationAreIsolatedFromRenderer){
  sensors(true);create({rclcpp::Parameter("scene_description",sceneFile()),rclcpp::Parameter("frame_prefix","r/")});auto e=event();e.cameras={0};cameraResponse(e);SimNodeUnitAccess::cameras(*node,e);
  ASSERT_NE(render_request,nullptr);EXPECT_EQ(render_request->scene_id,"unit-scene");EXPECT_EQ(render_request->run_id,engine->run_id);EXPECT_EQ(render_request->camera_names,std::vector<std::string>{"camera"});EXPECT_EQ(render_timeout,30);
  EXPECT_EQ(last<sensor_msgs::msg::Image>("/camera/front/color/image_raw").data[0],77);EXPECT_EQ(last<sensor_msgs::msg::CameraInfo>("/camera/front/depth/camera_info").header.frame_id,"r/optical");EXPECT_EQ(engine->processed,1U);
  for(int defect=0;defect<11;++defect){cameraResponse(e);if(defect==0)render_response->success=false;if(defect==1)render_response->frames.clear();if(defect==2)render_response->frames[0].camera_name="wrong";if(defect==3)++render_response->frames[0].info.header.stamp.nanosec;if(defect==4)++render_response->frames[0].info.width;if(defect==5)++render_response->frames[0].info.height;if(defect==6)render_response->frames[0].info.header.frame_id="wrong";if(defect==7)render_response->frames[0].color.encoding="wrong";if(defect==8)++render_response->frames[0].color.width;if(defect==9)++render_response->frames[0].color.step;if(defect==10)render_response->frames[0].color.data.pop_back();EXPECT_THROW(SimNodeUnitAccess::cameras(*node,e),std::runtime_error)<<defect;}
  camera_ready=false;EXPECT_THROW(SimNodeUnitAccess::cameras(*node,e),std::runtime_error);camera_ready=true;e.cameras.clear();EXPECT_NO_THROW(SimNodeUnitAccess::cameras(*node,e));
  node.reset();create({rclcpp::Parameter("scene_description",sceneFile())});camera_ready=false;clock+=std::chrono::seconds(31);SimNodeUnitAccess::update(*node);EXPECT_TRUE(node->failed());EXPECT_NE(engine->error.find("camera renderer readiness"),std::string::npos);
  node.reset();EXPECT_THROW(create(),std::runtime_error); // enabled camera needs imported scene
}
TEST_F(SimNodeUnits, DirectServicesRetainResultsAndReportBackendFailures){
  create();
  using Pause=std_srvs::srv::SetBool;using Step=litesim3d_interfaces::srv::Step;using SetPose=litesim3d_interfaces::srv::SetPose;
  Pause::Response pause;response_callback=[&](void*p){pause=*static_cast<Pause::Response*>(p);return RCL_RET_OK;};auto p=std::make_shared<Pause::Request>();p->data=true;SimNodeUnitAccess::service<Pause>(*node,p);EXPECT_TRUE(pause.success);EXPECT_EQ(engine->current,State::Paused);engine->throw_pause=true;SimNodeUnitAccess::service<Pause>(*node,p);EXPECT_FALSE(pause.success);
  Step::Response step;response_callback=[&](void*p){step=*static_cast<Step::Response*>(p);return RCL_RET_OK;};auto s=std::make_shared<Step::Request>();s->request_id="id";s->steps=3;SimNodeUnitAccess::service<Step>(*node,s);EXPECT_FALSE(step.success);SimNodeUnitAccess::ready(*node,true);SimNodeUnitAccess::service<Step>(*node,s);EXPECT_TRUE(step.success);EXPECT_EQ(step.message,"id:3");engine->throw_step=true;SimNodeUnitAccess::service<Step>(*node,s);EXPECT_FALSE(step.success);EXPECT_TRUE(node->failed());
  SetPose::Response posed;response_callback=[&](void*p){posed=*static_cast<SetPose::Response*>(p);return RCL_RET_OK;};auto q=std::make_shared<SetPose::Request>();q->pose.orientation.w=1;q->pose.position.x=2;SimNodeUnitAccess::service<SetPose>(*node,q);EXPECT_TRUE(posed.success);EXPECT_EQ(engine->snap.world_base.translation().x(),2);engine->throw_pose=true;SimNodeUnitAccess::service<SetPose>(*node,q);EXPECT_FALSE(posed.success);EXPECT_EQ(posed.code,1);
}
TEST_F(SimNodeUnits, StatusSeverityFaultShutdownAndTfOwnershipAreDeterministic){
  create();for(int cause=0;cause<5;++cause){engine->current=State::Running;engine->error.clear();engine->timeout=cause==2;engine->rejected=cause==3;SimNodeUnitAccess::counters(*node,cause==0,cause==1);if(cause==4)engine->fault("unit-fault");SimNodeUnitAccess::status(*node);const auto&d=last<diagnostic_msgs::msg::DiagnosticArray>("/sim/status").status[0];EXPECT_EQ(d.level,cause==4?2:1);}
  engine->current=State::Paused;engine->timeout=false;engine->rejected=0;engine->error.clear();SimNodeUnitAccess::counters(*node,0,0);SimNodeUnitAccess::status(*node);EXPECT_EQ(last<diagnostic_msgs::msg::DiagnosticArray>("/sim/status").status[0].level,0);
  engine->fault("fault");SimNodeUnitAccess::status(*node);clock+=std::chrono::milliseconds(401);SimNodeUnitAccess::status(*node);EXPECT_EQ(shutdowns,1U);context_ok=false;auto before=published.size();SimNodeUnitAccess::periodic(*node);SimNodeUnitAccess::status(*node);EXPECT_EQ(published.size(),before);context_ok=true;
  tf2_msgs::msg::TFMessage msg;geometry_msgs::msg::TransformStamped t;t.header.frame_id="unrelated";t.child_frame_id="base_link";msg.transforms={t};SimNodeUnitAccess::tf(*node,msg,1);t.header.frame_id="odom";msg.transforms={t};SimNodeUnitAccess::tf(*node,msg,1);SimNodeUnitAccess::tf(*node,msg,2);EXPECT_EQ(engine->error,"duplicate odom TF publishers");
}
TEST(SimCameraProtocol, FutureCompletionAndCancellationUseInertDependencies){
 struct Future{Render::Response::SharedPtr value;auto get(){return value;}};struct Client{unsigned removed=0;Render::Request::SharedPtr sent;Future async_send_request(Render::Request::SharedPtr r){sent=r;return{std::make_shared<Render::Response>()};}void remove_pending_request(const Future&){++removed;}}client;
 struct Executor{rclcpp::FutureReturnCode result=rclcpp::FutureReturnCode::SUCCESS;double timeout=0;rclcpp::FutureReturnCode spin_until_future_complete(Future&,std::chrono::duration<double>d){timeout=d.count();return result;}}executor;
 auto request=std::make_shared<Render::Request>();EXPECT_NE(requestCamera(client,executor,request,2),nullptr);EXPECT_EQ(client.sent,request);EXPECT_EQ(executor.timeout,2);for(auto code:{rclcpp::FutureReturnCode::TIMEOUT,rclcpp::FutureReturnCode::INTERRUPTED}){executor.result=code;EXPECT_THROW(requestCamera(client,executor,request,2),std::runtime_error);}EXPECT_EQ(client.removed,2U);
}
TEST_F(SimNodeUnits, ProcessLifecycleUsesInjectedRuntimeAndReportsStartupErrors){
  create();SimMainHooks h;unsigned initialized=0,spins=0,closed=0,waits=0,reports=0;
  h.initialize=[&](int argc,char**argv){EXPECT_EQ(argc,0);EXPECT_EQ(argv,nullptr);++initialized;};
  h.make_node=[&]{return node;};h.spin=[&](std::shared_ptr<rclcpp::Node>){++spins;};h.ok=[]{return true;};h.shutdown=[&]{++closed;};h.wait=[&]{++waits;};
  h.report=[&](const diagnostic_msgs::msg::DiagnosticArray&r){++reports;ASSERT_EQ(r.status.size(),1U);EXPECT_EQ(r.status[0].level,2);EXPECT_EQ(r.status[0].message,"unit startup failure");};
  EXPECT_EQ(runSimNode(0,nullptr,h),0);EXPECT_EQ(engine->exits,1U);EXPECT_EQ(initialized,1U);EXPECT_EQ(spins,1U);EXPECT_EQ(closed,1U);
  engine->fault("fault");EXPECT_EQ(runSimNode(0,nullptr,h),1);EXPECT_EQ(engine->exits,1U);
  h.make_node=[]()->std::shared_ptr<SimNode>{throw std::runtime_error("unit startup failure");};
  EXPECT_EQ(runSimNode(0,nullptr,h),1);EXPECT_EQ(reports,10U);EXPECT_EQ(waits,10U);
  h.ok=[]{return false;};EXPECT_EQ(runSimNode(0,nullptr,h),1);EXPECT_EQ(reports,10U);
}
TEST_F(SimNodeUnits, JointControlAndPublisherCountsUseIsolatedBackendAndFeedbackSink){
 cfg.control_mode="ros2_control";cfg.drive.model="multicopter";create();
 EXPECT_EQ(last<litesim3d_interfaces::msg::JointFeedback>("/sim/joint_feedback").run_id,engine->run_id);
 litesim3d_interfaces::msg::JointCommand command;command.run_id=engine->run_id;command.header.stamp.sec=2;command.name={"arm"};command.mode={command.POSITION};command.command={1};
 SimNodeUnitAccess::joint(*node,command);ASSERT_EQ(engine->joint_commands.size(),1U);EXPECT_EQ(engine->joint_commands[0].joints.at("arm").value,1);
 joint_publishers=2;SimNodeUnitAccess::joint(*node,command);EXPECT_EQ(engine->joint_commands.size(),1U);
 auto e=event();SimNodeUnitAccess::events(*node,{e});EXPECT_EQ(last<nav_msgs::msg::Odometry>("/sim/odometry").header.frame_id,"odom");
}
TEST_F(SimNodeUnits, MotionPeriodicStatusNullBackendAndShutdownRaceAreHandled){
 sc.visual[0].motion=std::make_shared<Motion>();sc.visual[0].motion->velocity={1,0,0};create();
 SimNodeUnitAccess::sceneState(*node);SimNodeUnitAccess::periodic(*node);auto before=published.size();SimNodeUnitAccess::periodic(*node);EXPECT_EQ(published.size(),before);
 clock+=std::chrono::milliseconds(201);SimNodeUnitAccess::statusTimer(*node);EXPECT_GT(published.size(),before);
 auto backend=SimNodeUnitAccess::takeBackend(*node);EXPECT_FALSE(node->failed());SimNodeUnitAccess::status(*node);node->finish();SimNodeUnitAccess::restoreBackend(*node,std::move(backend));
 engine->throw_tick=true;engine->on_tick=[this]{context_ok=false;};SimNodeUnitAccess::ready(*node,true);SimNodeUnitAccess::update(*node);EXPECT_FALSE(node->failed());
}
TEST_F(SimNodeUnits, CameraModesDisabledSensorsAndAdditionalBufferGuards){
 for(const char*type:{"rgb","depth"}){
  sensors(true);cfg.cameras[0].type=type;create({rclcpp::Parameter("scene_description",sceneFile())});auto e=event();e.cameras={0};cameraResponse(e);SimNodeUnitAccess::cameras(*node,e);EXPECT_EQ(engine->processed,1U);
  cameraResponse(e);auto &image=std::string(type)=="rgb"?render_response->frames[0].color:render_response->frames[0].depth;
  image.header.frame_id="wrong";EXPECT_THROW(SimNodeUnitAccess::cameras(*node,e),std::runtime_error);
  cameraResponse(e);(std::string(type)=="rgb"?render_response->frames[0].color:render_response->frames[0].depth).height=2;EXPECT_THROW(SimNodeUnitAccess::cameras(*node,e),std::runtime_error);
  node.reset();
 }
 cfg.cameras[0].enabled=false;create();EXPECT_EQ(engine->processed,0U);
 node.reset();EXPECT_THROW(create({rclcpp::Parameter("scene_description",(dir/"missing.json").string())}),std::runtime_error);
 cfg.cameras.clear();EXPECT_THROW(create({rclcpp::Parameter("manifest_path",(dir/"missing"/"manifest.json").string())}),std::runtime_error);
}
TEST_F(SimNodeUnits, OptionalSerializerRoutesHandledCloudsInsteadOfNativePublishers){
 struct Serializer: litesim3d_ros::LidarOutputPlugin{unsigned outputs=0;Json description_{{"unit",true}};void initialize(rclcpp::Node&,const Config&,const std::string&,const std::string&,const Json&,const rclcpp::PublisherOptions&)override{}bool handles(const std::string&t)const override{return t=="points_raw";}bool publish(const Cloud&)override{++outputs;return true;}void matches(Json&r)const override{r["vendor"]=2;}const Json&description()const override{return description_;}};
 auto serializer=std::make_shared<Serializer>();serializer_factory=[serializer]{return serializer;};sensors();cfg.lidars[0].scan_pattern="livox_mid360";
 create({rclcpp::Parameter("lidar_interface","auto")});auto e=event();Cloud c;c.topic="points_raw";c.frame="lidar_link";e.clouds={c};SimNodeUnitAccess::events(*node,{e});EXPECT_EQ(serializer->outputs,1U);EXPECT_EQ(count("/points_raw"),0U);SimNodeUnitAccess::status(*node);
}
TEST_F(SimNodeUnits, GenericOutputAdaptersUseRecordingDoublesForTheirDefaultPaths){
 create();auto &h=SimNodeUnitAccess::hooks(*node);h.publish={};h.subscriptions={};h.now={};h.ok={};h.publishers={};h.camera_ready={};
 struct Publisher{std_msgs::msg::String last;const char*get_topic_name()const{return"unit";}size_t get_subscription_count()const{return 17;}void publish(const std_msgs::msg::String&m){last=m;}};
 auto publisher=std::make_shared<Publisher>();std_msgs::msg::String value;value.data="unit-payload";SimNodeUnitAccess::emit(*node,publisher,value);EXPECT_EQ(publisher->last.data,value.data);EXPECT_EQ(SimNodeUnitAccess::subscriptions(*node,publisher),17U);
 struct Broadcaster{geometry_msgs::msg::TransformStamped last;void sendTransform(const geometry_msgs::msg::TransformStamped&m){last=m;}}broadcaster;
 geometry_msgs::msg::TransformStamped transform;transform.child_frame_id="unit-child";SimNodeUnitAccess::transform(*node,broadcaster,transform);EXPECT_EQ(broadcaster.last.child_frame_id,"unit-child");
 EXPECT_TRUE(SimNodeUnitAccess::ok(*node));EXPECT_GT(SimNodeUnitAccess::now(*node).time_since_epoch().count(),0);EXPECT_EQ(SimNodeUnitAccess::publishers(*node,"/unit_missing_topic"),0U);EXPECT_FALSE(SimNodeUnitAccess::cameraReady(*node));
}
TEST_F(SimNodeUnits, RemainingMetadataAndReadinessGuardsHaveExplicitOutcomes){
 sensors();cfg.lidars[0].publish_range_boundary=false;create({rclcpp::Parameter("sensor_reliable",true)});SimNodeUnitAccess::qos(*node);SimNodeUnitAccess::status(*node);EXPECT_EQ(last<diagnostic_msgs::msg::DiagnosticArray>("/sim/status").status[0].level,1);
 auto snapshot=engine->snap;snapshot.joint_velocities.clear();SimNodeUnitAccess::state(*node,snapshot);EXPECT_EQ(last<sensor_msgs::msg::JointState>("/joint_states").velocity,std::vector<double>{0});
 auto e=event();e.odom=false;auto before=count("/odom");SimNodeUnitAccess::events(*node,{e});EXPECT_EQ(count("/odom"),before);
 tf2_msgs::msg::TFMessage msg;geometry_msgs::msg::TransformStamped t;t.header.frame_id="odom";t.child_frame_id="unrelated";msg.transforms={t};SimNodeUnitAccess::tf(*node,msg,1);EXPECT_FALSE(node->failed());
 node.reset();cfg.lidars.clear();cfg.imus.clear();std::ofstream(dir/"ready")<<"unit";create({rclcpp::Parameter("viewer_ready_file",(dir/"ready").string())});SimNodeUnitAccess::update(*node);EXPECT_TRUE(SimNodeUnitAccess::ready(*node));EXPECT_EQ(engine->ticks,1U);
 node.reset();cfg.real_time_factor=1;create({rclcpp::Parameter("start_paused",true)});SimNodeUnitAccess::update(*node);EXPECT_EQ(engine->current,State::Paused);EXPECT_EQ(engine->ticks,1U);
 node.reset();cfg.real_time_factor=0;sensors(true);create({rclcpp::Parameter("scene_description",sceneFile())});SimNodeUnitAccess::update(*node);EXPECT_TRUE(SimNodeUnitAccess::ready(*node));auto &h=SimNodeUnitAccess::hooks(*node);h.camera_ready={};EXPECT_FALSE(SimNodeUnitAccess::cameraReady(*node));
 node.reset();cfg.cameras.clear();cfg.control_mode="ros2_control";create();SimNodeUnitAccess::status(*node);EXPECT_EQ(last<diagnostic_msgs::msg::DiagnosticArray>("/sim/status").status[0].level,0);
}
TEST_F(SimNodeUnits, DefaultInputLoadersAndFactoryFailBeforeConstructingPhysics){
 auto h=hooks();h.load_config={};rclcpp::NodeOptions options;options.parameter_overrides({rclcpp::Parameter("config",(dir/"missing.yaml").string())});EXPECT_THROW(std::make_shared<SimNode>(options,h),std::runtime_error);EXPECT_EQ(engine,nullptr);
 h=hooks();h.load_model={};options=rclcpp::NodeOptions{};EXPECT_THROW(std::make_shared<SimNode>(options,h),std::runtime_error);EXPECT_EQ(engine,nullptr);
 SimMainHooks main;main.initialize=[](int,char**){};main.ok=[]{return true;};main.spin=[](std::shared_ptr<rclcpp::Node>){};main.shutdown=[]{};main.wait=[]{};unsigned reports=0;main.report=[&](const diagnostic_msgs::msg::DiagnosticArray&r){++reports;EXPECT_EQ(r.status[0].level,2);};EXPECT_EQ(runSimNode(0,nullptr,main),1);EXPECT_EQ(reports,10U);EXPECT_EQ(engine,nullptr);
}
