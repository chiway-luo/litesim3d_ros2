// White-box orchestration tests include the unchanged Session implementation.
// Process and graph boundaries are deterministic; no child process or DDS peer.
#include "doubles/unit_clock.hpp"
#include "../src/session_protocol.hpp"
#include <ament_index_cpp/get_package_prefix.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <atomic>
#include <csignal>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <gtest/gtest.h>
namespace process_double {
using namespace litesim3d::runtime;
struct State{std::vector<std::string> argv;ProcessOptions options;std::optional<int> code;int stopped=0;bool forced=false,throw_stop=false;};
inline std::map<int,State> states;inline int next_pid=100;
inline std::function<void()> on_poll,on_spawn;
inline bool interrupted=false;inline CommandResult result{0,"validated",""};
inline std::vector<std::string> last_run;
void reset(){states.clear();next_pid=100;on_poll=on_spawn={};interrupted=false;result={0,"validated",""};last_run.clear();}
}
namespace litesim3d::runtime {
// Executable-local definitions interpose only OS/lookup boundaries in this target.
Process::Process(const std::vector<std::string>&argv,const ProcessOptions&options):pid_(process_double::next_pid++){
  process_double::states[pid_]={argv,options};if(process_double::on_spawn)process_double::on_spawn();
}
Process::~Process()=default;
Process::Process(Process&&other)noexcept:pid_(other.pid_),result_(other.result_){other.pid_=-1;}
Process& Process::operator=(Process&&other)noexcept{pid_=other.pid_;result_=other.result_;other.pid_=-1;return *this;}
std::optional<int> Process::poll(){if(process_double::on_poll)process_double::on_poll();return process_double::states.at(pid_).code;}
int Process::stop(double,bool*forced){auto&s=process_double::states.at(pid_);++s.stopped;if(s.throw_stop)throw std::runtime_error("stop failed");if(forced)*forced=s.forced;return s.code.value_or(0);}
bool interruption_requested(){return process_double::interrupted;}
fs::path executable(const std::string&p,const std::string&n){return fs::path("/unit-bin")/p/n;}
CommandResult run(const std::vector<std::string>&args,double,const std::string&,const std::map<std::string,std::string>&){
  process_double::last_run=args;return process_double::result;
}
}
namespace ament_index_cpp {std::string get_package_prefix(const std::string&){return "/unit-prefix";}}
#define private public
#define SteadyClock UnitClock
#define main litesim_session_entrypoint
#include "../src/sim_session.cpp"
#undef main
#undef SteadyClock
#undef private
namespace {
namespace transport=rclcpp::unit;
using namespace litesim3d::session;
class SessionUnits:public testing::Test {
protected:
  void SetUp()override{
    transport::reset();process_double::reset();UnitClock::reset();
    temp=std::make_unique<TemporaryDirectory>("session-units-");
    config=temp->path()/"config.yaml";output=temp->path()/"out";
    write(temp->path()/"robot.urdf","<robot name='test'/>");
    data={{"robot",{{"urdf","robot.urdf"}}}};save();
  }
  void TearDown()override{node.reset();transport::reset();process_double::reset();}
  void save(){write(config,dump_yaml(data));}
  void create(Json members=Json::array()){
    Options options;options.config=config;options.output=output;options.timeout=2;options.sensor_reliable=true;
    if(!members.empty()){options.members=temp->path()/"members.json";write(options.members,members.dump());}
    node=std::make_unique<Session>(options);
  }
  void status(std::string run="epoch",std::string ready="true",std::string state="paused",Json matches=Json::object()){
    diagnostic_msgs::msg::DiagnosticArray message;message.status.resize(1);
    for(const auto&p:std::map<std::string,std::string>{{"run_id",run},{"ready",ready},{"state",state},{"time_ns","123"},{"subscription_matches",matches.dump()}}){
      diagnostic_msgs::msg::KeyValue v;v.key=p.first;v.value=p.second;message.status.front().values.push_back(v);
    }
    transport::deliver("/sim/status",message);
  }
  Json records(){Json rows=Json::array();std::istringstream in(read(output/"session.jsonl"));std::string line;while(std::getline(in,line))rows.push_back(Json::parse(line));return rows;}
  Json member(){return Json::array({{{"name","consumer"},{"argv",Json::array({"/unit/consumer"})},{"ready_topics",Json::array({"/odom"})}}});}
  rclcpp::TopicEndpointInfo endpoint(std::string name,std::string ns="/",std::string type="type"){
    rclcpp::TopicEndpointInfo e;e.name=name;e.ns=ns;e.type=type;return e;
  }
  std::unique_ptr<TemporaryDirectory>temp;fs::path config,output;Json data;std::unique_ptr<Session>node;
};
TEST_F(SessionUnits,ConstructorRejectsInvalidConfigAndOccupiedOutput){
  data=Json::array();save();EXPECT_THROW(create(),std::runtime_error);
  data=Json::object();save();EXPECT_THROW(create(),std::runtime_error);
  data={{"robot",{{"urdf","robot.urdf"}}}};save();write(output,"occupied");
  EXPECT_THROW(create(),std::runtime_error);fs::remove(output);
  fs::create_directory(output);write(output/"kept","occupied");EXPECT_THROW(create(),std::runtime_error);
  EXPECT_EQ(read(output/"kept"),"occupied");
}
TEST_F(SessionUnits,TopicsAndStatusCallbackRetainOnlyFreshMessageValues){
  data["lidars"]=Json::array({Json::object(),{{"topic","//scan"}}});data["imus"]=Json::array({Json::object()});save();
  create();EXPECT_TRUE(node->input_topics_.count("/points_raw"));EXPECT_TRUE(node->input_topics_.count("/scan"));
  EXPECT_TRUE(node->input_topics_.count("/imu/data_raw"));EXPECT_EQ(topic_name("///"),"/");
  diagnostic_msgs::msg::DiagnosticArray empty;transport::deliver("/sim/status",empty);EXPECT_EQ(node->status().second,0);
  status("first");status("second");EXPECT_EQ(node->status().second,2);EXPECT_EQ(node->status().first.at("run_id"),"second");
  EXPECT_EQ(records().front()["event"],"created");
}
TEST_F(SessionUnits,StartupWritesParametersSpawnsInOrderAndFinishesCleanly){
  create();process_double::on_poll=[this]{status();};node->start();
  EXPECT_EQ(node->epoch_,1);EXPECT_EQ(node->coordinator_.run_id,"epoch");EXPECT_FALSE(node->coordinator_.transitioning);
  ASSERT_EQ(process_double::states.size(),2u);
  EXPECT_EQ(process_double::states.begin()->second.argv[0],"/unit-bin/robot_state_publisher/robot_state_publisher");
  auto params=read_yaml(output/"epoch-0001"/"sim.yaml")["/**"]["ros__parameters"];
  EXPECT_TRUE(params["start_paused"]);EXPECT_TRUE(params["sensor_reliable"]);EXPECT_FALSE(params.contains("initial_pose_override"));
  EXPECT_FALSE(node->monitor());EXPECT_EQ(node->finish(0),0);EXPECT_TRUE(node->children_.empty());EXPECT_TRUE(node->coordinator_.stopping);
  EXPECT_EQ(records().back()["event"],"exit");
}
TEST_F(SessionUnits,StartupWaitChecksPredicateContextDeadlineAndChildFailures){
  create();unsigned calls=0;
  node->wait_startup([&]{return ++calls==2;},UnitClock::time_point{}+std::chrono::seconds(1),"predicate");EXPECT_EQ(calls,2);
  EXPECT_THROW(node->wait_startup([]{return false;},UnitClock::time_point{},"timeout"),std::runtime_error);
  process_double::interrupted=true;EXPECT_THROW(node->wait_startup([]{return true;},UnitClock::time_point{},"signal"),std::runtime_error);
  process_double::interrupted=false;transport::running=false;
  EXPECT_THROW(node->wait_startup([]{return true;},UnitClock::time_point{},"shutdown"),std::runtime_error);transport::running=true;
  node->spawn("failed",{"fake"},output);process_double::states.begin()->second.code=0;
  EXPECT_THROW(node->wait_startup([]{return true;},UnitClock::time_point{},"child"),std::runtime_error);
  EXPECT_FALSE(node->monitor());node->coordinator_.transitioning=false;EXPECT_TRUE(node->monitor());
  EXPECT_EQ(node->finish(0),1);
}
TEST_F(SessionUnits,StartupRejectsFaultAndDuplicateEpochAndWaitsForNewIdentity){
  create();process_double::on_poll=[this]{status("id","false","fault");};EXPECT_THROW(node->start(),std::runtime_error);
  node->stop_owned();node->epoch_=0;EXPECT_THROW(node->start(),std::runtime_error);
  node->epoch_=1;node->coordinator_.run_id="old";unsigned polls=0;
  process_double::on_poll=[&]{++polls;status(polls<=2?"old":"new",polls<=2?"false":"true");};
  node->start_epoch({1,2,3,0,0,0,1});EXPECT_EQ(node->coordinator_.run_id,"new");
  auto params=read_yaml(output/"epoch-0002"/"sim.yaml")["/**"]["ros__parameters"];
  EXPECT_EQ(params["initial_pose_override"],Json::array({1,2,3,0,0,0,1}));EXPECT_GT(polls,2);
}
TEST_F(SessionUnits,StopAccountsForSignalForcedExitAndExceptions){
  create();for(const auto&name:{"normal","signal","forced","bad","exception"})node->spawn(name,{name},output);
  auto it=process_double::states.begin();(++it)->second.code=-SIGINT;(++it)->second.forced=true;
  (++it)->second.code=2;(++it)->second.throw_stop=true;
  EXPECT_THROW(node->stop_owned(),std::runtime_error);EXPECT_TRUE(node->children_.empty());
  node->spawn("broken",{"broken"},output);process_double::states.rbegin()->second.throw_stop=true;
  EXPECT_EQ(node->finish(0),1);auto rows=records();EXPECT_EQ(rows[rows.size()-2]["event"],"cleanup_failure");
  node->failure("detail");EXPECT_EQ(records().back()["reason"],"detail");
}
TEST_F(SessionUnits,GraphRequiresKnownEndpointsAndMemberParameterServices){
  create(member());EXPECT_FALSE(node->graph_complete());transport::services["consumer"]={"/consumer/get_parameters"};
  EXPECT_TRUE(node->graph_complete());
  for(const auto&e:{endpoint(""),endpoint("_NODE_NAME_UNKNOWN_"),endpoint("ok",""),endpoint("ok","relative")}){
    transport::subscribers["/odom"]={e};EXPECT_FALSE(node->graph_complete());
  }
  transport::subscribers["/odom"]={endpoint("consumer"),endpoint("ignored","/other")};
  EXPECT_TRUE(node->graph_complete());node->remember_topics();EXPECT_TRUE(node->members_.topics.at("consumer").count("/odom"));
  transport::subscribers["/joint_states"]={endpoint("consumer")};node->remember_topics();
  EXPECT_TRUE(node->members_.topics.at("consumer").count("/joint_states"));
}
TEST_F(SessionUnits,ParticipantOwnershipIncludesObserverAndCommandRestrictions){
  create(member());node->options_.observers.insert("viewer");
  transport::subscribers["/odom"]={endpoint("consumer"),endpoint("viewer"),endpoint("litesim3d")};
  transport::publishers["/cmd_vel"]={endpoint("consumer")};EXPECT_NO_THROW(node->check_participants());
  for(const auto&e:{endpoint("unknown"),endpoint("consumer","/foreign")}){
    transport::subscribers["/tf"]={e};EXPECT_THROW(node->check_participants(),std::runtime_error);
  }
  transport::subscribers.erase("/tf");
  for(const auto&e:{endpoint("viewer"),endpoint("consumer","/foreign")}){
    transport::publishers["/cmd_vel"]={e};EXPECT_THROW(node->check_participants(),std::runtime_error);
  }
}
TEST_F(SessionUnits,QosCompatibilityRejectsTypeAndReliabilityMismatch){
  create();auto pub=endpoint("litesim3d"),sub=endpoint("consumer");
  EXPECT_TRUE(node->compatible(pub,sub));sub.type="other";EXPECT_FALSE(node->compatible(pub,sub));sub.type=pub.type;
  pub.qos.best_effort();sub.qos.reliable();EXPECT_FALSE(node->compatible(pub,sub));sub.qos.best_effort();EXPECT_TRUE(node->compatible(pub,sub));
}
TEST_F(SessionUnits,MemberReadinessProbesExecutorFreshStatusAndMatchCounts){
  create(member());node->coordinator_.run_id="epoch";
  transport::services["consumer"]={"/consumer/get_parameters"};
  transport::subscribers["/odom"]={endpoint("consumer"),endpoint("observer"),endpoint("different","/ns","different")};
  transport::publishers["/odom"]={endpoint("wrong"),endpoint("litesim3d","/wrong"),endpoint("litesim3d")};
  UnitClock::on_now=[this]{status("epoch","true","paused",{{"/odom",2}});};
  node->wait_members(UnitClock::time_point{}+std::chrono::seconds(10));
  EXPECT_EQ(transport::request_count,1u);EXPECT_EQ(transport::removed,0u);EXPECT_EQ(records().back()["event"],"member_ready");
}
TEST_F(SessionUnits,ExecutorProbeRetriesAndRemovesPendingRequests){
  create(member());node->coordinator_.run_id="epoch";node->members_.topics["consumer"].clear();
  transport::services["consumer"]={"/consumer/get_parameters"};transport::response_ready=false;
  UnitClock::step=std::chrono::seconds(1);UnitClock::on_now=[this]{status();};
  unsigned waits=0;transport::on_wait=[&]{if(++waits==3)transport::response_ready=true;};
  node->wait_members(UnitClock::time_point{}+std::chrono::seconds(20));
  EXPECT_EQ(transport::request_count,2u);EXPECT_EQ(transport::removed,1u);
  bool retry=false;for(const auto&row:records())retry=retry||row["event"]=="member_executor_probe_retry";EXPECT_TRUE(retry);
}
TEST_F(SessionUnits,MemberProbeExceptionsAndDeadlineAlwaysClearPendingRequest){
  create(member());transport::response_throws=true;
  EXPECT_THROW(node->wait_members(UnitClock::time_point{}+std::chrono::seconds(1)),std::runtime_error);
  EXPECT_EQ(transport::removed,1u);transport::response_throws=false;transport::response_ready=false;
  UnitClock::step=std::chrono::seconds(2);
  EXPECT_THROW(node->wait_members(UnitClock::time_point{}+std::chrono::seconds(1)),std::runtime_error);
  EXPECT_EQ(transport::removed,2u);
  transport::service_ready=false;EXPECT_THROW(node->wait_members(UnitClock::time_point{}),std::runtime_error);
  EXPECT_EQ(transport::removed,2u);
}
TEST_F(SessionUnits,PreflightPreservesNormalizedPoseAndRejectsNonzeroResult){
  create();Session::Reset::Request request;request.initial_pose.position.x=2;request.initial_pose.orientation.w=2;
  EXPECT_EQ(node->validate_pose(request),(std::vector<double>{2,0,0,0,0,0,1}));
  ASSERT_EQ(process_double::last_run.size(),4u);EXPECT_EQ(process_double::last_run[0],"/unit-prefix/bin/sim_headless");
  EXPECT_EQ(read(output/"preflight.log"),"validated");
  auto candidate=read_yaml(output/"reset-candidate.yaml");EXPECT_EQ(candidate["robot"]["initial_pose"]["xyz"][0],2);
  process_double::result={3,"","bad pose"};EXPECT_THROW(node->validate_pose(request),std::runtime_error);
  EXPECT_EQ(read(output/"preflight.log"),"bad pose");EXPECT_EQ(node->epoch_,0);
}
TEST_F(SessionUnits,ResetServiceRunsTransactionalActionsAndKeepsCachedResult){
  create();process_double::on_poll=[this]{status("run-"+std::to_string(node->epoch_));};node->start();
  auto request=std::make_shared<Session::Reset::Request>();auto response=std::make_shared<Session::Reset::Response>();
  request->request_id="reset";request->expected_run_id="run-1";request->set_initial_pose=true;request->initial_pose.orientation.w=1;
  node->service_->callback(request,response);
  EXPECT_TRUE(response->success);EXPECT_EQ(response->run_id,"run-2");EXPECT_EQ(node->epoch_,2);
  node->service_->callback(request,response);EXPECT_TRUE(response->success);EXPECT_EQ(node->epoch_,2);
  request->request_id="stale";node->service_->callback(request,response);
  EXPECT_FALSE(response->success);EXPECT_EQ(node->epoch_,2);
}
TEST_F(SessionUnits,ConcurrentResetIsRejectedWithoutMutatingEpoch){
  create();std::promise<void> locked,release;auto future=release.get_future();
  std::thread holder([&]{std::lock_guard<std::timed_mutex> lock(node->mutation_);locked.set_value();future.wait();});
  locked.get_future().wait();Session::Reset::Request request;Session::Reset::Response response;
  node->reset(request,response);release.set_value();holder.join();
  EXPECT_FALSE(response.success);EXPECT_EQ(response.message,"reset already in progress");EXPECT_EQ(node->epoch_,0);
}
int call_session(std::vector<std::string>args){
  args.insert(args.begin(),"sim_session");std::vector<char*>argv;for(auto&s:args)argv.push_back(s.data());
  std::ostringstream out,err;auto *o=std::cout.rdbuf(out.rdbuf()),*e=std::cerr.rdbuf(err.rdbuf());
  int result=litesim_session_entrypoint(argv.size(),argv.data());std::cout.rdbuf(o);std::cerr.rdbuf(e);return result;
}
TEST_F(SessionUnits,CliValidatesAllOptionsBeforeConstruction){
  EXPECT_EQ(call_session({"--help"}),0);EXPECT_EQ(call_session({"-h"}),0);
  for(auto args:std::vector<std::vector<std::string>>{
      {},{"--config"},{"--unknown","x"},{"--timeout","1x"},{"--timeout","nan"},
      {"--config","x","--output","y","--timeout",".9"},{"--config","x","--output","y","--timeout","121"}})
    EXPECT_EQ(call_session(args),1);
  EXPECT_EQ(call_session({"--config",config.string(),"--output",output.string(),"--timeout","1",
                         "--observer","viewer","--members","/missing/unit-members","--sensor-reliable"}),1);
}
TEST_F(SessionUnits,CliFinishesOwnedProcessesAfterMonitoredExit){
  unsigned polls=0;
  process_double::on_poll=[&]{
    status("main-run");
    if(++polls>2)process_double::states.begin()->second.code=0;
  };
  EXPECT_EQ(call_session({"--config",config.string(),"--output",output.string(),"--timeout","1"}),1);
  EXPECT_GE(polls,3u);for(const auto&item:process_double::states)EXPECT_EQ(item.second.stopped,1);
  EXPECT_EQ(records().back()["event"],"exit");EXPECT_FALSE(transport::running);
}
TEST_F(SessionUnits,CliHandlesStartupFailureAndCleanup){
  process_double::on_poll=[this]{status("","false","fault");};
  EXPECT_EQ(call_session({"--config",config.string(),"--output",output.string()}),1);
  EXPECT_EQ(records().back()["event"],"exit");
}
}
