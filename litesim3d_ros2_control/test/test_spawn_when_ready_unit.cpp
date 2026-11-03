// No exec, controller manager or DDS peer: production startup gate and argv
// forwarding run against recording ROS and executable lookup/exec boundaries.
#include <gtest/gtest.h>
#include <ament_index_cpp/get_package_prefix.hpp>
#include <unistd.h>
#include <cerrno>
#include <sstream>
namespace launch_double {
inline std::string path;inline std::vector<std::string>args;
}
namespace ament_index_cpp {std::string get_package_prefix(const std::string&){return "/unit-prefix";}}
extern "C" int execv(const char*file,char*const args[])noexcept {
  launch_double::path=file;launch_double::args.clear();
  for(size_t i=0;args[i];++i)launch_double::args.emplace_back(args[i]);errno=ENOENT;return -1;
}
#define main litesim_spawn_when_ready_entrypoint
#include "../src/spawn_when_ready.cpp"
#undef main
namespace {
namespace transport=rclcpp::unit;
class StartupUnits:public testing::Test {
protected:
  void SetUp()override{transport::reset();launch_double::path.clear();launch_double::args.clear();}
  void TearDown()override{transport::reset();}
  int invoke(std::vector<std::string>args){
    args.insert(args.begin(),"spawn_when_ready");std::vector<char*>argv;for(auto&s:args)argv.push_back(s.data());
    std::ostringstream out,err;auto*o=std::cout.rdbuf(out.rdbuf());auto*e=std::cerr.rdbuf(err.rdbuf());
    int result=litesim_spawn_when_ready_entrypoint(argv.size(),argv.data());std::cout.rdbuf(o);std::cerr.rdbuf(e);
    output=out.str();error=err.str();return result;
  }
  void deliver(const std::string&status_topic="/sim/status",const std::string&clock_topic="/clock",bool fault=false){
    diagnostic_msgs::msg::DiagnosticArray m;m.status.resize(1);
    for(const auto&p:std::map<std::string,std::string>{{"ready","true"},{"run_id","run"},{"state",fault?"fault":"running"}}){
      diagnostic_msgs::msg::KeyValue v;v.key=p.first;v.value=p.second;m.status[0].values.push_back(v);
    }
    transport::deliver(status_topic,m);rosgraph_msgs::msg::Clock c;c.clock.sec=1;c.clock.nanosec=transport::spins;transport::deliver(clock_topic,c);
  }
  std::string output,error;
};
TEST_F(StartupUnits,HelpAndInvalidOptionsDoNotInitializeOrExecute){
  for(const auto&flag:{"--help","-h"}){EXPECT_EQ(invoke({flag}),0);EXPECT_NE(output.find("usage:"),std::string::npos);}
  for(const auto&args:std::vector<std::vector<std::string>>{
      {},{"--"},{"--timeout"},{"--unknown","x"},{"","x"},{"--timeout","0"},
      {"--executable","invalid","a"},{"--timeout","nan","a"}}){
    EXPECT_EQ(invoke(args),1);EXPECT_FALSE(error.empty());EXPECT_FALSE(transport::active_context);EXPECT_TRUE(launch_double::path.empty());
  }
}
TEST_F(StartupUnits,ReadinessRequiresAnAdvancingClockAndShutsDownContext){
  transport::on_spin=[this]{deliver();};EXPECT_NO_THROW(wait_for_simulation(.1,"/sim/status","/clock"));
  EXPECT_EQ(transport::spins,2u);ASSERT_TRUE(transport::active_context);
  EXPECT_FALSE(transport::active_context->is_valid());EXPECT_EQ(transport::active_context->shutdown_reason,"ready");
}
TEST_F(StartupUnits,FaultTimeoutAndContextShutdownNeverExecuteATarget){
  transport::on_spin=[this]{deliver("/sim/status","/clock",true);};
  EXPECT_THROW(wait_for_simulation(.1,"/sim/status","/clock"),std::runtime_error);
  EXPECT_EQ(transport::active_context->shutdown_reason,"startup failed");
  transport::reset();transport::on_spin=[] {transport::active_context->shutdown("external");};
  EXPECT_THROW(wait_for_simulation(.1,"/sim/status","/clock"),std::runtime_error);
  transport::reset();EXPECT_THROW(wait_for_simulation(0,"/sim/status","/clock"),std::runtime_error);
  EXPECT_EQ(transport::active_context->shutdown_reason,"startup failed");EXPECT_TRUE(launch_double::path.empty());
}
TEST_F(StartupUnits,EmptyStatusAndNoClockCannotReportReady){
  transport::on_spin=[] {diagnostic_msgs::msg::DiagnosticArray m;transport::deliver("/sim/status",m);};
  EXPECT_THROW(wait_for_simulation(.001,"/sim/status","/clock"),std::runtime_error);
  EXPECT_GT(transport::spins,0u);EXPECT_TRUE(launch_double::path.empty());
}
TEST_F(StartupUnits,SpawnerArgumentsPreserveDelimiterAndBareForms){
  for(bool delimiter:{false,true}){
    transport::reset();transport::on_spin=[this]{deliver();};
    std::vector<std::string>args={"joint_state","-c","/manager"};if(delimiter)args.insert(args.begin(),"--");
    EXPECT_EQ(invoke(args),1);EXPECT_EQ(launch_double::path,"/unit-prefix/lib/controller_manager/spawner");
    EXPECT_EQ(launch_double::args,(std::vector<std::string>{launch_double::path,"joint_state","-c","/manager"}));
    EXPECT_NE(error.find("cannot execute"),std::string::npos);
  }
}
TEST_F(StartupUnits,ControlNodeAndCustomTopicsUseTheirOwnPackage){
  transport::on_spin=[this]{deliver("/custom/status","/custom/clock");};
  EXPECT_EQ(invoke({"--timeout",".1","--status-topic","/custom/status","--clock-topic","/custom/clock",
                    "--executable","ros2_control_node","--","--ros-args","-p","name:=value"}),1);
  EXPECT_EQ(launch_double::path,"/unit-prefix/lib/litesim3d_ros2_control/ros2_control_node");
  EXPECT_EQ(launch_double::args.back(),"name:=value");EXPECT_EQ(transport::active_context->shutdown_reason,"ready");
}
}
