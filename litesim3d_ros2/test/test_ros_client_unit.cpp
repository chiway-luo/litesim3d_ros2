// The CLI implementation runs against a recording ROS boundary and virtual time.
// Serialization, option validation and every message callback are production code.
#include "doubles/unit_clock.hpp"
#define SteadyClock UnitClock
#define main litesim_ros_client_entrypoint
#include "../src/ros_client.cpp"
#undef main
#undef SteadyClock
#include <gtest/gtest.h>
#include <sstream>
namespace {
namespace transport=rclcpp::unit;
struct Capture {
  std::ostringstream out,err;
  std::streambuf *old_out=std::cout.rdbuf(out.rdbuf()),*old_err=std::cerr.rdbuf(err.rdbuf());
  ~Capture(){std::cout.rdbuf(old_out);std::cerr.rdbuf(old_err);}
};
class ClientUnits:public testing::Test {
protected:
  void SetUp()override{transport::reset();UnitClock::reset();}
  void TearDown()override{transport::reset();}
  int invoke(std::vector<std::string> args){
    args.insert(args.begin(),"ros_client");std::vector<char*> argv;
    for(auto&s:args)argv.push_back(s.data());
    Capture capture;int code=litesim_ros_client_entrypoint(argv.size(),argv.data());
    output=capture.out.str();error=capture.err.str();
    if(!output.empty()&&output.front()=='{')report=Json::parse(output);
    return code;
  }
  void messages(bool clock=true,bool points=true,bool odom=true,bool imu=true,double x=0){
    if(clock){rosgraph_msgs::msg::Clock m;m.clock.sec=2;m.clock.nanosec=3;transport::deliver("/clock",m);}
    if(points){
      sensor_msgs::msg::PointCloud2 m;m.header.frame_id="lidar";m.width=2;m.height=1;m.point_step=16;
      m.data.resize(32);sensor_msgs::msg::PointField f;f.name="x";m.fields={f};transport::deliver("points_raw",m);
    }
    if(odom){nav_msgs::msg::Odometry m;m.header.frame_id="odom";m.child_frame_id="base";m.pose.pose.position.x=x;transport::deliver("odom",m);}
    if(imu){sensor_msgs::msg::Imu m;m.header.frame_id="imu";m.linear_acceleration.z=9.8;transport::deliver(imu_topic,m);}
  }
  Json report;std::string output,error,imu_topic="imu/data_raw";
};
TEST_F(ClientUnits,HelpAndInvalidArgumentsHaveSpecificDiagnostics){
  for(const auto&flag:{"--help","-h"}){EXPECT_EQ(invoke({flag}),0);EXPECT_NE(output.find("usage: ros_client"),std::string::npos);}
  for(const auto&args:std::vector<std::vector<std::string>>{
      {"--namespace"},{"--unknown","x"},{"--linear","no"},{"--linear","1x"},
      {"--angular","nan"},{"--duration","0"},{"--duration","-2"},
      {"--require-motion"},{"--require-motion","--drive","--linear","0","--angular","0"}}){
    EXPECT_EQ(invoke(args),1);EXPECT_NE(error.find("ros_client:"),std::string::npos);EXPECT_TRUE(transport::subscriptions.empty());
  }
}
TEST_F(ClientUnits,DisconnectedReportRetainsAllCountsAndNoCommands){
  EXPECT_EQ(invoke({"--duration",".05"}),1);
  EXPECT_FALSE(report["connected"]);EXPECT_FALSE(report["success"]);EXPECT_EQ(report["counts"]["commands"],0);
  EXPECT_FALSE(report.contains("odom_displacement_m"));EXPECT_TRUE(transport::publications.empty());
}
TEST_F(ClientUnits,AllMessageCallbacksPopulateIndependentReportFields){
  transport::on_spin=[this]{messages(true,true,true,true,.03*transport::spins);};
  EXPECT_EQ(invoke({"--duration",".12","--require-imu","--expect-imu-frame","imu"}),0);
  EXPECT_EQ(report["clock_ns"],2000000003LL);EXPECT_EQ(report["cloud"]["data_bytes"],32);
  EXPECT_EQ(report["cloud"]["fields"],Json::array({"x"}));EXPECT_EQ(report["frames"]["base"],"base");
  EXPECT_EQ(report["imu_acceleration"],Json::array({0,0,9.8}));EXPECT_GT(report["odom_displacement_m"].get<double>(),0);
  EXPECT_TRUE(report["checks"]["imu_frame"]);EXPECT_EQ(transport::node_parameters.front().name,"use_sim_time");
}
TEST_F(ClientUnits,EveryConnectivityRequirementIsNecessary){
  for(unsigned missing=0;missing<3;++missing){
    transport::reset();UnitClock::reset();
    transport::on_spin=[&,missing]{messages(missing!=0,missing!=1,missing!=2);};
    EXPECT_EQ(invoke({"--duration",".05"}),1);EXPECT_FALSE(report["checks"]["connected"]);
  }
}
TEST_F(ClientUnits,TwistDrivePublishesMotionAndThreeFinalStops){
  transport::on_spin=[this]{messages(true,true,true,true,.1*transport::spins);};
  EXPECT_EQ(invoke({"--drive","--duration",".4","--linear",".3","--angular",".2","--require-motion"}),0);
  const auto&sent=transport::publications.at("cmd_vel");ASSERT_GT(sent.size(),3u);
  auto first=std::any_cast<geometry_msgs::msg::Twist>(sent.front());
  EXPECT_DOUBLE_EQ(first.linear.x,.3);EXPECT_DOUBLE_EQ(first.angular.z,.2);
  for(size_t i=sent.size()-3;i<sent.size();++i){auto m=std::any_cast<geometry_msgs::msg::Twist>(sent[i]);EXPECT_EQ(m.linear.x,0);EXPECT_EQ(m.angular.z,0);}
  EXPECT_EQ(report["counts"]["commands"].get<size_t>(),sent.size());EXPECT_TRUE(report["checks"]["motion"]);
}
TEST_F(ClientUnits,StampedModeUsesNormalizedNamespaceAndExplicitFrame){
  for(const auto&frame:{std::string{},std::string{"custom"}}){
    transport::reset();UnitClock::reset();transport::on_spin=[this]{messages();};
    std::vector<std::string>args={"--drive","--stamped","--namespace","///robot///","--duration",".1","--linear","0","--angular",".4"};
    if(!frame.empty()){args.push_back("--frame");args.push_back(frame);}
    EXPECT_EQ(invoke(args),0);EXPECT_EQ(report["namespace"],"robot");EXPECT_EQ(report["cmd_mode"],"stamped");
    auto m=std::any_cast<geometry_msgs::msg::TwistStamped>(transport::publications.at("cmd_vel").front());
    EXPECT_EQ(m.header.frame_id,frame.empty()?"robot/base_link":frame);EXPECT_EQ(m.header.stamp.sec,1);EXPECT_DOUBLE_EQ(m.twist.angular.z,.4);
  }
}
TEST_F(ClientUnits,ImuFrameRequirementDetectsAbsentAndWrongFrames){
  for(bool present:{false,true}){
    transport::reset();UnitClock::reset();imu_topic="imu_custom";
    transport::on_spin=[&,present]{messages(true,true,true,present);};
    EXPECT_EQ(invoke({"--duration",".05","--imu-topic","imu_custom","--expect-imu-frame","wrong"}),1);
    EXPECT_EQ(report["checks"]["imu"],present);EXPECT_FALSE(report["checks"]["imu_frame"]);
  }
}
TEST_F(ClientUnits,DriveWaitsForTimeConnectivityAndSubscribers){
  for(unsigned blocked=0;blocked<3;++blocked){
    transport::reset();UnitClock::reset();
    transport::now_ns=blocked==0?0:1000000000;transport::subscriber_count=blocked==2?0:1;
    transport::on_spin=[&,blocked]{messages(blocked!=1);};
    EXPECT_EQ(invoke({"--drive","--duration",".1","--require-motion"}),1);
    EXPECT_EQ(report["counts"]["moving_commands"],0);EXPECT_EQ(report["counts"]["commands"],3);
    EXPECT_FALSE(report["checks"]["motion"]);
  }
}
TEST_F(ClientUnits,StationaryOdometryFailsRequiredMotionDespiteCommands){
  transport::on_spin=[this]{messages();};
  EXPECT_EQ(invoke({"--drive","--duration",".2","--require-motion"}),1);
  EXPECT_GT(report["counts"]["moving_commands"].get<int>(),0);EXPECT_EQ(report["odom_displacement_m"],0);
}
TEST_F(ClientUnits,ShutdownSkipsFinalStopsAndMarksReportInterrupted){
  transport::on_spin=[] {transport::running=false;};
  EXPECT_EQ(invoke({"--drive","--stamped","--duration",".1"}),1);
  EXPECT_TRUE(report["interrupted"]);EXPECT_FALSE(report["success"]);EXPECT_EQ(report["counts"]["commands"],0);
}
TEST_F(ClientUnits,RemapsAndAtomicOutputArePreserved){
  TemporaryDirectory temp("client-units-");auto file=temp.path()/"report.json";
  transport::on_spin=[this]{messages();};
  EXPECT_EQ(invoke({"--duration",".05","--output",file.string(),"--ros-args","-r","a:=b"}),0);
  EXPECT_EQ(Json::parse(read(file)),report);
  EXPECT_EQ(transport::ros_arguments,(std::vector<std::string>{"ros_client","--ros-args","-r","a:=b"}));
}
}
