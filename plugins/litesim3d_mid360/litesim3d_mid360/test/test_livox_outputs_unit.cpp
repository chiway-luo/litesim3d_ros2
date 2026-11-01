// Serializer units: all observed messages, topic resolution and match counts
// use recording capabilities. No sensor kernel, driver, peer or executor runs.
#include "../src/livox_outputs.hpp"
#include <gtest/gtest.h>
#include <cstring>
#include <limits>

namespace {
using litesim3d_ros::LivoxOutputs;
using litesim3d::Json;
using CloudMessage=sensor_msgs::msg::PointCloud2;
using CustomMessage=livox_ros_driver2::msg::CustomMsg;
struct Capture {
  std::vector<std::pair<std::string,CloudMessage>> clouds;
  std::vector<std::pair<std::string,CustomMessage>> custom;
  std::vector<std::string> created;
  std::vector<std::string> published;
  bool collide=false;
  LivoxOutputs::Transport transport() {
    return {
      [this](rclcpp::Node &,const std::string &name){return collide?"/collision":"/"+name;},
      [this](rclcpp::Node &,const std::string &name,const rclcpp::QoS &qos,const rclcpp::PublisherOptions &){
        EXPECT_EQ(qos.get_rmw_qos_profile().depth,256u);
        EXPECT_EQ(qos.get_rmw_qos_profile().reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        created.push_back(name);
        return LivoxOutputs::CloudOutput{name,[this,name](const CloudMessage &m){published.push_back("cloud:"+name);clouds.emplace_back(name,m);},[]{return 7u;}};
      },
      [this](rclcpp::Node &,const std::string &name,const rclcpp::QoS &qos,const rclcpp::PublisherOptions &){
        EXPECT_EQ(qos.get_rmw_qos_profile().depth,256u);
        EXPECT_EQ(qos.get_rmw_qos_profile().reliability,RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        created.push_back(name);
        return LivoxOutputs::CustomOutput{name,[this,name](const CustomMessage &m){published.push_back("custom:"+name);custom.emplace_back(name,m);},[]{return 3u;}};
      }};
  }
};
class LivoxUnits : public testing::Test {
protected:
  static void SetUpTestSuite(){rclcpp::init(0,nullptr);}
  static void TearDownTestSuite(){rclcpp::shutdown();}
  rclcpp::Node node{"livox_serializer_unit"};
  Capture capture;
  litesim3d::Config config;
  void SetUp() override {
    litesim3d::LidarConfig lidar; lidar.name="first"; lidar.link="optical";
    lidar.topic="source"; lidar.scan_pattern="livox_mid360";
    lidar.scan_mode="rolling"; lidar.columns=4; lidar.rate=10;
    config.lidars={lidar};
  }
  Json settings(int format=1,int id=0,const std::string &interface="livox") {
    return {{"lidar_interface",interface},{"xfer_format",format},{"livox_lidar_id",id}};
  }
  static litesim3d::Cloud input() {
    litesim3d::Cloud cloud; cloud.topic="source"; cloud.frame="optical";
    cloud.stamp=2000000011; cloud.width=4; cloud.height=2; cloud.row_step=100;
    cloud.frame_index=3; cloud.data.resize(cloud.row_step*cloud.height,0xff);
    const auto nan=std::numeric_limits<float>::quiet_NaN();
    const float rows[8][4]={{1,2,3,-1},{nan,2,3,.2},{1,nan,3,.3},{1,2,nan,.4},
      {-4,5,6,2},{7,8,9,.5},{10,11,12,0},{13,14,15,1}};
    for(size_t i=0;i<8;++i) for(size_t f=0;f<4;++f) {
      uint32_t bits; std::memcpy(&bits,&rows[i][f],4);
      for(unsigned byte=0;byte<4;++byte) cloud.data[(i/4)*100+(i%4)*24+f*4+byte]=uint8_t(bits>>(8*byte));
    }
    cloud.range_boundary=cloud.data;
    return cloud;
  }
};

template<class T> T number(const std::vector<uint8_t> &data,size_t offset) {
  using Bits=std::conditional_t<sizeof(T)==4,uint32_t,uint64_t>;
  Bits bits=0; for(size_t i=0;i<sizeof(T);++i) bits|=Bits(data.at(offset+i))<<(8*i);
  T value; std::memcpy(&value,&bits,sizeof(value)); return value;
}

TEST_F(LivoxUnits, SelectorsAndIdBudgetsRejectBeforeCreatingPublishers) {
  LivoxOutputs output(capture.transport());
  EXPECT_THROW(output.initialize(node,config,"","",settings(1,0,"bad"),{}),std::runtime_error);
  for(int format:{-1,2}) EXPECT_THROW(output.initialize(node,config,"","",settings(format),{}),std::runtime_error);
  for(int id:{-1,256}) EXPECT_THROW(output.initialize(node,config,"","",settings(1,id),{}),std::runtime_error);
  auto none=config; none.lidars.clear();
  EXPECT_THROW(output.initialize(node,none,"","",settings(),{}),std::runtime_error);
  auto two=config; two.lidars.push_back(two.lidars[0]);
  EXPECT_THROW(output.initialize(node,two,"","",settings(1,255),{}),std::runtime_error);
  EXPECT_TRUE(capture.created.empty());
  output.initialize(node,none,"","",settings(0,255,"auto"),{});
  EXPECT_TRUE(output.description().at("lidars").empty());
  EXPECT_FALSE(output.publish(input()));
  EXPECT_FALSE(output.handles("source"));
}

TEST_F(LivoxUnits, NativeSkipsMid360AndAutoSkipsOtherPatternsWithoutLoadingOutputs) {
  LivoxOutputs output(capture.transport());
  output.initialize(node,config,"","",settings(0,255,"native"),{});
  EXPECT_FALSE(output.handles("source")); EXPECT_TRUE(capture.created.empty());
  config.lidars[0].scan_pattern="spinning";
  output.initialize(node,config,"","",settings(0,255,"auto"),{});
  EXPECT_FALSE(output.handles("source")); EXPECT_TRUE(capture.created.empty());
  auto matches=Json::object(); output.matches(matches); EXPECT_TRUE(matches.empty());
}

TEST_F(LivoxUnits, CustomAndCloudStreamsPreserveAbiSlotsClampsAndNanFiltering) {
  config.lidars[0].publish_range_boundary=true;
  // Row offsets use per-ring timing even though this small serialization
  // fixture does not run an actual Mid-360 sampler.
  config.lidars[0].ring_delays={0,.001};
  LivoxOutputs output(capture.transport());
  output.initialize(node,config,"robot/","robot/",settings(1,255),{});
  ASSERT_EQ(capture.created.size(),3u);
  const auto &description=output.description().at("lidars").at(0);
  EXPECT_EQ(description.at("topic"),"/robot/livox/lidar");
  EXPECT_EQ(description.at("pointcloud_topic"),"/robot/livox/lidar/pointcloud");
  EXPECT_EQ(description.at("frame_id"),"robot/optical");
  EXPECT_EQ(description.at("lidar_id"),255);
  EXPECT_EQ(description.at("message_type"),"livox_ros_driver2/msg/CustomMsg");
  auto cloud=input(); ASSERT_TRUE(output.publish(cloud));
  ASSERT_EQ(capture.clouds.size(),2u); ASSERT_EQ(capture.custom.size(),1u);
  EXPECT_EQ(capture.published,(std::vector<std::string>{"custom:/robot/livox/lidar",
      "cloud:/robot/livox/lidar/pointcloud","cloud:/robot/livox/lidar/range_boundary"}));
  const auto &points=capture.clouds[0].second;
  EXPECT_EQ(points.header.stamp.sec,2); EXPECT_EQ(points.header.stamp.nanosec,11u);
  EXPECT_EQ(points.header.frame_id,"robot/optical");
  EXPECT_EQ(points.height,1u); EXPECT_EQ(points.width,5u);
  EXPECT_EQ(points.point_step,26u); EXPECT_EQ(points.row_step,130u);
  EXPECT_FALSE(points.is_bigendian); EXPECT_TRUE(points.is_dense);
  const char *names[]={"x","y","z","intensity","tag","line","timestamp"};
  const uint32_t offsets[]={0,4,8,12,16,17,18}; const uint8_t types[]={7,7,7,7,2,2,8};
  ASSERT_EQ(points.fields.size(),7u);
  for(unsigned i=0;i<7;++i) {
    EXPECT_EQ(points.fields[i].name,names[i]); EXPECT_EQ(points.fields[i].offset,offsets[i]);
    EXPECT_EQ(points.fields[i].datatype,types[i]); EXPECT_EQ(points.fields[i].count,1u);
  }
  const size_t slots[]={0,4,5,6,7}; const float reflectivity[]={0,255,128,0,255};
  const float expected_x[]={1,-4,7,10,13},expected_y[]={2,5,8,11,14},expected_z[]={3,6,9,12,15};
  const auto &custom=capture.custom[0].second;
  EXPECT_EQ(custom.header,points.header); EXPECT_EQ(custom.timebase,cloud.stamp);
  EXPECT_EQ(custom.lidar_id,255); ASSERT_EQ(custom.point_num,5u);
  EXPECT_EQ(custom.rsvd,(std::array<uint8_t,3>{}));
  ASSERT_EQ(custom.points.size(),5u);
  for(size_t i=0;i<5;++i) {
    const auto slot=slots[i],r=slot/4,col=slot%4;
    EXPECT_FLOAT_EQ(number<float>(points.data,i*26+12),reflectivity[i]);
    EXPECT_EQ(points.data[i*26+16],0u);
    EXPECT_EQ(points.data[i*26+17],uint8_t((cloud.frame_index*cloud.width*cloud.height+slot)%4));
    const auto dt=config.lidars[0].offsetNs(r,col);
    EXPECT_DOUBLE_EQ(number<double>(points.data,i*26+18),double(cloud.stamp+dt));
    EXPECT_EQ(custom.points[i].offset_time,dt);
    EXPECT_EQ(custom.points[i].reflectivity,reflectivity[i]);
    EXPECT_EQ(custom.points[i].line,points.data[i*26+17]);
    EXPECT_EQ(custom.points[i].tag,0);
    EXPECT_FLOAT_EQ(number<float>(points.data,i*26),expected_x[i]);
    EXPECT_FLOAT_EQ(number<float>(points.data,i*26+4),expected_y[i]);
    EXPECT_FLOAT_EQ(number<float>(points.data,i*26+8),expected_z[i]);
    EXPECT_FLOAT_EQ(custom.points[i].x,expected_x[i]);
    EXPECT_FLOAT_EQ(custom.points[i].y,expected_y[i]);
    EXPECT_FLOAT_EQ(custom.points[i].z,expected_z[i]);
    EXPECT_FLOAT_EQ(custom.points[i].x,number<float>(points.data,i*26));
    EXPECT_FLOAT_EQ(custom.points[i].y,number<float>(points.data,i*26+4));
    EXPECT_FLOAT_EQ(custom.points[i].z,number<float>(points.data,i*26+8));
  }
  EXPECT_EQ(capture.clouds[1].first,"/robot/livox/lidar/range_boundary");
  EXPECT_EQ(capture.clouds[1].second.data,points.data);
  auto matches=Json::object(); output.matches(matches);
  EXPECT_EQ(matches.at("/robot/livox/lidar"),3);
  EXPECT_EQ(matches.at("/robot/livox/lidar/pointcloud"),7);
  EXPECT_EQ(matches.at("/robot/livox/lidar/range_boundary"),7);
}

TEST_F(LivoxUnits, MultipleSensorsHaveIndependentIdsTopicsAndPointcloudOnlyMode) {
  config.lidars.push_back(config.lidars[0]);
  config.lidars[1].name="second"; config.lidars[1].topic="other";
  config.lidars[0].scan_mode="snapshot"; config.lidars[1].scan_mode="snapshot";
  LivoxOutputs output(capture.transport()); output.initialize(node,config,"","",settings(0,254),{});
  ASSERT_EQ(capture.created.size(),2u);
  EXPECT_EQ(output.description().at("lidars").at(0).at("lidar_id"),254);
  EXPECT_EQ(output.description().at("lidars").at(1).at("lidar_id"),255);
  EXPECT_EQ(capture.created[0],"/livox/first/lidar"); EXPECT_EQ(capture.created[1],"/livox/second/lidar");
  EXPECT_EQ(output.description().at("lidars").at(0).at("message_type"),"sensor_msgs/msg/PointCloud2");
  auto cloud=input(); ASSERT_TRUE(output.publish(cloud)); cloud.topic="other"; ASSERT_TRUE(output.publish(cloud));
  EXPECT_TRUE(capture.custom.empty()); ASSERT_EQ(capture.clouds.size(),2u);
  for(const auto &entry:capture.clouds) for(size_t i=0;i<entry.second.width;++i)
    EXPECT_DOUBLE_EQ(number<double>(entry.second.data,i*26+18),double(cloud.stamp));
  auto matches=Json::object(); output.matches(matches); EXPECT_EQ(matches.size(),2u);
  // Reinitialization must discard ownership of old topics and old metadata.
  config.lidars.clear(); output.initialize(node,config,"","",settings(0,0,"auto"),{});
  EXPECT_FALSE(output.handles("source")); EXPECT_TRUE(output.description().at("lidars").empty());
}

TEST_F(LivoxUnits, CollisionsOffsetsMalformedBuffersAndTimestampOverflowFailExplicitly) {
  capture.collide=true; config.lidars[0].publish_range_boundary=true;
  LivoxOutputs collision(capture.transport());
  EXPECT_THROW(collision.initialize(node,config,"","",settings(),{}),std::runtime_error);
  capture.collide=false; capture.created.clear(); config.lidars[0].publish_range_boundary=false;
  config.lidars[0].rate=.001;
  LivoxOutputs slow(capture.transport());
  EXPECT_THROW(slow.initialize(node,config,"","",settings(),{}),std::runtime_error);
  EXPECT_TRUE(capture.created.empty());
  config.lidars[0].rate=10;
  LivoxOutputs output(capture.transport()); output.initialize(node,config,"","",settings(0),{});
  auto cloud=input(); cloud.data.resize(187);
  EXPECT_THROW(output.publish(cloud),std::out_of_range);
  EXPECT_TRUE(capture.clouds.empty());
  cloud=input(); cloud.stamp=std::numeric_limits<uint64_t>::max();
  EXPECT_THROW(output.publish(cloud),std::runtime_error);
  EXPECT_TRUE(capture.clouds.empty());
  cloud=input(); cloud.data.clear(); cloud.range_boundary.clear(); cloud.width=cloud.height=0;
  EXPECT_TRUE(output.publish(cloud)); ASSERT_EQ(capture.clouds.size(),1u);
  EXPECT_TRUE(capture.clouds[0].second.data.empty()); EXPECT_EQ(capture.clouds[0].second.row_step,0u);
}

TEST_F(LivoxUnits, DefaultSdkAdapterRetainsLocalTopicAndQosContractWithoutAnyPeer) {
  // This tests only the SDK adapter wiring: no subscriber, driver, remote peer
  // or executor exists, so no delivery/integration claim follows from it.
  config.lidars[0].scan_mode="snapshot";
  config.lidars[0].publish_range_boundary=true;
  for(int format:{0,1}) {
    LivoxOutputs output;
    output.initialize(node,config,"unit/","frame/",settings(format),{});
    auto cloud=input(); EXPECT_TRUE(output.publish(cloud));
    const auto &record=output.description().at("lidars").at(0);
    EXPECT_EQ(record.at("topic"),"/unit/livox/lidar");
    auto matches=Json::object(); output.matches(matches);
    for(const auto &value:matches) EXPECT_EQ(value,0);
  }
}
}  // namespace
