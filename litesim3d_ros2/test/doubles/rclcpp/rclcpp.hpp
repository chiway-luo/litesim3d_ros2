#pragma once
#define LITESIM_UNIT_ROS_TRANSPORT 1
// Test-only transport boundary; callbacks use real ROS message types.
#include <any>
#include <chrono>
#include <functional>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include <builtin_interfaces/msg/time.hpp>
#include <rmw/qos_profiles.h>
namespace rclcpp {
class Context {
public:
  using SharedPtr=std::shared_ptr<Context>;bool valid=false;std::string shutdown_reason;
  void init(int,const char*const*){valid=true;}
  bool is_valid()const{return valid;}
  void shutdown(const std::string&reason){valid=false;shutdown_reason=reason;}
};
struct Parameter {
  std::string name; std::any value;
  Parameter(std::string n,const char*v):name(std::move(n)),value(std::string(v)){}
  template<class T> Parameter(std::string n,T v):name(std::move(n)),value(std::move(v)){}
};
struct NodeOptions {
  Context::SharedPtr ctx;
  NodeOptions& context(Context::SharedPtr c){ctx=std::move(c);return *this;}
  NodeOptions& use_global_arguments(bool){return *this;}
  std::vector<Parameter> parameters;
  NodeOptions& parameter_overrides(std::vector<Parameter> p){parameters=std::move(p);return *this;}
};
struct InitOptions{}; enum class SignalHandlerOptions { None };
struct ExecutorOptions{Context::SharedPtr context;};
enum class CallbackGroupType { MutuallyExclusive };
struct CallbackGroup { using SharedPtr=std::shared_ptr<CallbackGroup>; };
struct SubscriptionOptions { CallbackGroup::SharedPtr callback_group; };
struct QoS {
  rmw_qos_profile_t profile=rmw_qos_profile_default;
  QoS(int=10){}
  QoS& reliable(){profile.reliability=RMW_QOS_POLICY_RELIABILITY_RELIABLE;return *this;}
  QoS& best_effort(){profile.reliability=RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;return *this;}
  QoS& transient_local(){return *this;}
  const rmw_qos_profile_t& get_rmw_qos_profile()const{return profile;}
};
struct SensorDataQoS:QoS{};
struct TopicEndpointInfo {
  std::string name,ns="/",type="type";QoS qos;
  const std::string& node_name()const{return name;}
  const std::string& node_namespace()const{return ns;}
  const std::string& topic_type()const{return type;}
  const QoS& qos_profile()const{return qos;}
};
namespace unit {
inline Context::SharedPtr active_context;
inline bool running=true; inline int64_t now_ns=1000000000;
inline size_t subscriber_count=1,spins=0,removed=0,request_count=0;
inline bool service_ready=true,response_ready=true,response_throws=false;
inline std::function<void()> on_spin,on_wait,on_service;
inline std::map<std::string,std::function<void(std::shared_ptr<void>)>> subscriptions;
inline std::map<std::string,std::vector<std::any>> publications;
inline std::map<std::string,std::vector<TopicEndpointInfo>> subscribers,publishers;
inline std::map<std::string,std::vector<std::string>> services;
inline std::vector<std::string> ros_arguments; inline std::string node_namespace;
inline std::vector<Parameter> node_parameters;
template<class M> void deliver(const std::string&topic,const M&message) {
  auto found=subscriptions.find(topic);
  if(found!=subscriptions.end())found->second(std::make_shared<M>(message));
}
inline void reset(){
  running=true;now_ns=1000000000;subscriber_count=1;spins=removed=request_count=0;
  service_ready=response_ready=true;response_throws=false;on_spin=on_wait=on_service={};
  subscriptions.clear();publications.clear();subscribers.clear();publishers.clear();services.clear();
  ros_arguments.clear();node_namespace.clear();node_parameters.clear();active_context.reset();
}
}
inline bool ok(){return unit::running;}
inline void shutdown(){unit::running=false;}
inline void init(int argc,const char*const*argv,const InitOptions& = {},SignalHandlerOptions=SignalHandlerOptions::None){
  unit::running=true;for(int i=0;i<argc;++i)unit::ros_arguments.emplace_back(argv[i]);
}
struct Time {
  int64_t value;int64_t nanoseconds()const{return value;}
  operator builtin_interfaces::msg::Time()const{
    builtin_interfaces::msg::Time t;t.sec=value/1000000000;t.nanosec=value%1000000000;return t;
  }
};
struct MessageInfo {};
template<class M>struct Subscription {
  using SharedPtr=std::shared_ptr<Subscription>;
  std::function<void(std::shared_ptr<void>)> callback;
  void handle_message(std::shared_ptr<void> message,const MessageInfo&){callback(std::move(message));}
};
template<class M>struct Publisher {
  using SharedPtr=std::shared_ptr<Publisher>;std::string topic;
  void publish(const M&m){unit::publications[topic].emplace_back(m);}
  size_t get_subscription_count()const{return unit::subscriber_count;}
};
template<class S>struct Service {
  using SharedPtr=std::shared_ptr<Service>;
  std::function<void(typename S::Request::SharedPtr,typename S::Response::SharedPtr)> callback;
};
template<class S>struct Client {
  using SharedPtr=std::shared_ptr<Client>;
  struct Future {
    std::future_status wait_for(std::chrono::seconds) {
      if(unit::on_wait)unit::on_wait();
      return unit::response_ready?std::future_status::ready:std::future_status::timeout;
    }
    typename S::Response::SharedPtr get(){
      if(unit::response_throws)throw std::runtime_error("response error");
      return std::make_shared<typename S::Response>();
    }
  };
  bool service_is_ready(){if(unit::on_service)unit::on_service();return unit::service_ready;}
  Future async_send_request(typename S::Request::SharedPtr){++unit::request_count;return {};}
  void remove_pending_request(Future&){++unit::removed;}
};
struct NodeBaseInterface {
  Context::SharedPtr context;
  const Context::SharedPtr& get_context()const{return context;}
};
class Node {
  std::vector<Parameter> parameters_;
  std::shared_ptr<NodeBaseInterface> base_=std::make_shared<NodeBaseInterface>();
public:
  explicit Node(const std::string&,const std::string&ns="",const NodeOptions&options={}){
    unit::node_namespace=ns;unit::node_parameters=options.parameters;unit::active_context=options.ctx;parameters_=options.parameters;base_->context=options.ctx;
  }
  Node(const std::string&name,const NodeOptions&options):Node(name,"",options){}
  virtual ~Node()=default;
  std::shared_ptr<NodeBaseInterface> get_node_base_interface(){return base_;}
  template<class T>T declare_parameter(const std::string&name,const T&fallback){
    for(const auto&p:parameters_)if(p.name==name)return std::any_cast<T>(p.value);
    return fallback;
  }
  CallbackGroup::SharedPtr create_callback_group(CallbackGroupType){return std::make_shared<CallbackGroup>();}
  template<class M,class Q,class F>
  typename Subscription<M>::SharedPtr create_subscription(
      const std::string&topic,Q,F callback,const SubscriptionOptions& = {}){
    unit::subscriptions[topic]=[callback](std::shared_ptr<void>p){callback(std::static_pointer_cast<const M>(p));};
    auto sub=std::make_shared<Subscription<M>>();sub->callback=unit::subscriptions.at(topic);return sub;
  }
  template<class M>typename Publisher<M>::SharedPtr create_publisher(const std::string&topic,int){
    auto p=std::make_shared<Publisher<M>>();p->topic=topic;return p;
  }
  template<class S,class F>typename Service<S>::SharedPtr create_service(
      const std::string&,F f,const rmw_qos_profile_t&,CallbackGroup::SharedPtr){
    auto p=std::make_shared<Service<S>>();p->callback=f;return p;
  }
  template<class S>typename Client<S>::SharedPtr create_client(
      const std::string&,const rmw_qos_profile_t&,CallbackGroup::SharedPtr){
    return std::make_shared<Client<S>>();
  }
  std::vector<TopicEndpointInfo> get_subscriptions_info_by_topic(const std::string&t){return unit::subscribers[t];}
  std::vector<TopicEndpointInfo> get_publishers_info_by_topic(const std::string&t){return unit::publishers[t];}
  std::map<std::string,std::vector<std::string>> get_service_names_and_types_by_node(
      const std::string&n,const std::string&){
    std::map<std::string,std::vector<std::string>> result;
    for(const auto&s:unit::services[n])result[s]={};return result;
  }
  Time now()const{return {unit::now_ns};}
};
namespace executors {
struct SingleThreadedExecutor {
  template<class...T>explicit SingleThreadedExecutor(T&&...){}
  void add_node(std::shared_ptr<Node>){}
  void remove_node(std::shared_ptr<Node>){}
  void spin_once(std::chrono::milliseconds){++unit::spins;if(unit::on_spin)unit::on_spin();}
  void spin_some(){++unit::spins;if(unit::on_spin)unit::on_spin();}
};
struct MultiThreadedExecutor:SingleThreadedExecutor {
  using SingleThreadedExecutor::SingleThreadedExecutor;
  void spin(){if(unit::on_spin)unit::on_spin();}
  void cancel(){}
};
}
}
