CyberRT:

1. Talker-Listener
The first part of demonstrating CyberRT API is to understand the Talker/Listener example. 
Following are three essential concepts: node (basic unit), reader(facility to read message) and writer(facility to write message) of the example.

2. Create a node
In the CyberRT framework, the node is the most fundamental unit, similar to the role of a handle. When creating a specific functional object (writer, reader, etc.), 
you need to create it based on an existing node instance. The node creation interface is as follows:

std::unique_ptr<Node> apollo::cyber::CreateNode(const std::string& node_name, const std::string& name_space = "");

Parameters:
node_name: name of the node, globally unique identifier
name_space: name of the space where the node is located
name_space is empty by default. It is the name of the space concatenated with node_name. The format is /namespace/node_name
Return value - An exclusive smart pointer to Node
Error Conditions - when cyber::Init() has not called, the system is in an uninitialized state, unable to create a node, return nullptr

3.Create a writer
The writer is the basic facility used in CyberRT to send messages. Every writer corresponds to a channel with a specific data type. The writer is created by the CreateWriter interface in the node class. The interfaces are listed as below:

template <typename MessageT>
   auto CreateWriter(const std::string& channel_name)
       -> std::shared_ptr<Writer<MessageT>>;
template <typename MessageT>
   auto CreateWriter(const proto::RoleAttributes& role_attr)
       -> std::shared_ptr<Writer<MessageT>>;
Parameters:
channel_name: the name of the channel to write to
MessageT: The type of message to be written out
Return value - Shared pointer to the Writer object

4.Create a reader
The reader is the basic facility used in cyber to receive messages. Reader has to be bound to a callback function when it is created. When a new message arrives in the channel, the callback will be called. The reader is created by the CreateReader interface of the node class. The interfaces are listed as below:

template <typename MessageT>
auto CreateReader(const std::string& channel_name, const std::function<void(const std::shared_ptr<MessageT>&)>& reader_func)
    -> std::shared_ptr<Reader<MessageT>>;

template <typename MessageT>
auto CreateReader(const ReaderConfig& config,
                  const CallbackFunc<MessageT>& reader_func = nullptr)
    -> std::shared_ptr<cyber::Reader<MessageT>>;

template <typename MessageT>
auto CreateReader(const proto::RoleAttributes& role_attr,
                  const CallbackFunc<MessageT>& reader_func = nullptr)
-> std::shared_ptr<cyber::Reader<MessageT>>;
Parameters:

MessageT: The type of message to read
channel_name: the name of the channel to receive from
reader_func: callback function to process the messages
Return value - Shared pointer to the Reader object

5.Service Creation and Use
	Introduction
In an autonomous driving system, there are many scenarios that require more from module communication than just sending or receiving messages. Service is another way of communication between nodes. Unlike channel, service implements two-way communication, e.g. a node obtains a response by sending a request. This section introduces the service module in CyberRT API with examples.

Demo - Example
Problem: create a client-server model that pass Driver.proto back and forth. When a request is sent in by the client, the server parses/processes the request and returns the response.

The implementation of the demo mainly includes the following steps.






6. Apollo Cyber RT框架基于组件的概念构建、加载各功能模块。Localization、 Perception、Prediction、Planning、Control等功能模块均作为Apollo Cyber RT框架的一个组件而存在，基于Cyber RT提供的调度程序mainboard加载运行。
	基于Apollo Cyber RT框架创建和发布新的功能模块组件，需执行以下五个基本步骤：
																		1. 设置组件文件结构
																		2. 实现组件类
																		3. 提供构建文件
																		4. 提供配置文件
																		5. 启动组件



	1. 设置组件文件结构
			基于路径${APOLLO_HOME}/modules/planning（${APOLLO_HOME}表示Apollo项目的根目
																						头文件: planning_component.h
																						实现文件: planning_component.cc
																						构建文件: BUILD；
																						DAG配置文件: dag/planning.dag
																						Launch配置文件: launch/planning.launch



	2. 实现组件类


	apollo/modules/planning/planning_component.h

	namespace apollo {
	namespace planning {

	// 基于模板类Component派生出规划模块的组件类PlanningComponent
	class PlanningComponent final: public cyber::Component < prediction::PredictionObstacles, canbus::Chassis, localization::LocalizationEstimate > {    //C++ 11添加了两个继承控制关键字：override和final。override确保在派生类中声明的重载函数跟基类的虚函数有相同的签名。
																																					  // final阻止类的进一步派生和虚函数的进一步重载
	 public:
	  PlanningComponent() = default;

	  ~PlanningComponent() = default;

	 // 在派生类PlanningComponent中覆盖要虚函数Init() and Proc()函数
	 public:
	  bool Init() override;

	  bool Proc ( const std::shared_ptr<prediction::PredictionObstacles>&   prediction_obstacles,
	              const std::shared_ptr<canbus::Chassis>& chassis,
	              const std::shared_ptr<localization::LocalizationEstimate>& localization_estimate ) override;

	 private:
	  void CheckRerouting();
	  bool CheckInput();

	  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>> traffic_light_reader_;

	  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;

	  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_message_reader_;

	  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;

	  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;

	  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;

	  std::mutex mutex_;

	  perception::TrafficLightDetection traffic_light_;

	  routing::RoutingResponse routing_;

	  PadMessage pad_message_;

	  relative_map::MapMsg relative_map_;

	  LocalView local_view_;

	  std::unique_ptr<PlanningBase> planning_base_;

	  PlanningConfig config_;
	};

	CYBER_REGISTER_COMPONENT(PlanningComponent)	//使用宏CYBER_REGISTER_COMPONENT(PlanningComponent)注册组件类PlanningComponent，以便Cyber RT能正确创建并加载该类对象

	}  // namespace planning
	}  // namespace apollo



	// 基类Component的定义
	apollo/cyber/component/component.h

	template < typename M0 = NullType, typename M1 = NullType,typename M2 = NullType, typename M3 = NullType > //Component类最多接受4个模板参数，每个模板参数均表示一种输入的消息类型，这些消息在Proc函数中被周期性地接收并处理
	class Component : public ComponentBase {
											}


    // 而PlanningComponent继承的是该模板类接受3个参数的一个特化版本：
	template <typename M0, typename M1, typename M2>
	class Component<M0, M1, M2, NullType> : public ComponentBase {
																	}

    
	// PlanningComponent的实现主要包括两个覆盖的虚函数Init() and Proc()函数
    apollo/modules/planning/planning_component.cc

    // 其中Init()函数用于创建实际规划类对象，创建除prediction::PredictionObstacles、canbus::Chassis、localization::LocalizationEstimate三类消息以外的其他消息处理回调函数，
    // 创建Planning模块的输出器：轨迹输出器planning_writer_和重新生成路由输出器rerouting_writer_

	bool PlanningComponent::Init() {
	  if (FLAGS_use_navigation_mode) {
	    planning_base_ = std::make_unique<NaviPlanning>();
	  } else {
	    planning_base_ = std::make_unique<OnLanePlanning>();
	  }
	  
      CHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_planning_config_file, &config_)) << "failed to load planning config file " << FLAGS_planning_config_file;
	  planning_base_->Init(config_);

	  //直接创建消息接收器，一般用于接收非周期性消息或模块的次要输入消息
	  routing_reader_ = node_->CreateReader<RoutingResponse>(
	      FLAGS_routing_response_topic,
	      [this](const std::shared_ptr<RoutingResponse>& routing) {
	        AINFO << "Received routing data: run routing callback."
	              << routing->header().DebugString();
	        std::lock_guard<std::mutex> lock(mutex_);
	        routing_.CopyFrom(*routing);
	      });

	  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
	      FLAGS_traffic_light_detection_topic,
	      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
	        ADEBUG << "Received traffic light data: run traffic light callback.";
	        std::lock_guard<std::mutex> lock(mutex_);
	        traffic_light_.CopyFrom(*traffic_light);
	      });

	  if (FLAGS_use_navigation_mode) {
	    pad_message_reader_ = node_->CreateReader<PadMessage>(
	        FLAGS_planning_pad_topic,
	        [this](const std::shared_ptr<PadMessage>& pad_message) {
	          ADEBUG << "Received pad data: run pad callback.";
	          std::lock_guard<std::mutex> lock(mutex_);
	          pad_message_.CopyFrom(*pad_message);
	        });
	    relative_map_reader_ = node_->CreateReader<MapMsg>(
	        FLAGS_relative_map_topic,
	        [this](const std::shared_ptr<MapMsg>& map_message) {
	          ADEBUG << "Received relative map data: run relative map callback.";
	          std::lock_guard<std::mutex> lock(mutex_);
	          relative_map_.CopyFrom(*map_message);
	        });
	  }

	  planning_writer_ = node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);	// 轨迹输出器planning_writer_

	  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(FLAGS_routing_request_topic);	// 重新生成路由输出器rerouting_writer_

	  return true;
	}



	// Proc()函数周期性地接收prediction::PredictionObstacles、canbus::Chassis、localization::LocalizationEstimate三类消息，
	// 调用planning_base_->RunOnce()函数执行实际的路径与速度规划，并将规划结果adc_trajectory_pb借助函数planning_writer_->Write()将生成的规划轨迹输出给控制模块执行


	bool PlanningComponent::Proc(
	    const std::shared_ptr<prediction::PredictionObstacles>& prediction_obstacles,
	    const std::shared_ptr<canbus::Chassis>& chassis,
	    const std::shared_ptr<localization::LocalizationEstimate>& localization_estimate) {
	  CHECK(prediction_obstacles != nullptr);

	  // check and process possible rerouting request
	  CheckRerouting();

	  // process fused input data
	  local_view_.prediction_obstacles = prediction_obstacles;
	  local_view_.chassis = chassis;
	  local_view_.localization_estimate = localization_estimate;
	  {
	    std::lock_guard<std::mutex> lock(mutex_);
	    if (!local_view_.routing ||
	        hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
	      local_view_.routing = std::make_shared<routing::RoutingResponse>(routing_);
	    }
	  }
	  {
	    std::lock_guard<std::mutex> lock(mutex_);
	    local_view_.traffic_light = std::make_shared<TrafficLightDetection>(traffic_light_);
	    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
	  }

	  if (!CheckInput()) {
	    AERROR << "Input check failed";
	    return false;
	  }

	  ADCTrajectory adc_trajectory_pb;
	  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);		//planning_base_->RunOnce()函数执行实际的路径与速度规划 //adc_trajectory_pb:规划结果
	  auto start_time = adc_trajectory_pb.header().timestamp_sec();
	  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

	  // modify trajectory relative time due to the timestamp change in header
	  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
	  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
	    p.set_relative_time(p.relative_time() + dt);
	  }
	  planning_writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));	//planning_writer_->Write()将生成的规划轨迹输出给控制模块执行
	  return true;
	}





	3. 提供构建文件
		/apollo/modules/planning/BUILD

		/apollo/modules/planning/BUILD文件中与Cyber RT相关的内容，可见基于planning_component_lib库最终生成了一个共享库文件libplanning_component.so，而该共享库通过Cyber RT调度程序mainboard动态加载运行


	4. 提供配置文件

		/apollo/dag/planning.dag
		DAG配置文件是Cyber RT调度程序mainboard动态加载Planning模块的最终配置文件，加载命令一般为：

		/apollo/bazel-bin/cyber/mainboard -d /apollo/modules/planning/dag/planning.dag

		# Define all coms in DAG streaming.
		module_config {
		  module_library : "/apollo/bazel-bin/modules/planning/libplanning_component.so"	// 共享库文件路径
		  components {
		    class_name : "PlanningComponent"	//  组件类名称，一定不能写错，否则mainboard无法动态创建PlanningComponent组件对象
		    config {
		      name: "planning"	// 模块名
		      flag_file_path:  "/apollo/modules/planning/conf/planning.conf"   // GFlag配置文件路径，注意路径一定写成绝对路径，否则可能无法找到配置文件，导致模块加载失败
		      
		      // # PlanningComponent组件Proc()函数中使用的三个消息接收器
		      readers: [
		        {
		          channel: "/apollo/prediction"
		        },
		        {
		          channel: "/apollo/canbus/chassis"
		          qos_profile: {
		              depth : 15
		          }
		          pending_queue_size: 50
		        },
		        {
		          channel: "/apollo/localization/pose"
		          qos_profile: {
		              depth : 15
		          }
		          pending_queue_size: 50
		        }
		      ]
		    }
		  }
		}



	5. 启动组件
	/apollo/launch/planning.launch
	+Launch配置文件是Cyber RT提供的一个Python工具程序cyber_launch加载Planning模块所需的配置文件，启动命令如下所示（最终仍归结于mainboard加载）：

	<cyber>
	    <module>
	        <name>planning</name>
	        <dag_conf>/apollo/modules/planning/dag/planning.dag</dag_conf>
	        <process_name>planning</process_name>
	    </module>
	</cyber>




7.  如何发布消息？
	基于Cyber RT发布消息非常直观，首先创建发布器对象，然后填充消息，最后发布消息

	// // 1.创建发布器
	planning_writer_ = node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);	// 轨迹输出器planning_writer_

	// 2.填充消息
	  ADCTrajectory adc_trajectory_pb;
	  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
	  auto start_time = adc_trajectory_pb.header().timestamp_sec();
	  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

	  // modify trajecotry relative time due to the timestamp change in header
	  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
	  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
	    p.set_relative_time(p.relative_time() + dt);
	  }


	  // 3.发布消息
 	 planning_writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));



8.如何接收消息？


	如何接收消息？
	基于Cyber RT接收消息分两种情形，第一种是1.2.1节描述的在虚函数PlanningComponent::Proc()中处理指定的消息类型，这类消息是周期性触发，但最多只能接收4种（因为cyber::Component的模板参数最多只有4个），一般用于模块主要输入消息的接收。
	第二种是直接创建消息接收器，一般用于接收非周期性消息或模块的次要输入消息，示例代码如下，注意消息处理回调函数均以Lambda表达式的方式展现：

	 routing_reader_ = node_->CreateReader<RoutingResponse>(
	      FLAGS_routing_response_topic,
	      [this](const std::shared_ptr<RoutingResponse>& routing) {
	        AINFO << "Received routing data: run routing callback."
	              << routing->header().DebugString();
	        std::lock_guard<std::mutex> lock(mutex_);
	        routing_.CopyFrom(*routing);
	      });
	 
	  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>(
	      FLAGS_traffic_light_detection_topic,
	      [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
	        ADEBUG << "Received traffic light data: run traffic light callback.";
	        std::lock_guard<std::mutex> lock(mutex_);
	        traffic_light_.CopyFrom(*traffic_light);
	      });
