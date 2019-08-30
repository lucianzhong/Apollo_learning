
Reference: https://paul.pub/apollo-routing/
//  r 3.5.0
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

1. //Introduction
	The Routing module generates high level navigation information based on requests.
	The Routing module depends on a routing topology file, usually named routing_map.* in apollo.
	The routing map can be genreated using this command:
	bash scripts/generate_routing_topo_graph.sh
	Input:
	Map data
	Routing request (Start and end location)
	Output:
	Routing navigation information

2.  // 常量定义
	// modules/routing/common/routing_gflags.h

	DECLARE_string(routing_conf_file);
	DECLARE_string(routing_node_name);
	DECLARE_string(routing_adapter_config_filename);
	DECLARE_double(min_length_for_lane_change);
	DECLARE_bool(enable_change_lane_in_result);


	routing_conf_file：Routing模块配置文件的路径。
	routing_node_name：Routing模块的节点名称。
	min_length_for_lane_change：在变道前，在当前车道上行驶的最短距离。
	enable_change_lane_in_result：导航结果是否允许变道。
	routing_response_history_interval_ms：路由请求的响应时长。


	Routing模块的代码中通过FLAGS_routing_conf_file便可以读取到值“/apollo/modules/routing/conf/routing_config.pb.txt“。




2.// Proto数据结构
	Apollo项目中的很多数据结构都是通过Protocol Buffers定义的。所以你看不到这些类的C++文件，因为C++需要的相关文件是在编译时通过proto文件自动生成的。
	Protocol Buffers是Google的开源项目。它具有语言无关，平台无关的特性，并且有很好的可扩展性。Protocol Buffers通常用于序列化结构化数据。
	Apollo使用Protocol Buffers的一个很重要的作用是，用它来将结构化数据导出到物理文件中，并且也可以很方便的从物理文件中读取信息。例如，Routing模块需要的Topo地图就是proto结构导出的。另外，如果导出的是文本形式的文件，也可以方便的进行人为的修改。
	例如，上面提到的routing_config.pb.txt。
	proto文件都位于名称为proto的文件夹中，你可以通常下面这条命令在apollo源码的根目录下找到所有的proto文件夹：

	apollo$ find . -name proto
	这其中自然就包含了Routing模块的proto文件夹：modules/routing/proto
	Routing proto:
    // /apolloauto/modules/routing/proto:

	// poi.proto
	 Point of interest的缩写，一个POI中可以包含多个Landmark

	 message Landmark {
	  optional string name = 1;
	  repeated LaneWaypoint waypoint = 2;
	  optional string parking_space_id = 3;
	}

	message POI {
	  repeated Landmark landmark = 1;    //地图上的一个点，包含了名称和位置信息
	}


	// routing_config.proto
	描述了Routing模块的配置信息，上面提到的routing_config.pb.txt文件就是这个格式的

	message RoutingConfig {
	  optional double base_speed = 1;  // base speed for node creator [m/s]
	  optional double left_turn_penalty = 2;  // left turn penalty for node creater [m]
	  optional double right_turn_penalty = 3;  // right turn penalty for node creater [m]
	  optional double uturn_penalty = 4;  // left turn penalty for node creater [m]
	  optional double change_penalty = 5;  // change penalty for edge creater [m]
	  optional double base_changing_length = 6;  // base change length penalty for edge creater [m]
	}


	//  routing.proto

	类型名称	描述
	LaneWaypoint	道路上的路径点，包含了id，长度和位置点信息。
	LaneSegment	道路的一段，包含了id和起止点信息。
	RoutingRequest	描述了路由请求的信息，一次路由请求可以包含多个路径点。详细结构见下文。
	Measurement	描述测量的距离。
	ChangeLaneType	道路的类型，有FORWARD，LEFT，RIGHT三种取值。
	Passage	一段通路，其中可以包含多个LaneSegment，以及ChangeLaneType。
	RoadSegment	道路的一段，拥有一个id，并可以包含多个Passage。
	RoutingResponse	路由请求的响应结果，可以包含多个RoadSegment，距离等信息

		// LaneWaypoint	道路上的路径点，包含了id，长度和位置点信息
		message LaneWaypoint {
		  optional string id = 1;
		  optional double s = 2;
		  optional apollo.common.PointENU pose = 3;
		}

		// LaneSegment	道路的一段，包含了id和起止点信息
		message LaneSegment {
		  optional string id = 1;
		  optional double start_s = 2;
		  optional double end_s = 3;
		}

		// RoutingRequest	描述了路由请求的信息，一次路由请求可以包含多个路径点
		message RoutingRequest {
		  optional apollo.common.Header header = 1;
		  // at least two points. The first is start point, the end is final point. The routing must go through each point in waypoint.
		  repeated LaneWaypoint waypoint = 2;	//它描述了一次路由请求的路径点，repeated表示这个数据可以出现多次，因此是Routing模块是支持一次搜索多个途经点的
		  repeated LaneSegment blacklisted_lane = 3;
		  repeated string blacklisted_road = 4;
		  optional bool broadcast = 5 [default = true];
		  optional apollo.hdmap.ParkingSpace parking_space = 6;
		}


		// Measurement	描述测量的距离
		message Measurement {
		  optional double distance = 1;
		}

		// ChangeLaneType	道路的类型，有FORWARD，LEFT，RIGHT三种取值
		enum ChangeLaneType {
		  FORWARD = 0;
		  LEFT = 1;
		  RIGHT = 2;
		};

		// Passage	一段通路，其中可以包含多个LaneSegment，以及ChangeLaneType
		message Passage {
		  repeated LaneSegment segment = 1;	//描述车道，车道是道路中的一段，自动驾驶车辆会尽可能沿着车道的中心线行驶
		  optional bool can_exit = 2;
		  optional ChangeLaneType change_lane_type = 3 [default = FORWARD];
		}

		// RoadSegment	道路的一段，拥有一个id，并可以包含多个Passage  // 描述道路，一条道路可能包含了并行的几条通路（Passage）
		message RoadSegment {
		  optional string id = 1;
		  repeated Passage passage = 2;	//描述通路，通路是直连不含变道的可行驶区域。一个通路可能包含了前后连接的多个车道
		}

		// RoutingResponse	路由请求的响应结果，可以包含多个RoadSegment，距离等信息
		message RoutingResponse {
		  optional apollo.common.Header header = 1; //消息头
		  repeated RoadSegment road = 2;			//具体的路径信息，最重要的数据 // 这个数据其实是一个三层的结构体嵌套
		  optional Measurement measurement = 3;		//距离
		  optional RoutingRequest routing_request = 4;	//原始请求
		  // the map version which is used to build road graph
		  optional bytes map_version = 5;		//地图版本
		  optional apollo.common.StatusPb status = 6;  //状态位
		}



	// topo_graph.proto	
	类型名称	描述
	CurvePoint	曲线上的一个点。
	CurveRange	曲线上的一段。
	Node	车道上的一个节点，包含了所属车道，道路，长度，曲线起止点，中心线等信息。
	Edge	连接车道之间的边，包含了起止车道id，代价和方向等信息。
	Graph	完整地图的Topo结构，这其中包含了多个Node和Edge。

	// CurvePoint	曲线上的一个点
	message CurvePoint {
	  optional double s = 1;
	}

	// CurveRange	曲线上的一段
	message CurveRange {
	  optional CurvePoint start = 1;
	  optional CurvePoint end = 2;
	}

	// Node	车道上的一个节点，包含了所属车道，道路，长度，曲线起止点，中心线等信息
	// NODE - 包括车道唯一id，长度，左边出口，右边出口（这里的出口对应车道虚线的部分，或者自己定义的一段允许变道的路段），路段代价（限速或者拐弯的路段会增加成本，代价系数在routing_config.pb.txt中定义)，中心线（虚拟的，用于生成参考线），是否可见，车道所属的道路id。
	message Node {
	  optional string lane_id = 1;
	  optional double length = 2;
	  repeated CurveRange left_out = 3;
	  repeated CurveRange right_out = 4;
	  optional double cost = 5;
	  optional apollo.hdmap.Curve central_curve = 6;
	  optional bool is_virtual = 7 [default = true];
	  optional string road_id = 8;
	}

	// Edge	连接车道之间的边，包含了起止车道id，代价和方向等信息
	// EDGE - 则包括起始车道id，到达车道id，切换代价，方向（向前，向左，向右）
	message Edge {
	  enum DirectionType {
	    FORWARD = 0;
	    LEFT = 1;
	    RIGHT = 2;
	 }

	  optional string from_lane_id = 1;
	  optional string to_lane_id = 2;
	  optional double cost = 3;
	  optional DirectionType direction_type = 4;
	}

	// Graph	完整地图的Topo结构，这其中包含了多个Node和Edge
	message Graph {
	  optional string hdmap_version = 1;
	  optional string hdmap_district = 2;
	  repeated Node node = 3;
	  repeated Edge edge = 4;
	}
	                         
  例如，Routing模块以及其他模块都需要用的数据结构就定义在modules/common/proto/目录下。这其中包含的proto文件如下：

.
├── drive_event.proto
├── drive_state.proto
├── error_code.proto
├── geometry.proto
├── header.proto
├── pnc_point.proto
└── vehicle_signal.proto

3.  // Topo地图
	为了计算路由路径，在Routing模块中包含一系列的类用来描述Topo地图的详细结构,简单来说，Topo地图中最重要的就是节点和边，节点对应了道路，边对应了道路的连接关系

	// /modules/routing/graph:

	类名						描述

	TopoNode	//Topo地图中的一个节点。包含了所属Lane和Road等信息。很显然，这是Topo地图中的核心数据结构。
	TopoEdge	//连接TopoNode之间的边，该结构中包含了起止TopoNode等信息。
	NodeSRange	//描述节点的某一段范围。一个TopoNode可以分为若干个NodeSRange。
	NodeWithRange	//描述节点及其范围，该类是NodeSRange的子类。
	TopoRangeManager	//NodeSRange的管理器。可以进行查找，添加，排序和合并操作。
	SubTopoGraph	//Topo子图，由搜索算法所用（目前是A*搜索算法）。
	TopoGraph	//对应了整个Topo地图。其构造函数需要一个Proto结构导出的地图文件，它将从地图文件中读取完整的Topo结构。



	从源码中可以看到，Routing模块需要的地图结构通过TopoGraph来描述，而TopoGraph的初始化需要一个地图文件。但该地图文件与其他模块需要的地图文件并不一样，这里的地图文件是Proto结构导出的数据。
	之所以这样做是因为：Routing模块不仅需要地图的Topo结构，还需要知道每条路线的行驶代价。在Proto结构中包含了这些信息。在下面的内容中，我们将看到这个行驶代价是从哪里来的。

	很显然，两个地点的导航路径结果通常会有多个。而计算导航路径的时候需要有一定的倾向，这个倾向就是行驶的代价越小越好。我们很自然的想到，影响行驶代价最大的因素就是行驶的距离。
	但实际上，影响行驶代价的因素远不止距离这一个因素。距离只是宏观上的考虑，而从微观的角度来看，行驶过程中，需要进行多少次转弯，多少次掉头，多少变道，这些都是影响行驶代价的因素。所以，在计算行驶代价的时候，需要综合考虑这些因素。
	再从另外一个角度来看，（在路线已经确定的情况下）行驶的距离是一个物理世界客观存在的结果，这是我们无法改变的。不过，对于行驶过程中，有多在意转弯，掉头和变道，每个人或者每个场景下的偏好就不一样了。
	而这，就是上文中提到的配置文件“/apollo/modules/routing/conf/routing_config.pb.txt“存在的意义了。这里面配置了上面提到的这些动作的惩罚基数，而这些基数会影响路线时的计算代价。

	通过将这种偏好以配置文件的形式存储在代码之外，可以在不用重新编译代码的情况下，直接调整导航搜索的结果。并且可以方便的为不同的场景进行策略的配置（例如：高速环境和城市道路，这些参数的值很可能就是不一样的）





4.
	// /modules/routing/graph/topo_node.cc

	Topo地图本质上是一系列的Topo节点以及它们的连接关系。因此TopoNode就要能够描述这些信息,有了这些信息之后，在进行路径搜索时，可以方便的查找线路

	  const std::string & LaneId() const;
	  const std::string & RoadId() const;
	  const hdmap::Curve & CentralCurve() const;
	  const common::PointENU & AnchorPoint() const;
	  const std::vector<NodeSRange> & LeftOutRange() const;
	  const std::vector<NodeSRange> & RightOutRange() const;

	  const std::unordered_set<const TopoEdge*> & InFromAllEdge() const;
	  const std::unordered_set<const TopoEdge*> & InFromLeftEdge() const;
	  const std::unordered_set<const TopoEdge*> & InFromRightEdge() const;
	  const std::unordered_set<const TopoEdge*> & InFromLeftOrRightEdge() const;
	  const std::unordered_set<const TopoEdge*> & InFromPreEdge() const;
	  const std::unordered_set<const TopoEdge*> & OutToAllEdge() const;
	  const std::unordered_set<const TopoEdge*> & OutToLeftEdge() const;
	  const std::unordered_set<const TopoEdge*> & OutToRightEdge() const;
	  const std::unordered_set<const TopoEdge*> & OutToLeftOrRightEdge() const;
	  const std::unordered_set<const TopoEdge*> & OutToSucEdge() const;



5.	// TopoCreator

	// /modules/routing/topo_creator/topo_creator.cc: 就是先读取配置文件中的信息到RoutingConfig中，然后通过GraphCreator根据高清地图文件生成Routing模块需要的Topo地图

	与人类开车时所使用的导航系统不一样，自动驾驶需要包含更加细致信息的高精地图，高精地图描述了整个行驶过程中物理世界的详细信息，例如：道路的方向，宽度，曲率，红绿灯的位置等等。而物理世界的这些状态是很容易会发生改变的，例如，添加了一条新的道路，或者是新的红绿灯。
	这就要求高精地图也要频繁的更新。
	那么Routing模块需要的地图文件也需要一起配套的跟着变化，这就很自然的需要有一个模块能够完成从原先的高精地图生成Routing模块的Proto格式地图这一转换工作。而完成这一工作的，就是TopoCreator模块就是先读取配置文件中的信息到RoutingConfig中，然后通过GraphCreator根据高清地图文件生成Routing模块需要的Topo地图

	// apollo/modules/routing/topo_creator/topo_creator.cc
	TopoCreator的源码位于modules/routing/topo_creator/目录下，这是一个可执行程序
	
	int main(int argc, char **argv) {
		  google::InitGoogleLogging(argv[0]);
		  google::ParseCommandLineFlags(&argc, &argv, true);

		  apollo::routing::RoutingConfig routing_conf;

		  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_routing_conf_file,  //先读取配置文件中的信息到RoutingConfig中
													   &routing_conf))
			  << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

		  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

		  const auto base_map = apollo::hdmap::BaseMapFile();
		  const auto routing_map = apollo::hdmap::RoutingMapFile();

		  apollo::routing::GraphCreator creator(base_map, routing_map, routing_conf);  // 然后通过GraphCreator根据高清地图文件生成Routing模块需要的Topo地图。
		  CHECK(creator.Create()) << "Create routing topo failed!";

		  AINFO << "Create routing topo successfully from " << base_map << " to "
				<< routing_map;
		  return 0;
		}

6. // Routing模块初始化

	// apollo/modules/routing/routing.cc:
	Routing模块通过Init方法来初始化。在初始化时，会创建Navigator对象以及加载地图

	apollo::common::Status Routing::Init() {
	  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
	  AINFO << "Use routing topology graph path: " << routing_map_file;

	  navigator_ptr_.reset(new Navigator(routing_map_file)); //创建Navigator对象
	  CHECK( cyber::common::GetProtoFromFile(FLAGS_routing_conf_file, &routing_conf_)) << "Unable to load routing conf file: " + FLAGS_routing_conf_file;
	  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

	  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr(); //加载地图  // Routing内部会通过Navigator来搜索路径。因为需要搜索路径，所以Navigator需要完整的Topo地图。在其构造函数中，会完成Topo地图的加载
	  CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

	  return apollo::common::Status::OK();
	}





7. // Navigator的初始化

	// apollo/modules/routing/core/navigator.cc

	Routing内部会通过Navigator来搜索路径。因为需要搜索路径，所以Navigator需要完整的Topo地图。在其构造函数中，会完成Topo地图的加载


	 Navigator::Navigator(const std::string& topo_file_path) {
	  Graph graph;
	  if (!common::util::GetProtoFromFile(topo_file_path, &graph)) {
	    AERROR << "Failed to read topology graph from " << topo_file_path;
	    return;
	  }

	  graph_.reset(new TopoGraph());
	  if (!graph_->LoadGraph(graph)) {
	    AINFO << "Failed to init navigator graph failed! File path: "  << topo_file_path;
	    return;
	  }

	  black_list_generator_.reset(new BlackListRangeGenerator);   // BlackListRangeGenerator：隐藏地图生成器
	  result_generator_.reset(new ResultGenerator);				  // ResultGenerator：当搜索完成之后，这个对象用来生成搜索结果
	  is_ready_ = true;
	  AINFO << "The navigator is ready.";
	}




8. // 	路由请求:
	bool Routing::Process(const std::shared_ptr<RoutingRequest> &routing_request, RoutingResponse* const routing_response);
	// 一个是描述请求的输入参数routing_request，一个是包含结果的输出参数routing_response。它们都是在proto文件中定义的



9. // BlackMap
	在一些情况下，地图可能会有信息缺失。在这种情况下，Routing模块支持动态的添加一些信息。这个逻辑主要是通过BlackListRangeGenerator和TopoRangeManager两个类完成的。这其中，前者提供了添加数据的接口，而后者则负责存储这些数据

	modules/routing/core/black_list_range_generator.h
	
	class BlackListRangeGenerator {
	 public:
	  BlackListRangeGenerator() = default;
	  ~BlackListRangeGenerator() = default;

	  void GenerateBlackMapFromRequest(const RoutingRequest& request,
									   const TopoGraph* graph,
									   TopoRangeManager* const range_manager) const;    // GenerateBlackMapFromRequest：是从RoutingRequest包含的数据中添加

	  void AddBlackMapFromTerminal(const TopoNode* src_node,
								   const TopoNode* dest_node, double start_s,
								   double end_s,
								   TopoRangeManager* const range_manager) const; // AddBlackMapFromTerminal：是从终端添加数据
	};
	

	这两个接口最后都会通过TopoRangeManager::Add接口来添加数据。该方法代码如下：

	void TopoRangeManager::Add(const TopoNode* node, double start_s, double end_s) {
	    NodeSRange range(start_s, end_s);
	    range_map_[node].push_back(range);
	}

	// TopoRangeManager中的数据最终会被ResultGenerator在组装搜索结果的时候用到



10. // 路由搜索过程
	前面我们提到了Navigator。如果你浏览了这个类的代码就会发现。Navigator本身并没有实现路径搜索的算法。它仅仅是借助其他类来完成路由路径的搜索过程
	相关逻辑在Navigator::SearchRoute方法中

	bool Navigator::SearchRoute(const RoutingRequest& request, RoutingResponse* const response) {

	  if (!ShowRequestInfo(request, graph_.get())) { 
	    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST, "Error encountered when reading request point!", response->mutable_status()); // 1. 对请求参数进行检查
	    return false;
	  }
	  if (!IsReady()) {
	    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!", response->mutable_status());  // 2.判断自身是否处于就绪状态
	    return false;
	  }
	  std::vector<const TopoNode*> way_nodes;  // 3.初始化请求需要的参数
	  std::vector<double> way_s;

	  if (!Init(request, graph_.get(), &way_nodes, &way_s)) {
	    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY,"Failed to initialize navigator!", response->mutable_status());
	    return false;
	  }

	  std::vector<NodeWithRange> result_nodes; //搜索的结果

	  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {  // 4.执行搜索算法
	    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,"Failed to find route with request!", response->mutable_status());
	    return false;
	  }

	  if (result_nodes.empty()) {
	    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!", response->mutable_status());
	    return false;
	  }

	  result_nodes.front().SetStartS(request.waypoint().begin()->s());
	  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());

	  if (!result_generator_->GeneratePassageRegion( graph_->MapVersion(), request, result_nodes, topo_range_manager_, response)) { //5. 搜索结果的组装就是通过ResultGenerator借助搜索的结果std::vector<NodeWithRange>以及TopoRangeManager来进行组装的
	    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to generate passage regions based on result lanes",  response->mutable_status());
	    return false;
	  }

	  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());

	  PrintDebugData(result_nodes);
	  return true;
	}



	这段代码虽长，但其实主体逻辑是很清晰的，主要包含了这么几个步骤：

		1.对请求参数进行检查；
		2.判断自身是否处于就绪状态；
		3.初始化请求需要的参数；
		4.执行搜索算法；
		5.组装搜索结果；搜索结果的组装就是通过ResultGenerator借助搜索的结果std::vector<NodeWithRange>以及TopoRangeManager来进行组装的。

		前面我们提到，搜索的结果RoutingResponse类型也是在proto文件中的定义的，其内容如下：

		message RoutingResponse {
		  optional apollo.common.Header header = 1;
		  repeated RoadSegment road = 2;
		  optional Measurement measurement = 3;
		  optional RoutingRequest routing_request = 4;
		  optional bytes map_version = 5;
		  optional apollo.common.StatusPb status = 6;
		}
		
		
		message Passage {
		  repeated LaneSegment segment = 1;
		  optional bool can_exit = 2;
		  optional ChangeLaneType change_lane_type = 3 [default = FORWARD];
		}

		message RoadSegment {
		  optional string id = 1;
		  repeated Passage passage = 2;
		}

		message RoutingResponse {
		  optional apollo.common.Header header = 1;
		  repeated RoadSegment road = 2;
		  optional Measurement measurement = 3;
		  optional RoutingRequest routing_request = 4;

		  // the map version which is used to build road graph
		  optional bytes map_version = 5;
		  optional apollo.common.StatusPb status = 6;
		}


13. // AStarStrategy

		Navigator::SearchRoute方法的第四步调用了类自身的SearchRouteByStrategy方法。在这个方法中，会借助AStarStrategy来完成路径的搜索。

		AStarStrategy类是抽象类Strategy子类，





14. Cyber RT与模块启动

	查看Routing模块根目录下的BUILD文件。你会发现该模块的编译产物其实是一个动态库（so文件），而非一个可执行文件。
	Apollo 3.5彻底摒弃了ROS，改用自研的Cyber作为底层通讯与调度平台。Apollo Cyber RT 系统是Apollo开源软件平台层的一部分，作为运行时计算框架，处于实时操作系统 （RTOS）和应用模块之间。Apollo Cyber RT作为基础平台，支持流畅高效的运行所有应用模块。

	Routing模块的源码，你会发现一个dag文件: /modules/routing/dag

	Apollo Cyber RT 框架核心理念是基于的组件，组件有预先设定的输入输出。实际上，每个组件就代表一个专用得算法模块。框架可以根据所有预定义的组件生成有向无环图（DAG）
	在运行时刻，框架把融合好的传感器数据和预定义的组件打包在一起形成用户级轻量任务，之后，框架的调度器可以根据资源可用性和任务优先级来派发这些任务。
