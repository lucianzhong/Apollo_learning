Reference:
https://zhuanlan.zhihu.com/p/65533164

//////////////////////////////////////////////////////////////

1. 
	Routing类似于现在开车时用到的导航模块，通常考虑的是起点到终点的最优路径（通常是最短路径），Routing考虑的是起点到终点的最短路径，而Planning则是行驶过程中，当前一小段时间如何行驶，需要考虑当前路况，是否有障碍物。
	Routing模块则不需要考虑这些信息，只需要做一个长期的规划路径即可

	Routing - 主要关注起点到终点的长期路径，根据起点到终点之间的道路，选择一条最优路径。
	Planning - 主要关注几秒钟之内汽车的行驶路径，根据当前行驶过程中的交通规则，车辆行人等信息，规划一条短期路径。

	最短路径算法：    Dijkstra算法
					A*算法
					Bellman-Ford算法
					SPFA算法（Bellman-Ford算法的改进版本）
					Floyd-Warshall算法
					Johnson算法
					Bi-Direction BFS算法





2.	通过上面的介绍可以知道，routing需要的是一个拓扑结构的图，要想做routing，第一步就是要把原始的地图转换成包含拓扑结构的图，apollo中也实现了类似的操作，把base_map转换为routing_map，这里的base_map就是高精度地图，而routing_map则是导航地图，
	routing_map的结构为一个有向图。对应的例子在"modules/map/data/demo"中，这个例子比较简陋，因为routing_map.txt中只包含一个节点(Node)，没有边(Edge)信息。

	apollo建图的实现在"routing/topo_creator"中，首先apollo的拓扑图中的节点和上面介绍的传统的节点不一样，我们前面的例子中，节点就是路的起点和终点，边就是路，而自动驾驶中的道路是车道线级别的，原来的这种定义点和边的方式就不适用了（定义不了车道），
	所以apollo中引用的新的概念，apollo中的点就是一条车道，而边则是车道和车道之间的连接，点对应具体的车道，而边则是一个虚拟的概念，表示车道之间的关系。下面我们可以先看下apollo中道路(road)和车道(lane)的概念

	可以看到一条道路(road)，包含多个车道(lane)，图中一条道路分成了2段，每一段包含3条车道(lane)，车道的信息见图中，主要标识了车道唯一id，左边界，右边界，参考线，长度，前车道，后车道，左边相邻的车道，右边相邻的车道等，
	通过这些结构化的信息，我们就知道车道之间的相互关系，也就知道了我们能否到达下一个车道，从而规划出一条到达目的地的车道线级别的路线，
	Planning模块在根据规划好的线路进行行驶，因为已经到车道线级别了，所以相对规划起来就简单很多。最后我们会建立一张如下的图，其中节点是一个个的lane，而边则代表lane之间的连接。

	// 以lane2举例子
	id                              = 2	
	predecessor_id                  = null 			// 上一车道id，不考虑变道的情况
	successor_id                    = 5   	   		// 下一车道id，不考虑变道的情况
	left_neighbor_forward_lane_id   = 1    			// 左边邻居车道
	right_neighbor_forward_lane_id  = 3    			// 右边邻居车道
	type                            = CITY_DRIVING
	turn                            = NO_TURN  		// 没有拐弯，有些车道本身是曲线，如路口的左拐弯，右拐弯车道
	direction                       = FORWARD 		 // 前向，反向，或者双向
	speed_limit                     = 30      		// 限速30km/h

	// 以lane5举例子
	id                              = 5
	predecessor_id                  = 2 			// 上一车道id，不考虑变道的情况
	successor_id                    = null   		 // 下一车道id，不考虑变道的情况
	left_neighbor_forward_lane_id   = 4   			// 左边邻居车道
	right_neighbor_forward_lane_id  = 6   			 // 右边邻居车道
	type                            = CITY_DRIVING
	turn                            = NO_TURN 		 // 没有拐弯，有些车道本身是曲线，如路口的左拐弯，右拐弯车道
	direction                       = FORWARD  		// 前向，反向，或者双向
	speed_limit                     = 30     		 // 限速30km/h



3. 建图的代码目录为"routing/topo_creator":
		.
	├── BUILD
	├── edge_creator.cc            // 建边 
	├── edge_creator.h
	├── graph_creator.cc           // 建图
	├── graph_creator.h
	├── graph_creator_test.cc
	├── node_creator.cc            // 建节点
	├── node_creator.h
	└── topo_creator.cc           // main函数

	编译生成可执行文件"topo_creator"，地图需要事先通过"topo_creator"把base_map转换为routing_map



4. graph_creator.cc :

	小结一下创建的图的流程，首先是从base_map中读取道路信息，之后遍历道路，先创建节点，然后创建节点的边，之后把图(点和边的信息)保存到routing_map中，所以routing_map中就是graph_ protobuf格式的固化，
	后面routing模块会读取创建好的routing_map通过astar算法来进行路径规划

	bool GraphCreator::Create() {
		   // 这里注意，有2种格式，一种是openstreet格式，通过OpendriveAdapter来读取,另外一种是apollo自己定义的格式
		  if ( common::util::EndWith(base_map_file_path_, ".xml") ) {
		    if ( !hdmap::adapter::OpendriveAdapter::LoadData(base_map_file_path_, &pbmap_) ) {
		      AERROR << "Failed to load base map file from " << base_map_file_path_;
		      return false;
		    }
		  } else {
		    if (!common::util::GetProtoFromFile(base_map_file_path_, &pbmap_)) {
		      AERROR << "Failed to load base map file from " << base_map_file_path_;
		      return false;
		    }
		  }

		  AINFO << "Number of lanes: " << pbmap_.lane_size();

		  // graph_为最后保存的图，消息格式在topo_graph.proto中申明 //topo_graph.pb.h 
		  graph_.set_hdmap_version(pbmap_.header().version());
		  graph_.set_hdmap_district(pbmap_.header().district());

		  node_index_map_.clear();
		  road_id_map_.clear();
		  showed_edge_id_set_.clear();

		  // 从base_map中读取道路和lane对应关系，base_map的消息结构在map.proto和map_road.proto中
		  for (const auto& road : pbmap_.road()) {
		    for (const auto& section : road.section()) {
		      for (const auto& lane_id : section.lane_id()) {
		        road_id_map_[lane_id.id()] = road.id().id();
		      }
		    }
		  }

		  // 初始化禁止的车道线，从配置文件中读取最小掉头半径
		  InitForbiddenLanes();
		  const double min_turn_radius = VehicleConfigHelper::GetConfig().vehicle_param().min_turn_radius();

		  // 遍历base_map中的lane，并且创建节点
		  for (const auto& lane : pbmap_.lane()) {
		    const auto& lane_id = lane.id().id();
		    // 跳过不是城市道路(CITY_DRIVING)的车道
		    if ( forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end() ) {
		      ADEBUG << "Ignored lane id: " << lane_id << " because its type is NOT CITY_DRIVING.";
		      continue;
		    }
		     // 跳过掉头曲率太小的车道
		    if ( lane.turn() == hdmap::Lane::U_TURN && !IsValidUTurn(lane, min_turn_radius) ) {
		      ADEBUG << "The u-turn lane radius is too small for the vehicle to turn";
		      continue;
		    }

		    AINFO << "Current lane id: " << lane_id;

		    // 存储图中节点index和lane_id的关系，因为跳过node可以找到lane，而通过lane_id需要遍历节点才能找到节点index。
		    node_index_map_[lane_id] = graph_.node_size();
		    
		     // 如果从road_id_map_中找到lane_id，则把创建节点的时候指定道路id，如果没有找到那么road_id则为空
		    const auto iter = road_id_map_.find(lane_id);
		    if (iter != road_id_map_.end()) {
		      node_creator::GetPbNode(lane, iter->second, routing_conf_, graph_.add_node());
		    } else {
		      AWARN << "Failed to find road id of lane " << lane_id;
		      node_creator::GetPbNode(lane, "", routing_conf_, graph_.add_node());
		    }
		  }

		  std::string edge_id = "";
		  // 遍历base_map中的lane，并且创建边
		  for (const auto& lane : pbmap_.lane()) {
		    const auto& lane_id = lane.id().id();
		     // 跳过不是城市道路(CITY_DRIVING)的车道
		    if (forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end()) {
		      ADEBUG << "Ignored lane id: " << lane_id << " because its type is NOT CITY_DRIVING.";
		      continue;
		    }

		 // 这里就是通过上面所说的通过lane_id找到node的index，得到节点，
    	 // 如果不保存，则需要遍历所有节点通过lane_id来查找节点，原因为node中有lane_id，
    	 // 而lane结构中没有node_id
		    const auto& from_node = graph_.node(node_index_map_[lane_id]);


		    // 添加一条该节点到下一个节点的边，注意这里没有换道，所以方向为前
		    AddEdge(from_node, lane.successor_id(), Edge::FORWARD);
		    if (lane.length() < FLAGS_min_length_for_lane_change) {
		      continue;
		    }

		    // 车道有左边界，并且允许变道
    		// 添加一条该节点到左边邻居的边
		    if (lane.has_left_boundary() && IsAllowedToCross(lane.left_boundary())) {
		      AddEdge(from_node, lane.left_neighbor_forward_lane_id(), Edge::LEFT);
		    }

		    if (lane.has_right_boundary() && IsAllowedToCross(lane.right_boundary())) {
		      AddEdge(from_node, lane.right_neighbor_forward_lane_id(), Edge::RIGHT);
		    }
		  }

		  if (!EndWith(dump_topo_file_path_, ".bin") &&
		      !EndWith(dump_topo_file_path_, ".txt")) {
		    AERROR << "Failed to dump topo data into file, incorrect file type "
		           << dump_topo_file_path_;
		    return false;
		  }

		   // 保存routing_map文件，有2种格式txt和bin
		  auto type_pos = dump_topo_file_path_.find_last_of(".") + 1;
		  std::string bin_file = dump_topo_file_path_.replace(type_pos, 3, "bin");
		  std::string txt_file = dump_topo_file_path_.replace(type_pos, 3, "txt");
		  if (!common::util::SetProtoToASCIIFile(graph_, txt_file)) {
		    AERROR << "Failed to dump topo data into file " << txt_file;
		    return false;
		  }
		  AINFO << "Txt file is dumped successfully. Path: " << txt_file;
		  if (!common::util::SetProtoToBinaryFile(graph_, bin_file)) {
		    AERROR << "Failed to dump topo data into file " << bin_file;
		    return false;
		  }
		  AINFO << "Bin file is dumped successfully. Path: " << bin_file;
		  return true;
		}


	小结一下创建的图的流程，首先是从base_map中读取道路信息，之后遍历道路，先创建节点，然后创建节点的边，之后把图(点和边的信息)保存到routing_map中，所以routing_map中就是grap_ protobuf格式的固化，
	后面routing模块会读取创建好的routing_map通过astar算法来进行路径规划。



5.创建节点

	apollo/modules/routing/topo_creator/node_creator.h

	void GetPbNode(const hdmap::Lane& lane, const std::string& road_id, const RoutingConfig& routingconfig, Node* const node) {
	  // 1. 初始化节点信息
	  InitNodeInfo(lane, road_id, node);
	  // 2. 初始化节点代价
	  InitNodeCost(lane, routingconfig, node);
}

	// 1. 初始化哪些节点信息呢？
	void InitNodeInfo(const Lane& lane, const std::string& road_id, Node* const node) {
	  double lane_length = GetLaneLength(lane);
	  node->set_lane_id(lane.id().id());
	  node->set_road_id(road_id);
	  // 根据lane的边界，添加能够变道的路段
	  AddOutBoundary(lane.left_boundary(), lane_length, node->mutable_left_out());
	  AddOutBoundary(lane.right_boundary(), lane_length, node->mutable_right_out());
	  node->set_length(lane_length);
	  node->mutable_central_curve()->CopyFrom(lane.central_curve());
	  node->set_is_virtual(true);
	  if (!lane.has_junction_id() || lane.left_neighbor_forward_lane_id_size() > 0 || lane.right_neighbor_forward_lane_id_size() > 0) {
	    node->set_is_virtual(false);
	  }
	}


	// 2. 如何计算节点的代价呢？
	void InitNodeCost(const Lane& lane, const RoutingConfig& routing_config, Node* const node) {
	  double lane_length = GetLaneLength(lane);
	  double speed_limit = (lane.has_speed_limit()) ? lane.speed_limit() : routing_config.base_speed();
	  double ratio = (speed_limit >= routing_config.base_speed()) ? (1 / sqrt(speed_limit / routing_config.base_speed())): 1.0;
	  // 1. 根据道路长度和速度限制来计算代价
	  double cost = lane_length * ratio;
	  if (lane.has_turn()) {
	    if (lane.turn() == Lane::LEFT_TURN) {
	    	// 2. 掉头代价 > 左转代价 > 右转的代价  // left_turn_penalty: 50.0   // right_turn_penalty: 20.0   // uturn_penalty: 100.0
	      cost += routing_config.left_turn_penalty();
	    } else if (lane.turn() == Lane::RIGHT_TURN) {
	      cost += routing_config.right_turn_penalty();
	    } else if (lane.turn() == Lane::U_TURN) {
	      cost += routing_config.uturn_penalty();
	    }
	  }
	  node->set_cost(cost);
	}



6.	创建节点的边

 apollo/modules/routing/topo_creator/edge_creator.cc

 void GetPbEdge( const Node& node_from, const Node& node_to, const Edge::DirectionType& type, const RoutingConfig& routing_config, Edge* edge) {
	  // 设置起始，终止车道和类型
	  edge->set_from_lane_id(node_from.lane_id());
	  edge->set_to_lane_id(node_to.lane_id());
	  edge->set_direction_type(type);

	  // 默认代价为0，即直接向前开的代价
	  edge->set_cost(0.0);
	  if (type == Edge::LEFT || type == Edge::RIGHT) {
	    const auto& target_range = (type == Edge::LEFT) ? node_from.left_out() : node_from.right_out();
	    double changing_area_length = 0.0;
	    for (const auto& range : target_range) {
	      changing_area_length += range.end().s() - range.start().s();
	    }
	    double ratio = 1.0;
	    // 计算代价
	    if (changing_area_length < routing_config.base_changing_length()) {
	      ratio = std::pow( changing_area_length / routing_config.base_changing_length(), -1.5 );
	    }
	    edge->set_cost(routing_config.change_penalty() * ratio);
	  }
	}


我们可以看下edge cost的曲线，因为"changing_area_length / routing_config.base_changing_length() < 1"，这个函数最小值为1，最大值为无穷。
到这里制作routing_map的流程就结束了，建图的主要目的是把base结构的map转换为graph结构的map，从而利用图结构来查找最佳路径，下面会分析如何通过routing_map得到规划好的路线。





7. Routing主流程

	把一些主要的流程摘要如下：
	1.在cyber中注册component，接收request请求，响应请求结果response
	2.读取routing_map并且建图graph
	3.获取request中的routing请求节点
	4.根据black_map生成子图sub_graph
	5.通过astar算法查找最短路径
	6.合并请求结果并且返回

下面在结合具体的流程进行分析，这里主要要弄清楚2点：1.为什么要生成子图？ 2.如何通过astar算法查找最优路径？


8. apollo/modules/routing/routing_component.cc

	apollo的功能被划分为各个模块，启动时候由cyber框架根据模块间的依赖顺序加载(每个模块的dag文件定义了依赖顺序)，每次开始查看一个模块时，都是从component文件开始。
	启动时候由cyber框架根据模块间的依赖顺序加载(每个模块的dag文件定义了依赖顺序)，每次开始查看一个模块时，都是从component文件开始,	routing模块都按照cyber的模块申明和注册，cyber框架负责调用Init进行初始化，并且收到消息时候触发Proc执行
	从上面的分析可以看出，"RoutingComponent"模块实现的主要功能:

	实现"Init"和"Proc"函数
	接收"RoutingRequest"消息，输出"RoutingResponse"响应。

	routing模块都按照cyber的模块申明和注册，cyber框架负责调用Init进行初始化，并且收到消息时候触发Proc执行。

	namespace apollo {
	namespace routing {

	class RoutingComponent final : public ::apollo::cyber::Component<RoutingRequest> {
	 public:
	  RoutingComponent() = default;		// default用来控制默认构造函数的生成。显式地指示编译器生成该函数的默认版本
	  ~RoutingComponent() = default;

	 public:
	  bool Init() override;
	  bool Proc(const std::shared_ptr<RoutingRequest>& request) override;		// 收到routing request的时候触发

	 private:
	  std::shared_ptr<::apollo::cyber::Writer<RoutingResponse>> response_writer_ =  nullptr;		// routing消息发布handle

	  std::shared_ptr<::apollo::cyber::Writer<RoutingResponse>> response_history_writer_ = nullptr;

	  Routing routing_;		// Routing类
	  std::shared_ptr<RoutingResponse> response_ = nullptr;
	  // 定时器
	  std::unique_ptr<::apollo::cyber::Timer> timer_;
	  // 锁
	  std::mutex mutex_;
	};

	// 在cyber框架中注册routing模块
	CYBER_REGISTER_COMPONENT(RoutingComponent)

	}  // namespace routing
	}  // namespace apollo




	bool RoutingComponent::Init() {
		// 设置消息qos，控制流量，创建消息发布response_writer_
	  apollo::cyber::proto::RoleAttributes attr;
	  attr.set_channel_name(FLAGS_routing_response_topic);
	  auto qos = attr.mutable_qos_profile();
	  qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
	  qos->set_reliability(apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
	  qos->set_durability(apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
	  response_writer_ = node_->CreateWriter<RoutingResponse>(attr);

	  apollo::cyber::proto::RoleAttributes attr_history;
	  attr_history.set_channel_name(FLAGS_routing_response_history_topic);
	  auto qos_history = attr_history.mutable_qos_profile();
	  qos_history->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
	  qos_history->set_reliability(apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
	  qos_history->set_durability(apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);

	  // 历史消息发布，和response_writer_类似
	  response_history_writer_ = node_->CreateWriter<RoutingResponse>(attr_history);
	  // 创建定时器
	  std::weak_ptr<RoutingComponent> self =  std::dynamic_pointer_cast<RoutingComponent>(shared_from_this());
	  timer_.reset(new ::apollo::cyber::Timer(
	      FLAGS_routing_response_history_interval_ms,
	      [self, this]() {
	        auto ptr = self.lock();
	        if (ptr) {
	          std::lock_guard<std::mutex> guard(this->mutex_);
	          if (this->response_.get() != nullptr) {
	            auto response = *response_;
	            auto timestamp = apollo::common::time::Clock::NowInSeconds();
	            response.mutable_header()->set_timestamp_sec(timestamp);
	            this->response_history_writer_->Write(response);
	          }
	        }
	      },
	      false));
	  timer_->Start();
	   // 执行Routing类
	  return routing_.Init().ok() && routing_.Start().ok();
	}

// 接下来当routing模块收到routing_request时，会触发"Proc()"，返回routing_response:
	bool RoutingComponent::Proc(const std::shared_ptr<RoutingRequest>& request) {
	  auto response = std::make_shared<RoutingResponse>();
	   // 响应routing_请求
	  if (!routing_.Process(request, response.get())) {
	    return false;
	  }
	  //  // 填充响应头部信息，并且发布
	  common::util::FillHeader(node_->Name(), response.get());
	  response_writer_->Write(response);
	  {
	    std::lock_guard<std::mutex> guard(mutex_);
	    response_ = std::move(response);
	  }
	  return true;
	}



9. Routing类

	//Routing的初始化函数
	apollo::common::Status Routing::Init() {
		// 读取routing_map，也就是点和边
	  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
	  AINFO << "Use routing topology graph path: " << routing_map_file;

	  navigator_ptr_.reset(new Navigator(routing_map_file));

	  CHECK( cyber::common::GetProtoFromFile(FLAGS_routing_conf_file, &routing_conf_))    << "Unable to load routing conf file: " + FLAGS_routing_conf_file;
	  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

	  // 读取地图，用来查找routing request请求的点距离最近的lane，
  // 并且返回对应的lane id，这里很好理解，比如你在小区里面，需要打车，需要找到最近的乘车点，说直白点，就是找到最近的路
	  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();

	  CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

	  return apollo::common::Status::OK();
	}


	// 之后会执行"Process"主流程
	//"Process"主流程
	bool Routing::Process(const std::shared_ptr<RoutingRequest>& routing_request, RoutingResponse* const routing_response) {
	  CHECK_NOTNULL(routing_response);
	  AINFO << "Get new routing request:" << routing_request->DebugString();

	  // 找到routing_request节点最近的路
	  const auto& fixed_request = FillLaneInfoIfMissing(*routing_request);
	   // 是否能够找到规划路径
	  if (!navigator_ptr_->SearchRoute(fixed_request, routing_response)) {
	    AERROR << "Failed to search route with navigator.";

	    monitor_logger_buffer_.WARN("Routing failed! " + routing_response->status().msg());
	    return false;
	  }

	  monitor_logger_buffer_.INFO("Routing success!");
	  return true;
	}
	上述的过程总结一下就是，首先读取routing_map并初始化Navigator类，接着遍历routing_request，因为routing_request请求为一个个的点，所以先查看routing_request的点是否在路上，不在路上则找到最近的路，并且补充信息（不在路上的点则过不去），
	最后调用"navigator_ptr_->SearchRoute"返回routing响应。



10. Navigator类

	Navigator初始化

	bool Navigator::Init( const RoutingRequest& request, const TopoGraph* graph, std::vector<const TopoNode*>* const way_nodes, std::vector<double>* const way_s) {
	  Clear();
	  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {  // 获取routing请求，对应图中的节点
	    AERROR << "Failed to find search terminal point in graph!";
	    return false;
	  }
	  black_list_generator_->GenerateBlackMapFromRequest(request, graph_.get(), &topo_range_manager_);    // 根据请求生成对应的黑名单lane
	  return true;
	}

	 在routing请求中可以指定黑名单路和车道，这样routing请求将不会计算这些车道。应用场景是需要避开拥堵路段，这需要能够根据情况实时请求，在routing_request中可以设置黑名单也刚好可以满足上面的需求，如果直接把黑名单路段固定，则是一个比较蠢的设计。


	剩下的一些过程比较简单，我们直接看主函数"SearchRouteByStrategy":

	bool Navigator::SearchRouteByStrategy( const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes, const std::vector<double>& way_s, std::vector<NodeWithRange>* const result_nodes) const {
	  std::unique_ptr<Strategy> strategy_ptr;
	  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result)); // 通过Astar算法来查找路径

	  result_nodes->clear();
	  std::vector<NodeWithRange> node_vec;
	   // 编译routing_request节点
	  for (size_t i = 1; i < way_nodes.size(); ++i) {
	    const auto* way_start = way_nodes[i - 1];
	    const auto* way_end = way_nodes[i];
	    double way_start_s = way_s[i - 1];
	    double way_end_s = way_s[i];

	    TopoRangeManager full_range_manager = topo_range_manager_;
	     // 添加黑名单，这里主要是把车道根据起点和终点做分割。
	    black_list_generator_->AddBlackMapFromTerminal( way_start, way_end, way_start_s, way_end_s, &full_range_manager);

	     // 因为对车道做了分割，这里会创建子图，比如一个车道分成2个子节点， 2个子节点会创建一张子图。
	    SubTopoGraph sub_graph(full_range_manager.RangeMap());

	     // 获取起点
	    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);
	    if (start == nullptr) {
	      AERROR << "Sub graph node is nullptr, origin node id: " << way_start->LaneId() << ", s:" << way_start_s;
	      return false;
	    }
	     // 获取终点
	    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);
	    if (end == nullptr) {
	      AERROR << "Sub graph node is nullptr, origin node id: " << way_end->LaneId() << ", s:" << way_end_s;
	      return false;
	    }

		// 通过Astar查找最优路径
	    std::vector<NodeWithRange> cur_result_nodes;
	    if (!strategy_ptr->Search(graph, &sub_graph, start, end, &cur_result_nodes)) {
	      AERROR << "Failed to search route with waypoint from " << start->LaneId()  << " to " << end->LaneId();
	      return false;
	    }

	     // 保存结果到node_vec
	    node_vec.insert(node_vec.end(), cur_result_nodes.begin(),  cur_result_nodes.end());
	  }

	   // 合并Route
	  if (!MergeRoute(node_vec, result_nodes)) {
	    AERROR << "Failed to merge route.";
	    return false;
	  }
	  return true;
	}



11. 子节点
	下面我们把子图的概念讲解一下，"AddBlackMapFromTerminal"中会把节点(这里的节点就是lane)切分，切分之后的数据保存在"TopoRangeManager"中，而"SubTopoGraph"会根据"TopoRangeManager"中的数据初始化子图。
	我们先理解下子节点的概念，节点就是一条lane，而子节点是对lane做了切割，把一条lane根据黑名单区域，生成几个子节点。


	节点的切分是根据TopoRangeManager生成好的区间，然后进行切分生成子节点。我们先看下如何生成"TopoRangeManager"，
	在"AddBlackMapFromTerminal"**输入参数为routing_request的开始lane，结束的lane，开始位置，结束位置，输出参数为分段好的区间(range)**，在"range_map_"中保存lane和lane中range的关系，其中key为节点，value为区间(range)

	GenerateBlackMapFromRequest - 通过request请求传入黑名单lane和road，每次直接屏蔽一整条road或者lane。
	AddBlackMapFromTerminal - 虽然range_manager支持传入range，但是这种场景只是针对routing_request传入的点对lane做切割，方便计算，每次切割的区间的起点和终点重合，是一个特殊场景，后续应该有用到比如在一条lane里，有某一段不能行驶的功能。


	apollo/modules/routing/core/black_list_range_generator.cc

void BlackListRangeGenerator::AddBlackMapFromTerminal( const TopoNode* src_node, const TopoNode* dest_node, double start_s, double end_s, TopoRangeManager* const range_manager) const {
	  double start_length = src_node->Length();
	  double end_length = dest_node->Length();
	  if (start_s < 0.0 || start_s > start_length) {
	    AERROR << "Illegal start_s: " << start_s << ", length: " << start_length;
	    return;
	  }
	  if (end_s < 0.0 || end_s > end_length) {
	    AERROR << "Illegal end_s: " << end_s << ", length: " << end_length;
	    return;
	  }

	  double start_cut_s = MoveSBackward(start_s, 0.0); 
	  range_manager->Add(src_node, start_cut_s, start_cut_s);// 注意这里range的起点和终点是同一个点，为routing的起点
	  // 把平行的节点也按照比例做相同的切分
	  AddBlackMapFromOutParallel(src_node, start_cut_s / start_length, range_manager);
	  // 注意这里range的起点和终点是同一个点，为routing的终点
	  double end_cut_s = MoveSForward(end_s, end_length);
	  range_manager->Add(dest_node, end_cut_s, end_cut_s);
	  AddBlackMapFromInParallel(dest_node, end_cut_s / end_length, range_manager);
	   // 排序并且合并
	  range_manager->SortAndMerge();
	}



apollo/modules/routing/graph/sub_topo_graph.cc
	void GetSortedValidRange( const TopoNode* topo_node,
	                         const std::vector<NodeSRange>& origin_range,
	                         std::vector<NodeSRange>* valid_range) {
	  std::vector<NodeSRange> block_range;
	  MergeBlockRange(topo_node, origin_range, &block_range);
	  double start_s = topo_node->StartS();
	  double end_s = topo_node->EndS();
	  std::vector<double> all_value;
	   // 添加node起点，边界和node终点
	  all_value.push_back(start_s);

	  for (const auto& range : block_range) {
	    all_value.push_back(range.StartS());
	    all_value.push_back(range.EndS());
	  }
	  all_value.push_back(end_s);
	    // **核心在这里，每次i+2，即跳过balck_range，生成valid_range*
	  for (size_t i = 0; i < all_value.size(); i += 2) {
	    NodeSRange new_range(all_value[i], all_value[i + 1]);
	    valid_range->push_back(std::move(new_range));
	  }
	}



	SubTopoGraph::SubTopoGraph( const std::unordered_map<const TopoNode*, std::vector<NodeSRange> >& black_map ) {
	  std::vector<NodeSRange> valid_range;
	  for (const auto& map_iter : black_map) {
	    valid_range.clear();
	    GetSortedValidRange(map_iter.first, map_iter.second, &valid_range);
	     // 生成子节点
	    InitSubNodeByValidRange(map_iter.first, valid_range);
	  }

	  for (const auto& map_iter : black_map) {
	  	  // 生成子边
	    InitSubEdge(map_iter.first);
	  }
	  for (const auto& map_iter : black_map) {
	    AddPotentialEdge(map_iter.first);
	  }
	}


由于Graph节点中已经有边的信息，因此原先的Graph中的边的信息实际上已经保存在节点中了，最后Astar实际上只用到了子图的信息，因为节点有自己边的信息。subgraph的核心是通过边找到子边，如果节点不存在子节点，那么返回原先的边，
通过该函数可以同时找到边和子边，这样节点和子节点都可以找到了。我们下面重点分析下"GetSubInEdgesIntoSubGraph"

	void SubTopoGraph::GetSubInEdgesIntoSubGraph( const TopoEdge* edge,std::unordered_set<const TopoEdge*>* const sub_edges) const {
		  const auto* from_node = edge->FromNode();
		  const auto* to_node = edge->ToNode();
		  std::unordered_set<TopoNode*> sub_nodes;
		  // 如果起点是子节点，终点是子节点，或者终点没有子节点，返回边
  // TODO: **这里传入的是edge，根本不可能是子节点？**
		  if (from_node->IsSubNode() || to_node->IsSubNode() ||
		      !GetSubNodes(to_node, &sub_nodes)) {
		    sub_edges->insert(edge);
		    return;
		  }
		  // 如果终点有子节点，则返回所有的子边
		  for (const auto* sub_node : sub_nodes) {
		    for (const auto* in_edge : sub_node->InFromAllEdge()) {
		      if (in_edge->FromNode() == from_node) {
		        sub_edges->insert(in_edge);
		      }
		    }
		  }
		}

关于子图的分析就结束了，子图主要是针对一条lane切分为几个子节点的情况，根据切分好的子节点从新生成一张图，比原先根据routing_map建立的图有更细的粒度




Reference:

https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/planning_arch.md


12. apollo/modules/routing/proto/routing.proto

	Planning和Routing模块的核心数据结构。其中包括路由查询RoutingRequest与路由响应RoutingResponse。


	/*
	RoutingRequest里面的waypoint(LaneWaypoint类型)是路径查询的核心，例如我要查询公交站A到学校B的一条路径，那么waypoint就是两个；如果我要查询公交站A到学校B的一条路径，并且我还要经过早餐店C，那么最终的waypoint就是三个。
	LaneSegment和Prediction中的LaneSegment一样，定义了一条车道的[start_s, end_s]路段区间，使用repeated形式可以完整的离散化定义一条车道。
	LaneWaypoint可以定义车道上的任意一点，包括所在车道id，所在车道的累计距离s，以及世界坐标系下的坐标pose
	*/

	message LaneWaypoint {
	  optional string id = 1;
	  optional double s = 2;
	  optional apollo.common.PointENU pose = 3;
	}

	message LaneSegment {
	    optional string id = 1;
	    optional double start_s = 2;
	    optional double end_s = 3;
	  }

	message RoutingRequest {
	  optional apollo.common.Header header = 1;
	  // at least two points. The first is start point, the end is final point.
	  // The routing must go through each point in waypoint.
	  repeated LaneWaypoint waypoint = 2;
	  repeated LaneSegment blacklisted_lane = 3;
	  repeated string blacklisted_road = 4;
	  optional bool broadcast = 5 [default = true];
	}

	message Measurement {
	  optional double distance = 1;
	}

	enum ChangeLaneType {
	    FORWARD = 0;
	    LEFT = 1;
	    RIGHT = 2;
	};


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


以上是路径查询的返回/响应结果RoutingResponse，其中routing_request是对应发出的查询，measurement是行驶距离，最核心的内容就是road(repeated RoadSegment)，这是一条从起点公交站A到重点学校B，并且经过中间早餐店C的完整路径，由一段段离散化的RoadSegment组成。

RoadSegment类型包含了repeated Passage，这意味着，一个RoadSegment中包含了多个通道，每个通道可以理解为一条车道，一个道路段RoadSegment可以有多条并行向前行驶的车道。而Passage中每条车道可以有多个LaneSegment组成，意味着进一步划分成小的区间段，便于精细化调度。



13. 调试工具
	在routing/tools目录实现了如下3个功能：
	routing_cast.cc // 定时发送routing response响应
	routing_dump.cc // 保存routing请求
	routing_tester.cc // 定时发送routing request请求