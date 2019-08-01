1. apollo/modules/routing/strategy/a_star_strategy.h


		namespace apollo {
		namespace routing {

			class AStarStrategy : public Strategy {
			 public:
			  explicit AStarStrategy(bool enable_change);
			  ~AStarStrategy() = default;

			  // A*搜索函数
			  virtual bool Search( const TopoGraph* graph, const SubTopoGraph* sub_graph, const TopoNode* src_node, const TopoNode* dest_node, std::vector<NodeWithRange>* const result_nodes );

			 private:
			  void   Clear();   // 清空上次结果
			  double HeuristicCost(const TopoNode* src_node, const TopoNode* dest_node);    // 计算src_node到dest_node的启发式代价
			  double GetResidualS(const TopoNode* node);									// 计算结点node到终点的剩余距离s
			  double GetResidualS(const TopoEdge* edge, const TopoNode* to_node);  			//计算边edge到结点to_node的剩余距离s

			 private:
			  bool change_lane_enabled_;										//允许变换车道
			  std::unordered_set<const TopoNode*> open_set_;  					//OPEN集
			  std::unordered_set<const TopoNode*> closed_set_; 					//CLOSED集
			  std::unordered_map<const TopoNode*, const TopoNode*> came_from_;  // 子父结点键值对  // key: 子结点 // value: 父结点
			  std::unordered_map<const TopoNode*, double> g_score_;  			// 移动代价键值对  // key: Node  // value: 从源结点移动到Node的代价
			  std::unordered_map<const TopoNode*, double> enter_s_;  			// 结点的进入距离键值对  // key: Node  // value: Node的进入距离
			}; 

		}  // namespace routing
		}  // namespace apollo



	//待考察结点
   struct SearchNode {
	  const TopoNode* topo_node = nullptr;
	  double f = std::numeric_limits<double>::max(); //启发式代价值f

	  SearchNode() = default;

	  explicit SearchNode(const TopoNode* node) : topo_node(node), f(std::numeric_limits<double>::max()) {}

	  SearchNode(const SearchNode& search_node) = default;

	  bool operator < (const SearchNode& node) const {
	    // in order to let the top of priority queue is the smallest one!   // // 重载<运算符，改变小于逻辑，以更其作为优先级队列std::priority_queue的内部元素时， // std::priority_queue 的栈顶元素永远是值为最小的一个。
	    return f > node.f;
	  }

	  bool operator==(const SearchNode& node) const {
	    return topo_node == node.topo_node;
	  }
	};


2. 标准的启发式代价函数是曼哈顿距离（Manhattan distance）。所谓曼哈顿距离就是两点在南北方向上的距离加上在东西方向上的距离。曼哈顿距离又称为出租车距离，曼哈顿距离不是距离不变量，当坐标轴变动时，点间的距离就会不同。
   Apollo项目采用的就是曼哈顿距离函数：

    H(n) = abs ( n.x – goal.x ) + abs ( n.y – goal.y ) 

    double AStarStrategy::HeuristicCost(const TopoNode* src_node, const TopoNode* dest_node) {
	  const auto& src_point = src_node->AnchorPoint();
	  const auto& dest_point = dest_node->AnchorPoint();
	  double distance = fabs( src_point.x() - dest_point.x() ) + fabs( src_point.y() - dest_point.y() );
	  return distance;
	}


	modules/routing/strategy/a_star_strategy.cc:
	个节点的cost都可以由相邻节点的cost加上连接至自身的边的cost之和来计算

	double GetCostToNeighbor(const TopoEdge* edge) {
	  return (edge->Cost() + edge->ToNode()->Cost());
	}





// A_star:

	* 初始化open_set和close_set；
* 将起点加入open_set中，并设置优先级为0（优先级最高）；
* 如果open_set不为空，则从open_set中选取优先级最高的节点n：
    * 如果节点n为终点，则：
        * 从终点开始逐步追踪parent节点，一直达到起点；
        * 返回找到的结果路径，算法结束；
    * 如果节点n不是终点，则：
        * 将节点n从open_set中删除，并加入close_set中；
        * 遍历节点n所有的邻近节点：
            * 如果邻近节点m在close_set中，则：
                * 跳过，选取下一个邻近节点
            * 如果邻近节点m也不在open_set中，则：
                * 设置节点m的parent为节点n
                * 计算节点m的优先级
                * 将节点m加入open_set中




3.  modules/routing/strategy/a_star_strategy.cc

		bool AStarStrategy::Search(const TopoGraph* graph, const SubTopoGraph* sub_graph, const TopoNode* src_node, const TopoNode* dest_node, std::vector<NodeWithRange>* const result_nodes) {
		  
		  Clear();	 // 清空上次结果

		  AINFO << "Start A* search algorithm.";

		  std::priority_queue<SearchNode> open_set_detail;    // std::priority_queue是一种容器适配器，它提供常数时间的最大元素查找功能，亦即其栈顶元素top永远输出队列中的最大元素。但SearchNode内部重载了<运算符，
		  													  //对小于操作作了相反的定义，因此std::priority_queue<SearchNode>的栈顶元素,永远输出队列中的最小元素。

 		  SearchNode src_search_node(src_node);		// 将源结点设置为检查结点

		  src_search_node.f = HeuristicCost(src_node, dest_node);   // 计算检查结点的启发式代价值f

		  open_set_detail.push(src_search_node);  // 将检查结点压入OPEN集优先级队列
		  open_set_.insert(src_node);			  // 将源结点加入OPEN集   //std::unordered_set<const TopoNode*> open_set_;

		  g_score_[src_node] = 0.0;			// 源结点到自身的移动代价值g为0   //std::unordered_map<const TopoNode*, double> g_score_;

		  enter_s_[src_node] = src_node->StartS();		// 设置源结点的进入s值   //std::unordered_map<const TopoNode*, double> enter_s_;  //  double start_s_;

		  SearchNode current_node;

		  std::unordered_set<const TopoEdge*> next_edge_set;
		  std::unordered_set<const TopoEdge*> sub_edge_set;


		  while (!open_set_detail.empty()) {	  				// 只要OPEN集优先级队列不为空，就不断循环检查

		    current_node = open_set_detail.top();				// 取出栈顶元素（f值最小）

		    const auto* from_node = current_node.topo_node;		// 设置起始结点

		    //若起始结点已抵达最终的目标结点，则反向回溯输出完整的路由，返回。
		    if (current_node.topo_node == dest_node) {
		      if ( !Reconstruct(came_from_, from_node, result_nodes) ) {
		        AERROR << "Failed to reconstruct route.";
		        return false;
		      }
		      return true;
		    }

		    open_set_.erase(from_node);	 // 从OPEN集中删除起始结点
		    open_set_detail.pop();		 // 从OPEN集队列中删除起始结点


		    if (closed_set_.count(from_node) != 0) {     // 若起始结点from_node在CLOSED集中的计数不为0，表明之前已被检查过，直接跳过
		        continue;
		    }


		    closed_set_.emplace(from_node);	// 将起始结点加入关闭集

		    // if residual_s is less than FLAGS_min_length_for_lane_change, only move forward
		    // 获取起始结点from_node的所有相邻边,若起始结点from_node到终点的剩余距离s比FLAGS_min_length_for_lane_change要短，则不考虑变换车道，即只考虑前方结点而不考虑左右结点。
		    // 反之，若s比FLAGS_min_length_for_lane_change要长，则考虑前方及左右结点。
		    // std::unordered_set<const TopoEdge*> out_to_all_edge_set_;

		    const auto& neighbor_edges = ( GetResidualS(from_node) > FLAGS_min_length_for_lane_change && change_lane_enabled_ )  ?  from_node->OutToAllEdge() : from_node->OutToSucEdge();

		    double tentative_g_score = 0.0;	  // 当前测试的移动代价值

		    next_edge_set.clear();	

		    // 从相邻边neighbor_edges中获取其内部包含的边，将所有相邻边全部加入集合：next_edge_set
		    for (const auto* edge : neighbor_edges) {
		      sub_edge_set.clear();
		      sub_graph->GetSubInEdgesIntoSubGraph(edge, &sub_edge_set);		//modules/routing/graph/sub_topo_graph.cc
		      next_edge_set.insert(sub_edge_set.begin(), sub_edge_set.end());
		    }

		     // 所有相邻边的目标结点就是我们需要逐一测试的相邻结点，对相结点点逐一测试，寻找总代价f = g + h最小的结点，该结点就是起始结点所需的相邻目标结点
		for (const auto* edge : next_edge_set) {

		      const auto* to_node = edge->ToNode();

			// 相邻结点to_node在CLOSED集中，表明之前已被检查过，直接忽略。似乎修改为if (closed_set_.count(to_node) > 0)更好？
		      if (closed_set_.count(to_node) == 1) {
		        continue;
		      }

		       // 若当前边到相邻结点to_node的距离小于FLAGS_min_length_for_lane_change，表明不能通过变换车道的方式从当前边切换到相邻结点to_node，直接忽略。
		      if ( GetResidualS(edge, to_node) < FLAGS_min_length_for_lane_change ) {
		        continue;
		      }

		       // 更新当前结点的移动代价值g
		      tentative_g_score = g_score_[current_node.topo_node] + GetCostToNeighbor(edge);

		       // 如果边类型不是前向，而是左向或右向，表示变换车道的情形，则更改移动代价值g的计算方式
		      if ( edge->Type() != TopoEdgeType::TET_FORWARD ) {
		        tentative_g_score -= ( edge->FromNode()->Cost() + edge->ToNode()->Cost() ) / 2;
		      }

		      // 总代价 f = g + h
		      double f = tentative_g_score + HeuristicCost(to_node, dest_node);


		      // 若相邻结点to_node在OPEN集且当前总代价f大于源结点到相邻结点to_node的移动代价g，表明现有情形下从当前结点到相邻结点to_node的路径不是最优，直接忽略。
     		  // 因为相邻结点to_node在OPEN集中，后续还会对该结点进行考察。
      		  // open_set_.count(to_node) != 0修改为open_set_.count(to_node) > 0似乎更好
		      if (open_set_.count(to_node) != 0 && f >= g_score_[to_node]) {
		        continue;
		      }

			 // if to_node is reached by forward, reset enter_s to start_s
      		 // 如果是以向前（而非向左或向右）的方式抵达相邻结点to_node，则将to_node的进入距离更新为to_node的起始距离
		      if (edge->Type() == TopoEdgeType::TET_FORWARD) {
		        enter_s_[to_node] = to_node->StartS();
		      } else {


		      	// else, add enter_s with FLAGS_min_length_for_lane_change
        		// 若是以向左或向右方式抵达相邻结点to_node，则将to_node的进入距离更新为:当前结点from_node的进入距离加上最小换道长度，并乘以相邻结点to_node长度与当前结点from_node长度的比值（这么做的目的是为了归一化，以便最终的代价量纲一致）
		        double to_node_enter_s = ( enter_s_[from_node] + FLAGS_min_length_for_lane_change ) / from_node->Length() * to_node->Length();
		        // enter s could be larger than end_s but should be less than length
		        to_node_enter_s = std::min(to_node_enter_s, to_node->Length());
		        // if enter_s is larger than end_s and to_node is dest_node
		        if (to_node_enter_s > to_node->EndS() && to_node == dest_node) {
		          continue;
		        }
		        enter_s_[to_node] = to_node_enter_s;
		      }

		      // 更新从源点移动到结点to_node的移动代价（因为找到了一条代价更小的路径，必须更新它）
		      g_score_[to_node] = f;

		      // 将相邻结点to_node设置为下一个待考察结点
		      SearchNode next_node(to_node);
		      next_node.f = f;

		      // 当下一个待考察结点next_node加入到OPEN优先级队列
		      open_set_detail.push(next_node);

		      // 将to_node的父结点设置为from_node
		      came_from_[to_node] = from_node;

		      // 若相邻结点不在OPEN集中，则将其加入OPEN集，以便后续考察
		      if ( open_set_.count(to_node) == 0 ) {
		        open_set_.insert(to_node);
		      }
		    }
		  }

		  // 整个循环结束后仍未正确返回，表明搜索失败
		  AERROR << "Failed to find goal lane with id: " << dest_node->LaneId();
		  return false;
		}




4. 数据结构

	4.1 TopoGraph

     





     4.2 SubTopoGraph







     4.3 TopoNode

     	apollo/modules/routing/graph/topo_node.h






     4.4 TopoEdge

     	apollo/modules/routing/graph/topo_node.h








     4.5 NodeWithRange












5. Routing proto:
   /apolloauto/modules/routing/proto:

	 5.1 poi.proto

	 Point of interest的缩写，一个POI中可以包含多个Landmark

	 message Landmark {
	  optional string name = 1;
	  repeated LaneWaypoint waypoint = 2;
	  optional string parking_space_id = 3;
	}

	message POI {
	  repeated Landmark landmark = 1;    //地图上的一个点，包含了名称和位置信息
	}




	5.2 routing_config.proto

	描述了Routing模块的配置信息，上面提到的routing_config.pb.txt文件就是这个格式的

	message RoutingConfig {
	  optional double base_speed = 1;  // base speed for node creator [m/s]
	  optional double left_turn_penalty = 2;  // left turn penalty for node creater [m]
	  optional double right_turn_penalty = 3;  // right turn penalty for node creater [m]
	  optional double uturn_penalty = 4;  // left turn penalty for node creater [m]
	  optional double change_penalty = 5;  // change penalty for edge creater [m]
	  optional double base_changing_length = 6;  // base change length penalty for edge creater [m]
	}


	5.3  routing.proto

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



	5.4 topo_graph.proto

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
	                         


6.  Topo地图
	为了计算路由路径，在Routing模块中包含一系列的类用来描述Topo地图的详细结构,简单来说，Topo地图中最重要的就是节点和边，节点对应了道路，边对应了道路的连接关系

	/modules/routing/graph:

	类名	描述
	TopoNode	Topo地图中的一个节点。包含了所属Lane和Road等信息。
	很显然，这是Topo地图中的核心数据结构。
	TopoEdge	连接TopoNode之间的边，该结构中包含了起止TopoNode等信息。
	NodeSRange	描述节点的某一段范围。一个TopoNode可以分为若干个NodeSRange。
	NodeWithRange	描述节点及其范围，该类是NodeSRange的子类。
	TopoRangeManager	NodeSRange的管理器。可以进行查找，添加，排序和合并操作。
	SubTopoGraph	Topo子图，由搜索算法所用（目前是A*搜索算法）。
	TopoGraph	对应了整个Topo地图。其构造函数需要一个Proto结构导出的地图文件，
	它将从地图文件中读取完整的Topo结构。



	从源码中可以看到，Routing模块需要的地图结构通过TopoGraph来描述，而TopoGraph的初始化需要一个地图文件。但该地图文件与其他模块需要的地图文件并不一样，这里的地图文件是Proto结构导出的数据。之所以这样做是因为：Routing模块不仅需要地图的Topo结构，还需要知道每条路线的行驶代价。在Proto结构中包含了这些信息。在下面的内容中，我们将看到这个行驶代价是从哪里来的。

	很显然，两个地点的导航路径结果通常会有多个。而计算导航路径的时候需要有一定的倾向，这个倾向就是行驶的代价越小越好。我们很自然的想到，影响行驶代价最大的因素就是行驶的距离。

	但实际上，影响行驶代价的因素远不止距离这一个因素。距离只是宏观上的考虑，而从微观的角度来看，行驶过程中，需要进行多少次转弯，多少次掉头，多少变道，这些都是影响行驶代价的因素。所以，在计算行驶代价的时候，需要综合考虑这些因素。

	再从另外一个角度来看，（在路线已经确定的情况下）行驶的距离是一个物理世界客观存在的结果，这是我们无法改变的。不过，对于行驶过程中，有多在意转弯，掉头和变道，每个人或者每个场景下的偏好就不一样了。而这，就是上文中提到的配置文件“/apollo/modules/routing/conf/routing_config.pb.txt“存在的意义了。这里面配置了上面提到的这些动作的惩罚基数，而这些基数会影响路线时的计算代价。

	通过将这种偏好以配置文件的形式存储在代码之外，可以在不用重新编译代码的情况下，直接调整导航搜索的结果。并且可以方便的为不同的场景进行策略的配置（例如：高速环境和城市道路，这些参数的值很可能就是不一样的）





7. 
	/modules/routing/graph/topo_node.h

	Topo地图本质上是一系列的Topo节点以及它们的连接关系。因此TopoNode就要能够描述这些信息,有了这些信息之后，在进行路径搜索时，可以方便的查找线路

	  const std::string& LaneId() const;
	  const std::string& RoadId() const;
	  const hdmap::Curve& CentralCurve() const;
	  const common::PointENU& AnchorPoint() const;
	  const std::vector<NodeSRange>& LeftOutRange() const;
	  const std::vector<NodeSRange>& RightOutRange() const;

	  const std::unordered_set<const TopoEdge*>& InFromAllEdge() const;
	  const std::unordered_set<const TopoEdge*>& InFromLeftEdge() const;
	  const std::unordered_set<const TopoEdge*>& InFromRightEdge() const;
	  const std::unordered_set<const TopoEdge*>& InFromLeftOrRightEdge() const;
	  const std::unordered_set<const TopoEdge*>& InFromPreEdge() const;
	  const std::unordered_set<const TopoEdge*>& OutToAllEdge() const;
	  const std::unordered_set<const TopoEdge*>& OutToLeftEdge() const;
	  const std::unordered_set<const TopoEdge*>& OutToRightEdge() const;
	  const std::unordered_set<const TopoEdge*>& OutToLeftOrRightEdge() const;
	  const std::unordered_set<const TopoEdge*>& OutToSucEdge() const;



8. 	TopoCreator
	/modules/routing/topo_creator/topo_creator.cc: 就是先读取配置文件中的信息到RoutingConfig中，然后通过GraphCreator根据高清地图文件生成Routing模块需要的Topo地图

	与人类开车时所使用的导航系统不一样，自动驾驶需要包含更加细致信息的高精地图，高精地图描述了整个行驶过程中物理世界的详细信息，例如：道路的方向，宽度，曲率，红绿灯的位置等等。而物理世界的这些状态是很容易会发生改变的，例如，添加了一条新的道路，或者是新的红绿灯。这就要求高精地图也要频繁的更新。

	那么Routing模块需要的地图文件也需要一起配套的跟着变化，这就很自然的需要有一个模块能够完成从原先的高精地图生成Routing模块的Proto格式地图这一转换工作。而完成这一工作的，就是TopoCreator模块


	就是先读取配置文件中的信息到RoutingConfig中，然后通过GraphCreator根据高清地图文件生成Routing模块需要的Topo地图


9. Routing模块初始化

	apollo/modules/routing/routing.cc:
	Routing模块通过Init方法来初始化。在初始化时，会创建Navigator对象以及加载地图

	apollo::common::Status Routing::Init() {
	  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
	  AINFO << "Use routing topology graph path: " << routing_map_file;
	  navigator_ptr_.reset(new Navigator(routing_map_file));
	  CHECK(
	      cyber::common::GetProtoFromFile(FLAGS_routing_conf_file, &routing_conf_))
	      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

	  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

	  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
	  CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

	  return apollo::common::Status::OK();
	}


	路由请求:
	bool Routing::Process(const std::shared_ptr<RoutingRequest> &routing_request, RoutingResponse* const routing_response);
	// 一个是描述请求的输入参数routing_request，一个是包含结果的输出参数routing_response。它们都是在proto文件中定义的



10. Navigator的初始化

	apollo/modules/routing/core/navigator.cc

	Routing内部会通过Navigator来搜索路径。因为需要搜索路径，所以Navigator需要完整的Topo地图。在其构造函数中，会完成Topo地图的加载


	 Navigator::Navigator(const std::string& topo_file_path) {
	  Graph graph;
	  if (!common::util::GetProtoFromFile(topo_file_path, &graph)) {
	    AERROR << "Failed to read topology graph from " << topo_file_path;
	    return;
	  }

	  graph_.reset(new TopoGraph());
	  if (!graph_->LoadGraph(graph)) {
	    AINFO << "Failed to init navigator graph failed! File path: "
	          << topo_file_path;
	    return;
	  }
	  black_list_generator_.reset(new BlackListRangeGenerator);   //BlackListRangeGenerator：隐藏地图生成器
	  result_generator_.reset(new ResultGenerator);				// ResultGenerator：当搜索完成之后，这个对象用来生成搜索结果
	  is_ready_ = true;
	  AINFO << "The navigator is ready.";
	}


11. BlackMap
	在一些情况下，地图可能会有信息缺失。在这种情况下，Routing模块支持动态的添加一些信息。这个逻辑主要是通过BlackListRangeGenerator和TopoRangeManager两个类完成的。这其中，前者提供了添加数据的接口，而后者则负责存储这些数据

	modules/routing/core/black_list_range_generator.h

	namespace apollo {
	namespace routing {

	class BlackListRangeGenerator {
	 public:
	  BlackListRangeGenerator() = default;
	  ~BlackListRangeGenerator() = default;

	  void GenerateBlackMapFromRequest(const RoutingRequest& request, const TopoGraph* graph, TopoRangeManager* const range_manager) const; //GenerateBlackMapFromRequest：是从RoutingRequest包含的数据中添加

	  void AddBlackMapFromTerminal(const TopoNode* src_node, const TopoNode* dest_node, double start_s, double end_s, TopoRangeManager* const range_manager) const; //AddBlackMapFromTerminal：是从终端添加数据
	};

	}  // namespace routing
	}  // namespace apollo


	这两个接口最后都会通过TopoRangeManager::Add接口来添加数据。该方法代码如下：

	void TopoRangeManager::Add(const TopoNode* node, double start_s, double end_s) {
	    NodeSRange range(start_s, end_s);
	    range_map_[node].push_back(range);
	}

	TopoRangeManager中的数据最终会被ResultGenerator在组装搜索结果的时候用到



12. 路由搜索过程

	
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

	  std::vector<NodeWithRange> result_nodes;

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


13. AStarStrategy

		Navigator::SearchRoute方法的第四步调用了类自身的SearchRouteByStrategy方法。在这个方法中，会借助AStarStrategy来完成路径的搜索。

		AStarStrategy类是抽象类Strategy子类，





14. Cyber RT与模块启动

	查看Routing模块根目录下的BUILD文件。你会发现该模块的编译产物其实是一个动态库（so文件），而非一个可执行文件。

	Apollo 3.5彻底摒弃了ROS，改用自研的Cyber作为底层通讯与调度平台。Apollo Cyber RT 系统是Apollo开源软件平台层的一部分，作为运行时计算框架，处于实时操作系统 （RTOS）和应用模块之间。Apollo Cyber RT作为基础平台，支持流畅高效的运行所有应用模块。

	Routing模块的源码，你会发现一个dag文件: /modules/routing/dag