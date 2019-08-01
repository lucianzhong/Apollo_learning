// Reference:
https://zhuanlan.zhihu.com/p/65533164
https://paul.pub/apollo-routing/


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

	Routing类似于现在开车时用到的导航模块，通常考虑的是起点到终点的最优路径（通常是最短路径），Routing考虑的是起点到终点的最短路径，而Planning则是行驶过程中，当前一小段时间如何行驶，需要考虑当前路况，是否有障碍物。
	Routing模块则不需要考虑这些信息，只需要做一个长期的规划路径即可

	Routing - 主要关注起点到终点的长期路径，根据起点到终点之间的道路，选择一条最优路径。
	Planning - 主要关注几秒钟之内汽车的行驶路径，根据当前行驶过程中的交通规则，车辆行人等信息，规划一条短期路径。

	通过上面的介绍可以知道，routing需要的是一个拓扑结构的图，要想做routing，第一步就是要把原始的地图转换成包含拓扑结构的图，apollo中也实现了类似的操作，把base_map转换为routing_map，这里的base_map就是高精度地图，而routing_map则是导航地图，
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



15. 建图的代码目录为"routing/topo_creator":
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



16. graph_creator.cc :

	小结一下创建的图的流程，首先是从base_map中读取道路信息，之后遍历道路，先创建节点，然后创建节点的边，之后把图(点和边的信息)保存到routing_map中，所以routing_map中就是graph_ protobuf格式的固化，
	后面routing模块会读取创建好的routing_map通过astar算法来进行路径规划

	bool GraphCreator::Create() {
		   // 这里注意，有2种格式，一种是openstreet格式，通过OpendriveAdapter来读取,另外一种是apollo自己定义的格式
		  if (common::util::EndWith(base_map_file_path_, ".xml")) {
		    if (!hdmap::adapter::OpendriveAdapter::LoadData(base_map_file_path_, &pbmap_)) {
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
		    if (forbidden_lane_id_set_.find(lane_id) != forbidden_lane_id_set_.end()) {
		      ADEBUG << "Ignored lane id: " << lane_id << " because its type is NOT CITY_DRIVING.";
		      continue;
		    }
		     // 跳过掉头曲率太小的车道
		    if (lane.turn() == hdmap::Lane::U_TURN && !IsValidUTurn(lane, min_turn_radius)) {
		      ADEBUG << "The u-turn lane radius is too small for the vehicle to turn";
		      continue;
		    }

		    AINFO << "Current lane id: " << lane_id;

		    // 存储图中节点index和lane_id的关系，因为跳过node可以找到lane，而通过lane_id需要遍历节点才能找到节点index。
		    node_index_map_[lane_id] = graph_.node_size();
		    
		     // 如果从road_id_map_中找到lane_id，则把创建节点的时候指定道路id，如果没有找到那么road_id则为空
		    const auto iter = road_id_map_.find(lane_id);
		    if (iter != road_id_map_.end()) {
		      node_creator::GetPbNode(lane, iter->second, routing_conf_,
		                              graph_.add_node());
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



17.创建节点

	apollo/modules/routing/topo_creator/node_creator.h

	void GetPbNode(const hdmap::Lane& lane, const std::string& road_id, const RoutingConfig& routingconfig, Node* const node) {
	  // 1. 初始化节点信息
	  InitNodeInfo(lane, road_id, node);
	  // 2. 初始化节点代价
	  InitNodeCost(lane, routingconfig, node);
}




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
	  if (!lane.has_junction_id() ||
	      lane.left_neighbor_forward_lane_id_size() > 0 ||
	      lane.right_neighbor_forward_lane_id_size() > 0) {
	    node->set_is_virtual(false);
	  }
	}



		void InitNodeCost(const Lane& lane, const RoutingConfig& routing_config, Node* const node) {
	  double lane_length = GetLaneLength(lane);
	  double speed_limit = (lane.has_speed_limit()) ? lane.speed_limit()
	                                                : routing_config.base_speed();
	  double ratio = (speed_limit >= routing_config.base_speed())
	                     ? (1 / sqrt(speed_limit / routing_config.base_speed()))
	                     : 1.0;
	  // 1. 根据道路长度和速度限制来计算代价
	  double cost = lane_length * ratio;
	  if (lane.has_turn()) {
	    if (lane.turn() == Lane::LEFT_TURN) {

	    	// 2. 掉头代价 > 左转代价 > 右转的代价
		    // left_turn_penalty: 50.0
		    // right_turn_penalty: 20.0
		    // uturn_penalty: 100.0
	      cost += routing_config.left_turn_penalty();
	    } else if (lane.turn() == Lane::RIGHT_TURN) {
	      cost += routing_config.right_turn_penalty();
	    } else if (lane.turn() == Lane::U_TURN) {
	      cost += routing_config.uturn_penalty();
	    }
	  }
	  node->set_cost(cost);
	}



18.
	创建节点的边
 apollo/modules/routing/topo_creator/edge_creator.cc


 void GetPbEdge(const Node& node_from, const Node& node_to,
               const Edge::DirectionType& type,
               const RoutingConfig& routing_config, Edge* edge) {
	  // 设置起始，终止车道和类型
	  edge->set_from_lane_id(node_from.lane_id());
	  edge->set_to_lane_id(node_to.lane_id());
	  edge->set_direction_type(type);

	  // 默认代价为0，即直接向前开的代价
	  edge->set_cost(0.0);
	  if (type == Edge::LEFT || type == Edge::RIGHT) {
	    const auto& target_range =
	        (type == Edge::LEFT) ? node_from.left_out() : node_from.right_out();
	    double changing_area_length = 0.0;
	    for (const auto& range : target_range) {
	      changing_area_length += range.end().s() - range.start().s();
	    }
	    double ratio = 1.0;
	    // 计算代价
	    if (changing_area_length < routing_config.base_changing_length()) {
	      ratio = std::pow(
	          changing_area_length / routing_config.base_changing_length(), -1.5);
	    }
	    edge->set_cost(routing_config.change_penalty() * ratio);
	  }
	}


我们可以看下edge cost的曲线，因为"changing_area_length / routing_config.base_changing_length() < 1"，这个函数最小值为1，最大值为无穷。


到这里制作routing_map的流程就结束了，建图的主要目的是把base结构的map转换为graph结构的map，从而利用图结构来查找最佳路径，下面会分析如何通过routing_map得到规划好的路线。





19. Routing主流程

	把一些主要的流程摘要如下：
	1.在cyber中注册component，接收request请求，响应请求结果response
	2.读取routing_map并且建图graph
	3.获取request中的routing请求节点
	4.根据black_map生成子图sub_graph
	5.通过astar算法查找最短路径
	6.合并请求结果并且返回

下面在结合具体的流程进行分析，这里主要要弄清楚2点：1.为什么要生成子图？ 2.如何通过astar算法查找最优路径？


20. apollo/modules/routing/routing_component.cc

	apollo的功能被划分为各个模块，启动时候由cyber框架根据模块间的依赖顺序加载(每个模块的dag文件定义了依赖顺序)，每次开始查看一个模块时，都是从component文件开始。
	routing模块都按照cyber的模块申明和注册，cyber框架负责调用Init进行初始化，并且收到消息时候触发Proc执行
	从上面的分析可以看出，"RoutingComponent"模块实现的主要功能:

	实现"Init"和"Proc"函数
	接收"RoutingRequest"消息，输出"RoutingResponse"响应。




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
	  qos->set_reliability(
	      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
	  qos->set_durability(
	      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
	  response_writer_ = node_->CreateWriter<RoutingResponse>(attr);

	  apollo::cyber::proto::RoleAttributes attr_history;
	  attr_history.set_channel_name(FLAGS_routing_response_history_topic);
	  auto qos_history = attr_history.mutable_qos_profile();
	  qos_history->set_history(
	      apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
	  qos_history->set_reliability(
	      apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
	  qos_history->set_durability(
	      apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);

	  // 历史消息发布，和response_writer_类似
	  response_history_writer_ = node_->CreateWriter<RoutingResponse>(attr_history);
	  // 创建定时器
	  std::weak_ptr<RoutingComponent> self =
	      std::dynamic_pointer_cast<RoutingComponent>(shared_from_this());
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



21. Routing类

	//Routing的初始化函数
	apollo::common::Status Routing::Init() {
		// 读取routing_map，也就是点和边
	  const auto routing_map_file = apollo::hdmap::RoutingMapFile();
	  AINFO << "Use routing topology graph path: " << routing_map_file;

	  navigator_ptr_.reset(new Navigator(routing_map_file));

	  CHECK( cyber::common::GetProtoFromFile(FLAGS_routing_conf_file, &routing_conf_))    << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

	  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

	  // 读取地图，用来查找routing request请求的点距离最近的lane，
  // 并且返回对应的lane id，这里很好理解，比如你在小区里面，需要打车，
  // 需要找到最近的乘车点，说直白点，就是找到最近的路
	  hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();

	  CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

	  return apollo::common::Status::OK();
	}



	//"Process"主流程
	bool Routing::Process(const std::shared_ptr<RoutingRequest>& routing_request, RoutingResponse* const routing_response) {
	  CHECK_NOTNULL(routing_response);
	  AINFO << "Get new routing request:" << routing_request->DebugString();

	  // 找到routing_request节点最近的路
	  const auto& fixed_request = FillLaneInfoIfMissing(*routing_request);
	   // 是否能够找到规划路径
	  if (!navigator_ptr_->SearchRoute(fixed_request, routing_response)) {
	    AERROR << "Failed to search route with navigator.";

	    monitor_logger_buffer_.WARN("Routing failed! " +
	                                routing_response->status().msg());
	    return false;
	  }

	  monitor_logger_buffer_.INFO("Routing success!");
	  return true;
	}




	上述的过程总结一下就是，首先读取routing_map并初始化Navigator类，接着遍历routing_request，因为routing_request请求为一个个的点，所以先查看routing_request的点是否在路上，不在路上则找到最近的路，并且补充信息（不在路上的点则过不去），
	最后调用"navigator_ptr_->SearchRoute"返回routing响应。