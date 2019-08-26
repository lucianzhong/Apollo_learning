// Reference:
https://blog.csdn.net/davidhopper/article/details/87438774

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
	    // in order to let the top of priority queue is the smallest one!   // 重载<运算符，改变小于逻辑，以更其作为优先级队列std::priority_queue的内部元素时， // std::priority_queue 的栈顶元素永远是值为最小的一个。
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







4. // Navigator

	在Routing模块中，当所有条件就绪时，就会使用高精地图搜索指定路径，其中在Routing代码中搜索路径的代码为
	modules/routing/routing.cc

	navigator_ptr_->SearchRoute(fixed_request, routing_response)

	该代码的主要目标是在收到request请求后调用navigator指针中的SearchRoute方法，如果成功找到路径，就把结果赋值给routing_response；如果失败就返回错误。所以SearchRoute是navigator中关键的方法，本文也主要会针对该方法做一个介绍


	modules/routing/core/navigator.cc
	// ShowRequestInfo:  展示请求的详细信息，主要展示waypoint以及blacklist
	bool ShowRequestInfo(const RoutingRequest& request, const TopoGraph* graph)


	modules/routing/core/navigator.cc
	// GetWayNodes：将request中的node以及node的s加入
	bool GetWayNodes(const RoutingRequest& request, const TopoGraph* graph, std::vector<const TopoNode*>* const way_nodes, std::vector<double>* const way_s) {

	modules/routing/core/navigator.cc
	// SetErrorCode：报错信息
	void SetErrorCode(const common::ErrorCode& error_code_id, const std::string& error_string, common::StatusPb* const error_code) 

	modules/routing/core/navigator.cc
	// PritDebugData：debug信息
	void PrintDebugData(const std::vector<NodeWithRange>& nodes) 


	modules/routing/core/navigator.cc
	// Navigator：构造函数，查看是否已经准备好导航
	Navigator::Navigator(const std::string& topo_file_path) 


	modules/routing/core/navigator.cc
	// IsReady：就绪标记函数
	bool Navigator::IsReady() const { return is_ready_; }


	Clear：清除拓扑图管理数据


	modules/routing/core/navigator.cc
	// Init：初始化函数
	ool Navigator::Init(const RoutingRequest& request, const TopoGraph* graph, std::vector<const TopoNode*>* const way_nodes, std::vector<double>* const way_s) 


	modules/routing/core/navigator.cc
	// MergeRoute：路径结合
	bool Navigator::MergeRoute( const std::vector<NodeWithRange>& node_vec, std::vector<NodeWithRange>* const result_node_vec) const 


	modules/routing/core/navigator.cc
	// SearchRouteByStrategy：利用Strategy中的算法来做路径规划，这里用的是Astar
	bool Navigator::SearchRouteByStrategy( const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes, const std::vector<double>& way_s, std::vector<NodeWithRange>* const result_nodes) const


	modules/routing/core/navigator.cc
	// SearchRoute：搜索路径，如果有路径就返回True，并赋值，如果没有就返回False
	bool Navigator::SearchRoute(const RoutingRequest& request, RoutingResponse* const response) 



	// 主要函数详解:要函数包含有SearchRoute、Init、SearchRouteByStrategy、MergeRoute

	// Init
	bool Navigator::Init(const RoutingRequest& request, const TopoGraph* graph, std::vector<const TopoNode*>* const way_nodes, std::vector<double>* const way_s) {   // 初始化函数，传入请求，图，路径节点集合，路径s
	  Clear();// 清除缓存变量，主要是拓扑图管理变量
	  if (!GetWayNodes(request, graph_.get(), way_nodes, way_s)) {  //获取request的相关参数
	    AERROR << "Failed to find search terminal point in graph!";
	    return false;
	  }
	  black_list_generator_->GenerateBlackMapFromRequest(request, graph_.get(), &topo_range_manager_);   // 加入黑名单节点到拓扑图管理变量中
	  return true;
	}
	//代码主要功能为将请求加入路径节点，节点的s，这两个参数对于搜索路径是有帮助的。第一个参数是node，也就是节点，第二个参数是开始位置。然后将路径中的不可达黑名单加入管理变量中。



	//SearchRoute
	bool Navigator::SearchRoute(const RoutingRequest& request, RoutingResponse* const response) {   // 搜索算法，输入request，response作为输出
	  if (!ShowRequestInfo(request, graph_.get())) {
	    SetErrorCode(ErrorCode::ROUTING_ERROR_REQUEST, "Error encountered when reading request point!", response->mutable_status());
	    return false;  // 如果request详情没能展示，说明有误，返回错误
	  }
	  if (!IsReady()) {  // 如果构造函数初始化没有成功，就没能ready,返回错误
	    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Navigator is not ready!", response->mutable_status());
	    return false;
	  }
	  std::vector<const TopoNode*> way_nodes; // 定义路径节点vector
	  std::vector<double> way_s;  // 定义路径的S值vector
	  if (!Init(request, graph_.get(), &way_nodes, &way_s)) { // 初始化数据，这里加入了请求的节点和S值
	    SetErrorCode(ErrorCode::ROUTING_ERROR_NOT_READY, "Failed to initialize navigator!", response->mutable_status());
	    return false;
	  }
	  std::vector<NodeWithRange> result_nodes; // 这就是需要的结果
	  if (!SearchRouteByStrategy(graph_.get(), way_nodes, way_s, &result_nodes)) {  // 这里调用了使用策略算法来搜索路径
	    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE,  "Failed to find route with request!", response->mutable_status());
	    return false;
	  }
	  if (result_nodes.empty()) {  // 如果没有找到合适的结果
	    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to result nodes!", response->mutable_status()); 
	    return false;
	  }
	  result_nodes.front().SetStartS(request.waypoint().begin()->s());  // 结果的前项S为请求的开始点
	  result_nodes.back().SetEndS(request.waypoint().rbegin()->s());	// 结果的后项S的结束点为请求的结束点

	  if (!result_generator_->GeneratePassageRegion( graph_->MapVersion(), request, result_nodes, topo_range_manager_, response)) {  // 根据搜索得到的结果集设置路径
	    SetErrorCode(ErrorCode::ROUTING_ERROR_RESPONSE, "Failed to generate passage regions based on result lanes", response->mutable_status());
	    return false;
	  }
	  SetErrorCode(ErrorCode::OK, "Success!", response->mutable_status());	// 设置正确log

	  PrintDebugData(result_nodes);  // 打印出搜索的结果节点
	  return true;
	}
	// SearchRoute模块主要调用ShowRequestInfo，IsReady，Init分别初始化和检验，如果通过了就调用SearchRouteByStrategy函数来利用策略搜索路径，如果最后没有找到合适的结果或者最后结果无法更新路径，则返回错误。
	// 所以其中的SearchRouteByStrategy就是比较关键的策略函数。



	// SearchRouteByStrategy

	bool Navigator::SearchRouteByStrategy(const TopoGraph* graph, const std::vector<const TopoNode*>& way_nodes,const std::vector<double>& way_s, std::vector<NodeWithRange>* const result_nodes) const {  
										//输入图，节点集，s，目标结果vector
	  std::unique_ptr<Strategy> strategy_ptr;	// 策略指针
	  strategy_ptr.reset(new AStarStrategy(FLAGS_enable_change_lane_in_result));  // 这里装的是AStar算法，所以后面使用的Search也是A*
	  result_nodes->clear();	// 清空结果
	  std::vector<NodeWithRange> node_vec;	// 存储路径的一个容器
	  for (size_t i = 1; i < way_nodes.size(); ++i) {	// 对于节点集的所有路径
	    const auto* way_start = way_nodes[i - 1];  // 路径开始节点
	    const auto* way_end = way_nodes[i];	// 路径结束节点
	    double way_start_s = way_s[i - 1];	// 路径开始的S
	    double way_end_s = way_s[i];		// 路径结束的S

	    TopoRangeManager full_range_manager = topo_range_manager_;	// 对于该全连接管理器
	    black_list_generator_->AddBlackMapFromTerminal(way_start, way_end, way_start_s, way_end_s, &full_range_manager); // 黑名单加入全连接管理器

	    SubTopoGraph sub_graph(full_range_manager.RangeMap());	// 全连接管理器赋值给sub_graph
	    const auto* start = sub_graph.GetSubNodeWithS(way_start, way_start_s);	// 获取sub_graph的起始节点
	    if (start == nullptr) {	// 如果起始节点为空，说明没有找到
	      AERROR << "Sub graph node is nullptr, origin node id: " << way_start->LaneId() << ", s:" << way_start_s;
	      return false;
	    }
	    const auto* end = sub_graph.GetSubNodeWithS(way_end, way_end_s);	// 获取sub_graph的终止节点
	    if (end == nullptr) {	// 如果终止节点为空，说明没有找到
	      AERROR << "Sub graph node is nullptr, origin node id: " << way_end->LaneId() << ", s:" << way_end_s;
	      return false;
	    }

	    std::vector<NodeWithRange> cur_result_nodes;	// 结果集
	    if (!strategy_ptr->Search(graph, &sub_graph, start, end, &cur_result_nodes)) {	// 使用A*算法来搜索路径
	      AERROR << "Failed to search route with waypoint from " << start->LaneId()  << " to " << end->LaneId();
	      return false;
	    }

	    node_vec.insert(node_vec.end(), cur_result_nodes.begin(), cur_result_nodes.end());	// 节点集放入上一个节点的结束，当前路径的开始和结束
	  }

	  if (!MergeRoute(node_vec, result_nodes)) {		 // 这段的路径结合起来
	    AERROR << "Failed to merge route.";
	    return false;
	  }
	  return true;
	}


	//该函数主要目标是完成输入图以及节点集，得到搜索路径。对于节点集的每一对节点请求都需要做路径的规划，最后把路径之间merge起来。其中使用到了strategy中的算法A*来计算前后的路径。注：前后的节点对都来源于request请求。所以在求得了多个单条路径之后需要merge。



	// MergeRoute

	bool Navigator::MergeRoute(const std::vector<NodeWithRange>& node_vec, std::vector<NodeWithRange>* const result_node_vec) const {	// 输入为路径集合，目标结果集合
	  for (const auto& node : node_vec) {	// 对于所有的路径集合
	    if (result_node_vec->empty() || result_node_vec->back().GetTopoNode() != node.GetTopoNode()) {	// 如果结果集合为空（第一个循环）或者结果的最后一个节点不等于此次节点
	      result_node_vec->push_back(node);	// 把节点merge进去
	    } else {	// 说明节点重复了
	      if (result_node_vec->back().EndS() < node.StartS()) {	// 已有结果node节点的结束时的S与此时的S之间还有距离，说明已经有间断
	        AERROR << "Result route is not coninuous";
	        return false;
	      } else {	// 说明虽然节点重复了，但是两个节点是重叠的，所以合并成一个节点。这样让结果节点的back等于此时节点的结束
	        result_node_vec->back().SetEndS(node.EndS());
	      }
	    }
	  }
	  return true;
	}


	// MergeRoute的主要工作就是把得到的路径，一段一段的连接在一起。连接的时候注意是否具有重复和间隔，最后得到的就是首尾可以连接的路径。

	// Navigator总结
	Navigator相对条理比较清晰，首先是一些辅助的判断函数作为request的判断，如果一切检验合格后就会根据request的节点对来调用决策函数，最后把每一次计算得到的节点对路径merge起来得到最终的路径计算，返回回去。所以最终能够得到一条结合request的路径。
