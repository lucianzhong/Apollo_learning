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













