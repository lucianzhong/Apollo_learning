
// r3.0.0

1. 规划与控制地图: Pnc Map
	pnc map其实和高精地图hd map没有关系，后者是专门为规划与控制模块设计的库函数，在hd map层次之上，负责一些地图相关信息的处理。例如查询车辆可能的形式路由段(list)，然后对每个路由段合成一个路径Path，这是pnc map最重要的功能。
	pnc map目前被封装在指引线提供器ReferenceLineProvider中，但是由于其功能比较集中，我们单独将他拿出来讲解。规划控制地图pnc map主要的功能有三个：

	更新路由信息。这部分接受Routing模块的路径查询响应，将其响应信息处理存储到地图类中。

	短期路径段查询。根据Routing规划路径以及当前车辆的位置，计算当前车辆可行驶的车道区域。

	路径段生成最终路径。针对2中每个可行驶的车道路由段，生成一条路径Path，可以后续生成参考线Reference Line。


2. 更新路由信息 (PncMap::UpdateRoutingResponse)

	modules/map/pnc_map/pnc_map.cc

	  bool PncMap::UpdateRoutingResponse(const routing::RoutingResponse &routing) {	//响应结果剥离 //从上述代码可以响应结果剥离工作比较简单，就是对这条完整路径进行RoadSegment，Passage，LaneSegment的存储
		  range_lane_ids_.clear();
		  route_indices_.clear();
		  all_lane_ids_.clear();

		  for (int road_index = 0; road_index < routing.road_size(); ++road_index) {
		    const auto &road_segment = routing.road(road_index);
		    for (int passage_index = 0; passage_index < road_segment.passage_size(); ++passage_index) {
		      const auto &passage = road_segment.passage(passage_index);
		      for (int lane_index = 0; lane_index < passage.segment_size(); ++lane_index) {
		        all_lane_ids_.insert(passage.segment(lane_index).id());
		        route_indices_.emplace_back();
		        route_indices_.back().segment = ToLaneSegment(passage.segment(lane_index));
		        if (route_indices_.back().segment.lane == nullptr) {
		          AERROR << "Fail to get lane segment from passage.";
		          return false;
		        }
		        route_indices_.back().index = {road_index, passage_index, lane_index};
		      }
		    }
		  }




		  all_lane_ids_: [  'lane 1', 'lane 1', 'lane 1',       // RoadSegment 0, Passage 0, LaneSegment 0-2
			                'lane 2', 'lane 2', 'lane 2',       // RoadSegment 0, Passage 1, LaneSegment 0-2

			                'lane 1', 'lane 1', 'lane 1',       // RoadSegment 1, Passage 0, LaneSegment 0-2
			                'lane 2', 'lane 2', 'lane 2',       // RoadSegment 1, Passage 1, LaneSegment 0-2

			                'lane 1', 'lane 1', 'lane 1',       // RoadSegment 2, Passage 0, LaneSegment 0-2
			                'lane 2', 'lane 2', 'lane 2',]      // RoadSegment 2, Passage 1, LaneSegment 0-2

		 //  得到的结果
		route_indices_: [LaneSegment{id: 'lane 1', s: [100,110], index: [0,0,0]},     // RoadSegment 0, Passage 0, LaneSegment 0
		                 LaneSegment{id: 'lane 1', s: [110,120], index: [0,0,1]},     // RoadSegment 0, Passage 0, LaneSegment 1
		                 LaneSegment{id: 'lane 1', s: [120,130], index: [0,0,2]},     // RoadSegment 0, Passage 0, LaneSegment 2
		                 LaneSegment{id: 'lane 2', s: [240,250], index: [0,1,0]},     // RoadSegment 0, Passage 1, LaneSegment 0
		                 LaneSegment{id: 'lane 2', s: [250,260], index: [0,1,1]},     // RoadSegment 0, Passage 1, LaneSegment 1
		                 LaneSegment{id: 'lane 2', s: [260,270], index: [0,1,2]},     // RoadSegment 0, Passage 1, LaneSegment 2

		                 LaneSegment{id: 'lane 1', s: [130,140], index: [1,0,0]},     // RoadSegment 1, Passage 0, LaneSegment 0
		                 LaneSegment{id: 'lane 1', s: [140,150], index: [1,0,1]},     // RoadSegment 1, Passage 0, LaneSegment 1
		                 LaneSegment{id: 'lane 1', s: [150,160], index: [1,0,2]},     // RoadSegment 1, Passage 0, LaneSegment 2
		                 LaneSegment{id: 'lane 2', s: [270,280], index: [1,1,0]},     // RoadSegment 1, Passage 1, LaneSegment 0
		                 LaneSegment{id: 'lane 2', s: [280,290], index: [1,1,1]},     // RoadSegment 1, Passage 1, LaneSegment 1
		                 LaneSegment{id: 'lane 2', s: [290,300], index: [1,1,2]},     // RoadSegment 1, Passage 1, LaneSegment 2

		                 LaneSegment{id: 'lane 1', s: [160,170], index: [2,0,0]},     // RoadSegment 2, Passage 0, LaneSegment 0
		                 LaneSegment{id: 'lane 1', s: [170,180], index: [2,0,1]},     // RoadSegment 2, Passage 0, LaneSegment 1
		                 LaneSegment{id: 'lane 1', s: [180,190], index: [2,0,2]},     // RoadSegment 2, Passage 0, LaneSegment 2
		                 LaneSegment{id: 'lane 2', s: [300,310], index: [2,1,0]},     // RoadSegment 2, Passage 1, LaneSegment 0
		                 LaneSegment{id: 'lane 2', s: [310,320], index: [2,1,1]},     // RoadSegment 2, Passage 1, LaneSegment 1
		                 LaneSegment{id: 'lane 2', s: [320,330], index: [2,1,2]}]     // RoadSegment 2, Passage 1, LaneSegment 2

3. 查询点处理

	该次查询的查询点waypoint一共两个，起点(红色星星)和终点(蓝色星星)。这一步需要对这两个waypoint进行处理，计算这两个waypoint分别在上述的那些LaneSegment中，准确的说是上述all_lane_ids_中每个LaneSegment包含有哪些waypoint
	modules/map/pnc_map/route_segments.cc

	bool RouteSegments::WithinLaneSegment(const routing::LaneSegment &lane_segment, const routing::LaneWaypoint &waypoint) {
	  return lane_segment.id() == waypoint.id() && lane_segment.start_s() - kSegmentationEpsilon <= waypoint.s() && lane_segment.end_s() + kSegmentationEpsilon >= waypoint.s();
	}

	waypoint在lane_segment中需要满足条件：
		waypoint和lane_segment的所在的车道lane的id必须一致
		waypoint的累计距离s必须在lane_segment的start_s和end_s之间。




	  for (std::size_t j = 0; j < route_indices_.size(); ++j) {
	    while (i < request_waypoints.size() &&   RouteSegments::WithinLaneSegment(route_indices_[j].segment, request_waypoints.Get(i))) {
	      routing_waypoint_index_.emplace_back( LaneWaypoint(route_indices_[j].segment.lane, request_waypoints.Get(i).s()),  j);
	      ++i;
	    }
	  }


	//得到的结果
	  routing_waypoint_index_: [
		                          // waypoint 1(start point), s=105, j=0(route_indices_[0]: RoadSegment 0,Passage 0,LaneSegment 0)
		                          LaneWaypoint{id: 'lane 1', s: 105, j: 0},    
		                          // waypoint 2(end point), s=185, j=15(route_indices_[15]: RoadSegment 2,Passage 0,LaneSegment 2)
		                          LaneWaypoint{id: 'lane 1', s: 185, j: 15}
		                         ]    



4. 短期路径段查询 (PncMap::GetRouteSegments)
	Planning模块的短期路径查询是根据当前主车的位置，去查询无人驾驶汽车可以行使的车道段(Passage && LaneSegment),在这个查询阶段，
	必须要保证已经存在RoutingResponse，所以在PncMap::GetRouteSegments函数运行时，必须保证在之前已经更新过路由信息PncMap::UpdateRoutingResponse


	/modules/map/pnc_map/pnc_map.cc

	bool PncMap::GetRouteSegments(const VehicleState &vehicle_state, const double backward_length, const double forward_length, std::list<RouteSegments> *const route_segments) { }


	我们可以看到GetRouteSegments接受的参数最重要的是:车辆的状态(包含车辆的位置，速度，偏航角，加速度等信息)，
	backward_length和forward_length是路径短生成过程中前向与后向修正距离，目前暂时不用考虑。
	而返回的是短期内(注意是短期内，长期调度不确定因素太多，无法实现)车辆的运动轨迹route_segments，这是std::list类型的，与上上节的LaneGraph::LineSequence类似，每个元素都是代表当前车辆的一种运动方案。


	bool PncMap::GetRouteSegments(const VehicleState &vehicle_state, const double backward_length,  const double forward_length,  std::list<RouteSegments> *const route_segments) {
	  
	   // Step 1. update vehicle state
	   // 这里的无人车状态不是无人车自身坐标，速度等信息，而是在上述更新路由信息过程中得到的route_indices_中，无人车在哪个LaneSegment中，距离无人车最近的下一个查询点waypoint的信息
	   // ，这个函数完成的工作就是计算上图中的红色部分：包括当前车辆在对应车道上的投影adc_waypoint_，车辆投影点所在LaneSegment在route_indices_中的索引adc_route_index_，下一个最近查询点在routing_waypoint_index_中的索引next_routing_waypoint_index_
		  if (!UpdateVehicleState(vehicle_state)) {
		    AERROR << "Failed to update vehicle state in pnc_map";
		    return false;
		  }

		  // vehicle has to be this close to lane center before considering change
		  // lane
		  if (!adc_waypoint_.lane || adc_route_index_ < 0 ||
		      adc_route_index_ >= static_cast<int>(route_indices_.size())) {
		    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first";
		    return false;
		  }
		  const auto &route_index = route_indices_[adc_route_index_].index;
		  const int road_index = route_index[0];
		  const int passage_index = route_index[1];
		  const auto &road = routing_.road(road_index);
		  // raw filter to find all neighboring passages
		  auto drive_passages = GetNeighborPassages(road, passage_index);
		  for (const int index : drive_passages) {
		    const auto &passage = road.passage(index);
		    RouteSegments segments;
		    if (!PassageToSegments(passage, &segments)) {
		      ADEBUG << "Failed to convert passage to lane segments.";
		      continue;
		    }
		    PointENU nearest_point =
		        MakePointENU(adc_state_.x(), adc_state_.y(), adc_state_.z());
		    if (index == passage_index) {
		      nearest_point = adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s);
		    }
		    common::SLPoint sl;
		    LaneWaypoint segment_waypoint;
		    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
		      ADEBUG << "Failed to get projection from point: "
		             << nearest_point.ShortDebugString();
		      continue;
		    }
		    if (index != passage_index) {
		      if (!segments.CanDriveFrom(adc_waypoint_)) {
		        ADEBUG << "You cannot drive from current waypoint to passage: "
		               << index;
		        continue;
		      }
		    }
		    route_segments->emplace_back();
		    const auto last_waypoint = segments.LastWaypoint();
		    if (!ExtendSegments(segments, sl.s() - backward_length,
		                        sl.s() + forward_length, &route_segments->back())) {
		      AERROR << "Failed to extend segments with s=" << sl.s()
		             << ", backward: " << backward_length
		             << ", forward: " << forward_length;
		      return false;
		    }
		    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
		      route_segments->back().SetRouteEndWaypoint(last_waypoint);
		    }
		    route_segments->back().SetCanExit(passage.can_exit());
		    route_segments->back().SetNextAction(passage.change_lane_type());
		    std::string route_segment_id =
		        std::to_string(road_index) + "_" + std::to_string(index);
		    route_segments->back().SetId(route_segment_id);
		    route_segments->back().SetStopForDestination(stop_for_destination_);
		    if (index == passage_index) {
		      route_segments->back().SetIsOnSegment(true);
		      route_segments->back().SetPreviousAction(routing::FORWARD);
		    } else if (sl.l() > 0) {
		      route_segments->back().SetPreviousAction(routing::RIGHT);
		    } else {
		      route_segments->back().SetPreviousAction(routing::LEFT);
		    }
		  }
		  return !route_segments->empty();
		}




		UpdateVehicleState(vehicle_state){
			//  // Step 1. 计算当前车辆在对应车道上的投影adc_waypoint_
			GetNearestPointFromRouting(vehicle_state, &adc_waypoint_)



		}


















5. 计算临近通道

	在上一步更新车辆状态中，根据路由查询响应以及车辆状态可以得到当前车辆在规划路径中的位置adc_waypoint_，以及下一个必经查询点的索引next_routing_waypoint_index_。接下来这一步就是查询当前位置下，附近的通道。这部分功能由函数GetNeighborPassages实现

	modules/map/pnc_map/pnc_map.cc

	bool PncMap::GetRouteSegments(const VehicleState &vehicle_state, const double backward_length, const double forward_length,  std::list<RouteSegments> *const route_segments) {
		// Step 1. update vehicle state


		 // Step 2. compute neighbor passages
		  // raw filter to find all neighboring passages
		  auto drive_passages = GetNeighborPassages(road, passage_index);
	}

	// 获取邻接可驶入通道
	std::vector<int> PncMap::GetNeighborPassages(const routing::RoadSegment &road, int start_passage) const {
		  CHECK_GE(start_passage, 0);
		  CHECK_LE(start_passage, road.passage_size());
		  std::vector<int> result;
		  const auto &source_passage = road.passage(start_passage);
		  result.emplace_back(start_passage);

		  // 如果当前通道(Passage)是直行道(change_lane_type==routing::FORWARD)，无法变道，那么直接返回车辆所在的车道                                                                                                                                                                                                                                                                                                                                                                                                                                      
		  if (source_passage.change_lane_type() == routing::FORWARD) {
		    return result;
		  }

		  // 如果当前通道已经准备退出(can_exit=-True)，如上图中各个Passage中的LaneSegment3，车辆已经准备进入下一个Passage，不需要变道，直接返回车辆所在的车道
		  if (source_passage.can_exit()) {  // no need to change lane
		    return result;
		  }

		  // 如果下一个毕竟查询点(routing_waypoint_index_[next_routing_waypoint_index_].waypoint)在当前通道中，不需要变道，直接返回车辆所在的车道
		  RouteSegments source_segments;
		  if (!PassageToSegments(source_passage, &source_segments)) {
		    AERROR << "failed to convert passage to segments";
		    return result;
		  }
		  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&  source_segments.IsWaypointOnSegment(routing_waypoint_index_[next_routing_waypoint_index_].waypoint) ) {
		    ADEBUG << "need to pass next waypoint[" << next_routing_waypoint_index_ << "] before change lane";
		    return result;
		  }

		  // 如果车辆再左转车道或者右转车道，从高精地图hd map中查询当前车道对应左侧或者右侧的所有车道线，然后去和当前RoadSegment.passage()去做对比，找到两者共同包含的车道，就是最终的邻接车道
		  std::unordered_set<std::string> neighbor_lanes;
		  if (source_passage.change_lane_type() == routing::LEFT) {		// 当前passage是左转通道
		    for (const auto &segment : source_segments) {				// 查询当前Passage中每个LaneSegment所在车道的邻接左车道
		      for (const auto &left_id : segment.lane->lane().left_neighbor_forward_lane_id()) {
		        neighbor_lanes.insert(left_id.id());
		      }
		    }
		  } else if (source_passage.change_lane_type() == routing::RIGHT) {		//当前passage是右转通道
		    for (const auto &segment : source_segments) {
		      for (const auto &right_id :  segment.lane->lane().right_neighbor_forward_lane_id()) {	 // 查询当前Passage中每个LaneSegment所在车道的邻接右车道
		        neighbor_lanes.insert(right_id.id());
		      }
		    }
		  }

		  for (int i = 0; i < road.passage_size(); ++i) {
		    if (i == start_passage) {
		      continue;
		    }
		    const auto &target_passage = road.passage(i);
		    for (const auto &segment : target_passage.segment()) {
		      if (neighbor_lanes.count(segment.id())) {		// 查询当前RoadSegment中所有Passage::LaneSegment的所属车道，有交集就添加到结果中
		        result.emplace_back(i);
		        break;
		      }
		    }
		  }
		  return result;
		}


6. 创建车辆当前可行驶区域

	我们找到了当前车辆在规划轨迹中的邻接车道。这一步就对每个邻接车道做一个是否可驶入的检查，并做道路段截取。也就是制定出无人车在当前情况下可能行驶的区域，每个可行驶通道将划分出一个道路区间，上面已经提到过，
	道路区间的长度由前向查询距离forward_length和后向查询距离backward_length决定，短期内规划的可行驶道路段长度为forward_length+backward_length,最终的路由段RouteSegments生成只需要对每个邻接可驶入的通道进行Segments生成即可

	/modules/map/pnc_map/pnc_map.cc



	bool PncMap::GetRouteSegments(const VehicleState &vehicle_state, const double backward_length, const double forward_length, std::list<RouteSegments> *const route_segments) {
		  //  Step 1. update vehicle state
		  if (!UpdateVehicleState(vehicle_state)) {
		    AERROR << "Failed to update vehicle state in pnc_map";
		    return false;
		  }
		  // vehicle has to be this close to lane center before considering change
		  // lane
		  if (!adc_waypoint_.lane || adc_route_index_ < 0 || adc_route_index_ >= static_cast<int>(route_indices_.size())) {
		    AERROR << "Invalid vehicle state in pnc_map, update vehicle state first";
		    return false;
		  }
		  const auto &route_index = route_indices_[adc_route_index_].index;
		  const int road_index = route_index[0];
		  const int passage_index = route_index[1];
		  const auto &road = routing_.road(road_index);
		  // Step 2. compute neighbor passages
		  // raw filter to find all neighboring passages
		  auto drive_passages = GetNeighborPassages(road, passage_index);

		   // Step 3. compute route segment  // Passage生成对应的RouteSegments分为4个步骤
		  for (const int index : drive_passages) {
		    const auto &passage = road.passage(index);
		    RouteSegments segments;
		    if (!PassageToSegments(passage, &segments)) {
		      ADEBUG << "Failed to convert passage to lane segments.";
		      continue;
		    }

		    // step 3.1 project vehicle pos into passage  //Step 3.1 将当前车辆的坐标投影到Passage
		    PointENU nearest_point =  MakePointENU(adc_state_.x(), adc_state_.y(), adc_state_.z());
		    if (index == passage_index) {
		      nearest_point = adc_waypoint_.lane->GetSmoothPoint(adc_waypoint_.s);
		    }
		    common::SLPoint sl;
		    LaneWaypoint segment_waypoint;
		    if (!segments.GetProjection(nearest_point, &sl, &segment_waypoint)) {
		      ADEBUG << "Failed to get projection from point: " << nearest_point.ShortDebugString();
		      continue;
		    }

		    // Step 3.2 检查Passage是否可驶入 // step 3.2 check if an drive to passage
		    if (index != passage_index) {
		      if (!segments.CanDriveFrom(adc_waypoint_)) {
		        ADEBUG << "You cannot drive from current waypoint to passage: " << index;
		        continue;
		      }
		    }

		    // Step 3.3 生成RouteSegmens
		    // step 3.3 extend segment use backward_length and forward_length
		    // 对上述通过可驶入合法性检查的车道进行道路段的生成，同时使用backward_length和backward_length前后扩展道路段
		    route_segments->emplace_back();
		    const auto last_waypoint = segments.LastWaypoint();
		    if (!ExtendSegments(segments, sl.s() - backward_length, sl.s() + forward_length, &route_segments->back())) {	//原本车辆在passage中的投影点累计距离为sl.s(注意这个s是投影点在passage段起点的累计距离，而非整个road的累计距离)，扩展后前向增加forward_length，后向增加backward_length
		      AERROR << "Failed to extend segments with s=" << sl.s()  << ", backward: " << backward_length << ", forward: " << forward_length;
		      return false;
		    }

		    // Step 3.4 设置RouteSegments属性
		    if (route_segments->back().IsWaypointOnSegment(last_waypoint)) {
		      route_segments->back().SetRouteEndWaypoint(last_waypoint);
		    }

		    route_segments->back().SetCanExit(passage.can_exit());					//是否可以退出通道(最后一段Segment决定)
		    route_segments->back().SetNextAction(passage.change_lane_type());		// 下一步的动作(最后一段Segment决定)

		    // RouteSegments的id和是否是目的地
		    std::string route_segment_id = std::to_string(road_index) + "_" + std::to_string(index);
		    route_segments->back().SetId(route_segment_id);
		    route_segments->back().SetStopForDestination(stop_for_destination_);

		    // 设置上时刻的状态
		    if (index == passage_index) {
		      route_segments->back().SetIsOnSegment(true);
		      route_segments->back().SetPreviousAction(routing::FORWARD);
		    } else if (sl.l() > 0) {
		      route_segments->back().SetPreviousAction(routing::RIGHT);  // 如果当前车辆在passage左侧，那么车辆肯定需要向右变道到pass
		    } else {
		      route_segments->back().SetPreviousAction(routing::LEFT);	 // 如果当前车辆在passage右侧，那么车辆肯定需要向左变道到passage
		    }
		  }
		  return !route_segments->empty();
		}


7.  路由段RouteSegments到最终路径的生成
	std::list<RouteSegments> *const route_segments

	在上一个功能产生无人车短期规划路径，也就是行驶区域后，得到一个list的类型的结果，其中list中每个元素都是一个可能的形式路径。
	而最终路径的生成就是将上述的RouteSegments各个段，整合在一起并且拼接出一条完整的路径。
	这条最终的路径(Path类型)与HD Map中的车道Lane属性有点相似，不光是包含了各个LaneSegment，还有路径上的采样点，每个点的方向heading，以及减速带，交叉口，停车区的覆盖区域

	RouteSegments中的每个段包含的属性有：
		所属车道id
		起始点累积距离start_s
		结束点累积距离end_s

	而在高精地图HD Map中，车道Lane的结构存储是拆分成若干LineSegment2d段，每个段都有上述三个属性(lane_id, start_s, end_s)，也包含这个段的方向(unit_direction)。
	此外，还采样了这条Lane中的若干路径点point作为轨迹来保存，所以每个段LineSegment2d中都包含有一些采样点。采样点机制可以更加细节化刻画这条车道的属性


	但是RouteSegments中的段与HD Map中Lane的LineSegment2d又是不一样的。虽然两者都有上述三个属性，但是前者仅仅为了存储道路段，可以是很长的曲线道路段，因此只需要有lane_id, start_s和end_s即可，表示这条路起始到最终的路段，没有路段的细节信息；
	而后者是为了将Lane离散化采样存储，所以LineSegment2d是很短的道路段，长度不会太长(否则会丢失信息)，而且只能是直线段，同时额外包含细节信息，比如段的方向heading，段起始点坐标start_pos，段终点坐标end_pos等

	RouteSegments路径点生成的主要目的，是查询HD Map，根据其车道Lane段内的采样点point，将这个大而宏观的的RouteSegment划分成小的离散的轨迹点MapPathPoint，然后这些点两两组合成一个小的LineSegment2d，顺便计算这个段的方向heading(结束点坐标-起始点坐标)，
	这样就可以将行驶区域与Lane一样，利用更小的"LineSegment2d"来保存


	所以总结一下：下面的工作主要有4步:
		A. RouteSegments离散化MapPathPoint
		B. 离散LaneSegment2d && 重构RouteSegment的LaneSegment
		C. 道路采样点生成
		D. 覆盖区域设置	


	// A. RouteSegments离散化MapPathPoint
		针对每个可行驶区域方案(RouteSegments，由多个段组成)，生成段内的MapPathPoint路径点，言外之意将RouteSegments以路径点的形式进一步离散化

		apollo/modules/map/pnc_map/pnc_map.cc

		// 除了原本lane中的point封装成MapPathPoint，还加入了段起点和终点作为新的MapPathPoint，所以可能在接壤或者开始或者结尾部分，某些点的距离过近，PncMap::CreatePathFromLaneSegments函数中使用RemoveDuplicates(&points)完成去冗余点功能
		void PncMap::AppendLaneToPoints(LaneInfoConstPtr lane, const double start_s, const double end_s, std::vector<MapPathPoint> *const points) {
		  if (points == nullptr || start_s >= end_s) {
		    return;
		  }
		  double accumulate_s = 0.0;
		  for (size_t i = 0; i < lane->points().size(); ++i) {
		    if (accumulate_s >= start_s && accumulate_s <= end_s) {  	// 封装中间点point
		      points->emplace_back(lane->points()[i], lane->headings()[i], LaneWaypoint(lane, accumulate_s));
		    }
		    if (i < lane->segments().size()) {
		      const auto &segment = lane->segments()[i];
		      const double next_accumulate_s = accumulate_s + segment.length();

		      if (start_s > accumulate_s && start_s < next_accumulate_s) {		// 封装段起点waypoint
		        points->emplace_back(  segment.start() +    segment.unit_direction() * (start_s - accumulate_s),  lane->headings()[i], LaneWaypoint(lane, start_s));
		      }
		      if (end_s > accumulate_s && end_s < next_accumulate_s) {		 // 封装段终点waypoint
		        points->emplace_back(  segment.start() + segment.unit_direction() * (end_s - accumulate_s),  lane->headings()[i], LaneWaypoint(lane, end_s));
		      }
		      accumulate_s = next_accumulate_s;
		    }
		    if (accumulate_s > end_s) {
		      break;
		    }
		  }
		}


	// 离散LaneSegment2d && 重构RouteSegment的LaneSegment