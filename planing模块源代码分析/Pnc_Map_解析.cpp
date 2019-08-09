
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


		  RouteSegments source_segments;
		  if (!PassageToSegments(source_passage, &source_segments)) {
		    AERROR << "failed to convert passage to segments";
		    return result;
		  }

		  if (next_routing_waypoint_index_ < routing_waypoint_index_.size() &&  source_segments.IsWaypointOnSegment(routing_waypoint_index_[next_routing_waypoint_index_].waypoint) ) {
		    ADEBUG << "need to pass next waypoint[" << next_routing_waypoint_index_
		           << "] before change lane";
		    return result;
		  }



		  std::unordered_set<std::string> neighbor_lanes;
		  if (source_passage.change_lane_type() == routing::LEFT) {
		    for (const auto &segment : source_segments) {
		      for (const auto &left_id :
		           segment.lane->lane().left_neighbor_forward_lane_id()) {
		        neighbor_lanes.insert(left_id.id());
		      }
		    }
		  } else if (source_passage.change_lane_type() == routing::RIGHT) {
		    for (const auto &segment : source_segments) {
		      for (const auto &right_id :
		           segment.lane->lane().right_neighbor_forward_lane_id()) {
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
		      if (neighbor_lanes.count(segment.id())) {
		        result.emplace_back(i);
		        break;
		      }
		    }
		  }
		  return result;
		}