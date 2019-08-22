
1. // 高精地图在Apollo中的流转形式

	xml格式的地图见modules/map/data/sunnyvale_big_loop/base_map.xml文件。那接下来我们就看下从XML解析到proto的过程

	modules/map/hdmap/adapter/opendrive_adapter.cc

	bool OpendriveAdapter::LoadData(const std::string& filename,  apollo::hdmap::Map* pb_map) {
	  CHECK_NOTNULL(pb_map);

	  tinyxml2::XMLDocument document;
	  if (document.LoadFile(filename.c_str()) != tinyxml2::XML_SUCCESS) {
	    AERROR << "fail to load file " << filename;
	    return false;
	  }

	  // root node  // 1、根目录
	  const tinyxml2::XMLElement* root_node = document.RootElement();
	  CHECK(root_node != nullptr);
	  // header  // 2、header
	  PbHeader* map_header = pb_map->mutable_header();
	  Status status = HeaderXmlParser::Parse(*root_node, map_header);
	  if (!status.ok()) {
	    AERROR << "fail to parse opendrive header, " << status.error_message();
	    return false;
	  }
	  //  roads //3、道路
	  std::vector<RoadInternal> roads;
	  status = RoadsXmlParser::Parse(*root_node, &roads);
	  if (!status.ok()) {
	    AERROR << "fail to parse opendrive road, " << status.error_message();
	    return false;
	  }
	  // junction  //4、路口
	  std::vector<JunctionInternal> junctions;
	  status = JunctionsXmlParser::Parse(*root_node, &junctions);
	  if (!status.ok()) {
	    AERROR << "fail to parse opendrive junction, " << status.error_message();
	    return false;
	  }

	  ProtoOrganizer proto_organizer;
	  proto_organizer.GetRoadElements(&roads);
	  proto_organizer.GetJunctionElements(junctions);
	  proto_organizer.GetOverlapElements(roads, junctions);
	  proto_organizer.OutputData(pb_map);

	  return true;
	}





2. //对地图的操作方法
	有了原始从xml格式到protobuf的数据之后，就可以访问这些高精地图的元素，Apollo高精地图提供如下的方法获取元素：

	modules/map/hdmap/hdmap_impl.cc

	LoadMapFromFile (const std::string &map_filename) 从本地文件加载地图 
	GetLaneById (const Id &id) const
	GetJunctionById (const Id &id) const
	GetSignalById (const Id &id) const
	GetCrosswalkById (const Id &id) const
	GetStopSignById (const Id &id) const
	GetYieldSignById (const Id &id) const
	GetClearAreaById (const Id &id) const
	GetSpeedBumpById (const Id &id) const
	GetOverlapById (const Id &id) const
	GetRoadById (const Id &id) const
	// 在确定范围获取所有车道
	GetLanes (const apollo::common::PointENU &point, double distance, std::vector< LaneInfoConstPtr > *lanes) const 
	// 在确定范围获取所有路口区域
	GetJunctions (const apollo::common::PointENU &point, double distance, std::vector< JunctionInfoConstPtr > *junctions) const 
	// 在确定范围内获取所有人行道
	GetCrosswalks (const apollo::common::PointENU &point, double distance, std::vector< CrosswalkInfoConstPtr > *crosswalks) const 
	// 获取确定范围的所有信号灯
	GetSignals (const apollo::common::PointENU &point, double distance, std::vector< SignalInfoConstPtr > *signals) const
	// 获取确定范围内的所有停止标识
	GetStopSigns (const apollo::common::PointENU &point, double distance, std::vector< StopSignInfoConstPtr > *stop_signs) const
	// 获取确定范围内的所有避让标识
	GetYieldSigns (const apollo::common::PointENU &point, double distance, std::vector< YieldSignInfoConstPtr > *yield_signs) const
	// 获取确定范围内的所有禁止停车标识
	GetClearAreas (const apollo::common::PointENU &point, double distance, std::vector< ClearAreaInfoConstPtr > *clear_areas) const
	// 获取确定范围内的所有减速带
	GetSpeedBumps (const apollo::common::PointENU &point, double distance, std::vector< SpeedBumpInfoConstPtr > *speed_bumps) const
	// 获取确定范围内的所有道路
	GetRoads (const apollo::common::PointENU &point, double distance, std::vector< RoadInfoConstPtr > *roads) const
	// 获取从目标点的最近车道
	GetNearestLane (const apollo::common::PointENU &point, LaneInfoConstPtr *nearest_lane, double *nearest_s, double *nearest_l) const
	// 判断车辆姿态，获取在一定范围内最近的车道
	GetNearestLaneWithHeading (const apollo::common::PointENU &point, const double distance, const double central_heading, const double max_heading_difference, LaneInfoConstPtr *nearest_lane, double *nearest_s, double *nearest_l) const
	// 判断车辆姿态，获取所有车道
	GetLanesWithHeading (const apollo::common::PointENU &point, const double distance, const double central_heading, const double max_heading_difference, std::vector< LaneInfoConstPtr > *lanes) const

	// 获取确定范围内的所有道路和路口边界
	GetRoadBoundaries (const apollo::common::PointENU &point, double radius, std::vector< RoadROIBoundaryPtr > *road_boundaries, std::vector< JunctionBoundaryPtr > *junctions) const
	// 如果有两个与一条停止线相关的信号，则在车道上的某个范围内前进最近的信号，返回两个信号。
	GetForwardNearestSignalsOnLane (const apollo::common::PointENU &point, const double distance, std::vector< SignalInfoConstPtr > *signals) const





3. // 获取元素实例

	modules/planning/reference_line/reference_line_provider.cc
	
	bool ReferenceLineProvider::GetReferenceLinesFromRelativeMap( const relative_map::MapMsg &relative_map, std::list<ReferenceLine> *reference_lines, std::list<hdmap::RouteSegments> *segments) {
	  DCHECK_GE(relative_map.navigation_path_size(), 0);
	  DCHECK_NOTNULL(reference_lines);
	  DCHECK_NOTNULL(segments);

	  if (relative_map.navigation_path().empty()) {
	    AERROR << "There isn't any navigation path in current relative map.";
	    return false;
	  }

	  auto *hdmap = HDMapUtil::BaseMapPtr();
	  if (!hdmap) {
	    AERROR << "hdmap is null";
	    return false;
	  }

	  // 1.get adc current lane info ,such as lane_id,lane_priority,neighbor lanes
	  std::unordered_set<std::string> navigation_lane_ids;
	  for (const auto &path_pair : relative_map.navigation_path()) {
	    const auto lane_id = path_pair.first;
	    navigation_lane_ids.insert(lane_id);
	  }
	  if (navigation_lane_ids.empty()) {
	    AERROR << "navigation path ids is empty";
	    return false;
	  }
	  // get current adc lane info by vehicle state
	  common::VehicleState vehicle_state = common::VehicleStateProvider::Instance()->vehicle_state();
	  hdmap::LaneWaypoint adc_lane_way_point;
	  if (!GetNearestWayPointFromNavigationPath(vehicle_state, navigation_lane_ids, &adc_lane_way_point)) {
	    return false;
	  }
	  const std::string adc_lane_id = adc_lane_way_point.lane->id().id();
	  auto adc_navigation_path = relative_map.navigation_path().find(adc_lane_id);
	  if (adc_navigation_path == relative_map.navigation_path().end()) {
	    AERROR << "adc lane cannot be found in relative_map.navigation_path";
	    return false;
	  }
	  const uint32_t adc_lane_priority = adc_navigation_path->second.path_priority();
	  // get adc left neighbor lanes
	  std::vector<std::string> left_neighbor_lane_ids;
	  auto left_lane_ptr = adc_lane_way_point.lane;
	  while (left_lane_ptr != nullptr && left_lane_ptr->lane().left_neighbor_forward_lane_id_size() > 0) {
	    auto neighbor_lane_id = left_lane_ptr->lane().left_neighbor_forward_lane_id(0);
	    left_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
	    left_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);   // 从高精地图中获取对应车道
	  }
	  ADEBUG << adc_lane_id << " left neighbor size : " << left_neighbor_lane_ids.size();
	  for (const auto &neighbor : left_neighbor_lane_ids) {
	    ADEBUG << adc_lane_id << " left neighbor : " << neighbor;
	  }
	  // get adc right neighbor lanes
	  std::vector<std::string> right_neighbor_lane_ids;
	  auto right_lane_ptr = adc_lane_way_point.lane;
	  while (right_lane_ptr != nullptr && right_lane_ptr->lane().right_neighbor_forward_lane_id_size() > 0) {
	    auto neighbor_lane_id = right_lane_ptr->lane().right_neighbor_forward_lane_id(0);
	    right_neighbor_lane_ids.emplace_back(neighbor_lane_id.id());
	    right_lane_ptr = hdmap->GetLaneById(neighbor_lane_id);
	  }
	  ADEBUG << adc_lane_id << " right neighbor size : " << right_neighbor_lane_ids.size();
	  for (const auto &neighbor : right_neighbor_lane_ids) {
	    ADEBUG << adc_lane_id << " right neighbor : " << neighbor;
	  }
	  // 2.get the higher priority lane info list which priority higher
	  // than current lane and get the highest one as the target lane
	  using LaneIdPair = std::pair<std::string, uint32_t>;
	  std::vector<LaneIdPair> high_priority_lane_pairs;
	  ADEBUG << "relative_map.navigation_path_size = " << relative_map.navigation_path_size();
	  for (const auto &path_pair : relative_map.navigation_path()) {
	    const auto lane_id = path_pair.first;
	    const uint32_t priority = path_pair.second.path_priority();
	    ADEBUG << "lane_id = " << lane_id << " priority = " << priority  << " adc_lane_id = " << adc_lane_id << " adc_lane_priority = " << adc_lane_priority;
	    // the smaller the number, the higher the priority
	    if (adc_lane_id != lane_id && priority < adc_lane_priority) {
	      high_priority_lane_pairs.emplace_back(lane_id, priority);
	    }
	  }
	  // get the target lane
	  bool is_lane_change_needed = false;
	  LaneIdPair target_lane_pair;
	  if (!high_priority_lane_pairs.empty()) {
	    std::sort(high_priority_lane_pairs.begin(), high_priority_lane_pairs.end(),  [](const LaneIdPair &left, const LaneIdPair &right) {
	                return left.second < right.second;
	              });
	    ADEBUG << "need to change lane";
	    // the higheast priority lane as the target naviagion lane
	    target_lane_pair = high_priority_lane_pairs.front();
	    is_lane_change_needed = true;
	  }
	  // 3.get current lane's the neareast neighbor lane to the target lane
	  // and make sure it position is left or right on the current lane
	  routing::ChangeLaneType lane_change_type = routing::FORWARD;
	  std::string neareast_neighbor_lane_id;
	  if (is_lane_change_needed) {
	    // target on the left of adc
	    if (left_neighbor_lane_ids.end() !=
	        std::find(left_neighbor_lane_ids.begin(), left_neighbor_lane_ids.end(),
	                  target_lane_pair.first)) {
	      // take the id of the first adjacent lane on the left of adc as
	      // the neareast_neighbor_lane_id
	      neareast_neighbor_lane_id =
	          adc_lane_way_point.lane->lane().left_neighbor_forward_lane_id(0).id();
	    } else if (right_neighbor_lane_ids.end() !=
	               std::find(right_neighbor_lane_ids.begin(),
	                         right_neighbor_lane_ids.end(),
	                         target_lane_pair.first)) {
	      // target lane on the right of adc
	      // take the id  of the first adjacent lane on the right of adc as
	      // the neareast_neighbor_lane_id
	      neareast_neighbor_lane_id = adc_lane_way_point.lane->lane()
	                                      .right_neighbor_forward_lane_id(0)
	                                      .id();
	    }
	  }

	  for (const auto &path_pair : relative_map.navigation_path()) {
	    const auto &lane_id = path_pair.first;
	    const auto &path_points = path_pair.second.path().path_point();
	    auto lane_ptr = hdmap->GetLaneById(hdmap::MakeMapId(lane_id));
	    RouteSegments segment;
	    segment.emplace_back(lane_ptr, 0.0, lane_ptr->total_length());
	    segment.SetCanExit(true);
	    segment.SetId(lane_id);
	    segment.SetNextAction(routing::FORWARD);
	    segment.SetStopForDestination(false);
	    segment.SetPreviousAction(routing::FORWARD);

	    if (is_lane_change_needed) {
	      if (lane_id == neareast_neighbor_lane_id) {
	        ADEBUG << "adc lane_id = " << adc_lane_id
	               << " neareast_neighbor_lane_id = " << lane_id;
	        segment.SetIsNeighborSegment(true);
	        segment.SetPreviousAction(lane_change_type);
	      } else if (lane_id == adc_lane_id) {
	        segment.SetIsOnSegment(true);
	        segment.SetNextAction(lane_change_type);
	      }
	    }

	    segments->emplace_back(segment);
	    std::vector<ReferencePoint> ref_points;
	    for (const auto &path_point : path_points) {
	      ref_points.emplace_back(
	          MapPathPoint{Vec2d{path_point.x(), path_point.y()},
	                       path_point.theta(),
	                       LaneWaypoint(lane_ptr, path_point.s())},
	          path_point.kappa(), path_point.dkappa());
	    }
	    reference_lines->emplace_back(ref_points.begin(), ref_points.end());
	    reference_lines->back().SetPriority(path_pair.second.path_priority());
	  }
	  return !segments->empty();
	}




4. // map的代码目录结构

	├── data           // 生成好的地图
	│   └── demo
	├── hdmap          // 高精度地图
	│   ├── adapter    // 从xml文件读取地图(opendrive保存格式为xml)
	│   │   └── xml_parser
	│   └── test-data
	├── pnc_map        // 给规划控制模块用的地图
	│   └── testdata
	├── proto          // 地图各元素的消息格式(人行横道，车道线等)
	├── relative_map   // 相对地图
	│   ├── common
	│   ├── conf
	│   ├── dag
	│   ├── launch
	│   ├── proto
	│   ├── testdata
	│   │   └── multi_lane_map
	│   └── tools
	├── testdata       // 测试数据？
	│   └── navigation_dummy
	└── tools          // 工具


	地图的读取在adapter中，其中xml_parser目录提供解析xml的能力。而opendrive_adapter.cc则实现了地图的加载，转换为程序中的Map对象。然后地图在hdmap_impl.cc中提供一系列api接口给其他模块使用。