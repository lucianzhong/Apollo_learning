
1.  ReferenceLine
	参考线是整个决策规划算法的基础。从前面的内容我们也看到了，在Planning模块的每个计算循环中，会先生成ReferencePath，然后在这个基础上进行后面的处理。例如：把障碍物投影到参考线上。

	/modules/planning/reference_line/reference_line_provider.h

	bool GetReferenceLines(std::list<ReferenceLine>* reference_lines, std::list<hdmap::RouteSegments>* segments);

	planning涉及到三个模块:
	routing模块，这部分内容已经在Routing模块一文中讲解过，本文不再赘述。
	pnc_map模块：负责读取和处理Routing搜索结果。
	planning模块：根据Routing结果和车辆的实时状态（包括周边环境）生成参考线和轨迹。


	在Planning模块中有以下三个数据结构将是本文关注的重点：

	ReferenceLine：原始参考线，源码位于planning/reference_line/目录下。根据Routing的搜索结果生成。

	ReferenceLineInfo：源码位于planning/common/目录下。Planning实现中，逻辑计算的基础数据结构，很多操作都会在这个数据结构上进行（例如：交通规则逻辑，障碍物投影，路径优化，速度决策等）。本文中的“参考线”一词将不区分ReferenceLine和ReferenceLineInfo两个结构。

	Trajectory：下文中我们将看到，有好几个结构用来描述轨迹。它们在不同的场合下使用。这其中，ADCTrajectory是Planning模块的输出。它是Planning模块一次计算循环中，处理了所有逻辑的最终结果，包含了车辆行驶需要的所有信息。因此，这个数据将直接影响到自动驾驶车辆的行车行为。




2. 决策规划模块负责生成车辆的行驶轨迹。要做到这一点，决策规划模块需要从宏观到局部经过三个层次来进行决策:

	1.第一个层次是Routing的搜索结果。Routing模块的输入是若干个按顺序需要达到的途径点（也可能只有一个起点和终点）。
	  Routing模块根据地图的拓扑结构搜索出可达的完整路线来，这个路线的长度可能是几公里甚至几百公里。因此这个是最为宏观的数据。另外，Routing的搜索结果是相对固定的。在没有障碍物的情况下，车辆会一直沿着原先获取到的Routing路线行驶。
	  只有当车辆驶出了原先规划的路线之外（例如：为了避障），才会重新发送请求给Routing模块，以重新计算路线。

	2. 第二个层次就是参考线。决策规划模块会实时的根据车辆的具体位置来计算参考线。参考线的计算会以Routing的路线为基础。但同时，参考线会考虑车辆周边的动态信息，例如：障碍物，交通规则等。参考线是包含车辆所在位置周边一定的范围，通常是几百米的长度。
	   Routing结果，它是较为局部的数据。

	3.层次是轨迹。轨迹是决策规划模块的最终输出结果。它的依据是参考线。在同一时刻，参考线可能会有多条，例如：在变道的时候，自车所在车道和目标车道都会有一条参考线。而轨迹，是在所有可能的结果中，综合决策和优化的结果，最终的唯一结果。
	  因此它是更为具体和局部的数据。轨迹不仅仅包含了车辆的路线，还包含了车辆行驶这条路线时的详细状态，例如：车辆的方向，速度，加速度等等。 


	  在Planning模块一文中我们已经提到：参考线是整个决策规划算法的基础。在Planning模块的每个计算循环中，都会先生成参考线，然后在这个基础上进行后面的处理，例如：交通规则逻辑，障碍物投影，路径优化，速度决策等等。可以说，参考线贯穿了整个Planning模块的实现。



3. pnc_map
	pnc全称是Planning And Control。这是Planning用来对接Routing搜索结果的子模块

	 modules/common/proto/geometry.proto  //PointENU：描述了地图上的一个点
	 modules/common/proto/pnc_point.proto //SLPoint：描述了Frenet坐标系上的一个点。s表示距离起点的纵向距离，l表示距离中心线的侧向距离
	 modules/map/pnc_map/path.h      // LaneWaypoint：描述了车道上的点
	 modules/common/vehicle_state/proto/vehicle_state.proto    //VehicleState：描述车辆状态，包含了自车位置，姿态，方向，速度，加速度等信息

	
	RouteSegments:
	我们回顾一下，Routing的搜索结果RoutingResponse中包含了下面三个层次的结构：

		RoadSegment：描述道路，一条道路可能包含了并行的几条通路（Passage）
		Passage：描述通路，通路是直连不含变道的可行驶区域。一个通路可能包含了前后连接的多个车道
		LaneSegment：描述车道，车道是道路中的一段，自动驾驶车辆会尽可能沿着车道的中心线行驶
		
	而pnc_map模块中的RouteSegments对应了上面的Passage结构，它其中会包含若干个车道信息。这个类继承自std::vector<LaneSegment>



	RouteSegments中有如下一些方法值得关注：
	NextAction()：车辆接下来要采取的动作。可能是直行，左变道，或者右变道。
	CanExit()：当前通路是否可以接续到Routing结果的另外一个通路上。
	GetProjection()：将一个点投影到当前通路上。返回SLPoint和LaneWaypoint。
	Stitch()：缝合另外一个RouteSegments。即：去除两个RouteSegments间重合的多余部分，然后连接起来。
	Shrink()：缩短到指定范围。
	IsOnSegment()：车辆是否在当前RouteSegments上。
	IsNeighborSegment()：当前RouteSegments是否是车辆的临近RouteSegments


	PncMap类负责对接Routing搜索结果的更新
	PncMap会根据车辆当前位置，提供车辆周边的RouteSegments信息供ReferenceLineProvider生成ReferenceLine。“车辆周边”与车辆的纵向和横向相关

	对于纵向来说，PncMap返回的结果是前后一定范围内的。具体的范围由下面三个值决定：

	modules/map/pnc_map/pnc_map.cc:

	DEFINE_double(look_backward_distance, 30,"look backward this distance when creating reference line from routing"); //向后是30米的范围

	DEFINE_double(look_forward_short_distance, 180,"short look forward this distance when creating reference line " "from routing when ADC is slow");//向前是根据车辆的速度返回180米或者250米的范围

	DEFINE_double(look_forward_long_distance, 250,"look forward this distance when creating reference line from routing");

	对于横向来说，如果Routing的搜索结果包含变道的信息。则PncMap提供的数据会包含自车所在车道和变道后的相关通路


	/modules/map/pnc_map/pnc_map.c
	PncMap中的下面这个方法用来接收车辆的状态更新。当然，这其中很重要的就是位置状态。
	bool PncMap::UpdateVehicleState(const VehicleState &vehicle_state)






4. ReferenceLine结构

	前面我们已经说了，参考线是根据车辆位置相对局部的一个数据，它包含了车辆前后一定范围内的路径信息。在车辆行驶过程中，Planning会在每一个计算周期中生成ReferenceLine.

	modules/planning/reference_line/reference_line.h:

	
		// speed_limit_是限速数据
	  struct SpeedLimit {
	    double start_s = 0.0;
	    double end_s = 0.0;
	    double speed_limit = 0.0;  // unit m/s
	    SpeedLimit() = default;
	    SpeedLimit(double _start_s, double _end_s, double _speed_limit)
	        : start_s(_start_s), end_s(_end_s), speed_limit(_speed_limit) {}
	  };
	  /**
	   * This speed limit overrides the lane speed limit
	   **/
	  std::vector<SpeedLimit> speed_limit_;
	  std::vector<ReferencePoint> reference_points_;
	  hdmap::Path map_path_;	//reference_points_其实是从map_path_得到，具体见ReferenceLine的构造函数。所以这两个数据的作用其实是一样的。
	  uint32_t priority_ = 0;	//priority_是优先级，不过在PublicRoadPlanner中没有用到




	  // reference_points_其实是从map_path_得到，具体见ReferenceLine的构造函数。所以这两个数据的作用其实是一样的。
	  /modules/planning/reference_line/reference_line.cc:

	  ReferenceLine::ReferenceLine(const MapPath& hdmap_path): map_path_(hdmap_path) {
	  for (const auto& point : hdmap_path.path_points()) {
	    DCHECK(!point.lane_waypoints().empty());
	    const auto& lane_waypoint = point.lane_waypoints()[0];
	    reference_points_.emplace_back( hdmap::MapPathPoint(point, point.heading(), lane_waypoint), 0.0, 0.0 );
	  }
	  CHECK_EQ(map_path_.num_points(), reference_points_.size());
	}


5. std::vector<ReferencePoint>是一系列的点，点包含了位置的信息。因此这些点就是生成车辆行驶轨迹的基础数据:

	ReferencePoint由MapPathPoint继承而来

	/modules/common/math/vec2d.h:
	Vec2d描述一个二维的点，包含的数据成员如下：
	double x_：描述点的x坐标
	double y_：描述点的y坐标


	/modules/map/pnc_map/path.h
	MapPathPoint描述了一个地图上的点，包含的数据成员如下：
	double heading_：描述点的朝向。
	std::vector<LaneWaypoint> lane_waypoints_：描述路径上的点。有些车道可能会存在重合的部分，所以地图上的一个点可能同时属于多个车道，因此这里的数据是一个vector结构


	/modules/planning/reference_line/reference_point.h
	ReferencePoint描述了参考线中的点，包含的数据成员如下：
	double kappa_：描述曲线的曲率
	double dkappa_：描述曲率的导数	


	如果你打开Apollo项目中的demo地图文件你就会发现，地图中记录的仅仅是每个点x和y坐标，并没有记录heading和kappa数据。事实上，这些数据都是在读取地图原始点数据之后计算出来的.// /modules/map/data/demo/base_map.txt


6. 创建ReferenceLine

	在每一次计算循环中，Planning模块都会通过ReferenceLineProvider生成ReferenceLine。ReferenceLine由Routing的搜索结果决定。Routing是预先搜索出的全局可达路径，而ReferenceLine是车辆当前位置的前后一段范围
	直行的情况下，ReferenceLine是一个。而在需要变道的时候，会有多个ReferenceLine

	bool ReferenceLineProvider::CreateReferenceLine(
	    std::list<ReferenceLine> *reference_lines,
	    std::list<hdmap::RouteSegments> *segments) {
	  CHECK_NOTNULL(reference_lines);
	  CHECK_NOTNULL(segments);

	  common::VehicleState vehicle_state;
	  {
	    std::lock_guard<std::mutex> lock(vehicle_state_mutex_);
	    vehicle_state = vehicle_state_;
	  }

	  routing::RoutingResponse routing;
	  {
	    std::lock_guard<std::mutex> lock(routing_mutex_);
	    routing = routing_;
	  }
	  bool is_new_routing = false;
	  {
	    // Update routing in pnc_map
	    if (pnc_map_->IsNewRouting(routing)) {					// PncMap对接了Routing的搜索结果。如果Routing的路线变了，这里需要进行更新
	      is_new_routing = true;
	      if (!pnc_map_->UpdateRoutingResponse(routing)) { 	
	        AERROR << "Failed to update routing in pnc map";
	        return false;
	      }
	    }
	  }

	  if (!CreateRouteSegments(vehicle_state, segments)) {				// 车辆的位置一直会变动（vehicle_state中包含了这个信息）。如果Routing的结果需要变道，则segments将是多个，否则就是一个（直行的情况）
	    AERROR << "Failed to create reference line from routing";		// CreateRouteSegments方法中会调用pnc_map_->GetRouteSegments(vehicle_state, segments)来获取车辆当前位置周边范围的RouteSegment
	    	    return false;											// 如果Routing的结果需要变道，则segments将是多个，否则就是一个（直行的情况）
	  }

	  if (is_new_routing || !FLAGS_enable_reference_line_stitching) {
	    for (auto iter = segments->begin(); iter != segments->end();) {
	      reference_lines->emplace_back();
	      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
	        AERROR << "Failed to create reference line from route segments";
	        reference_lines->pop_back();
	        iter = segments->erase(iter);
	      } else {
	        ++iter;
	      }
	    }
	    return true;
	  } else {  // stitching reference line
	    for (auto iter = segments->begin(); iter != segments->end();) {
	      reference_lines->emplace_back();
	      if (!ExtendReferenceLine(vehicle_state, &(*iter), &reference_lines->back())) { // 大部分情况下，在车辆行驶过程中，会不停的根据车辆的位置对ReferenceLine进行长度延伸。ReferenceLine的长度是200多米的范围（往后30米左右，往前180米或者250米左右）
	        AERROR << "Failed to extend reference line";
	        reference_lines->pop_back();
	        iter = segments->erase(iter);
	      } else {
	        ++iter;
	      }
	    }
	  }
	  return true;
	}


	在车辆行驶过程中，必不可少的就是判断自车以及障碍物所处的位置。这就很自然的需要将物体投影到参考线上来进行计算

	ReferencePoint GetReferencePoint(const double s) const;
	common::FrenetFramePoint GetFrenetPoint(
	  const common::PathPoint& path_point) const;
	std::vector<ReferencePoint> GetReferencePoints(double start_s,
	                                             double end_s) const;
	size_t GetNearestReferenceIndex(const double s) const;
	ReferencePoint GetNearestReferencePoint(const common::math::Vec2d& xy) const;
	std::vector<hdmap::LaneSegment> GetLaneSegments(const double start_s,
	                                              const double end_s) const;
	ReferencePoint GetNearestReferencePoint(const double s) const;
	ReferencePoint GetReferencePoint(const double x, const double y) const;
	bool GetApproximateSLBoundary(const common::math::Box2d& box,
	                            const double start_s, const double end_s,
	                            SLBoundary* const sl_boundary) const;
	bool GetSLBoundary(const common::math::Box2d& box,
	                 SLBoundary* const sl_boundary) const;
	bool GetSLBoundary(const hdmap::Polygon& polygon,
	                 SLBoundary* const sl_boundary) const;



7.  ReferenceLineSmoother
	直接通过RouteSegments生成的ReferenceLine可能是不平滑的。	如果直接让车辆沿着不平滑的路线行驶可能造成车辆方向的抖动或者大幅变化，这对乘坐体验来说非常不好。因此，原始的路线数据需要经过算法的平滑。


	modules/planning/reference_line/reference_line_provider.cc

	bool ReferenceLineProvider::SmoothReferenceLine( const ReferenceLine &raw_reference_line, ReferenceLine *reference_line) {
	  if (!FLAGS_enable_smooth_reference_line) {
	    *reference_line = raw_reference_line;
	    return true;
	  }
	  // generate anchor points:
	  std::vector<AnchorPoint> anchor_points;
	  GetAnchorPoints(raw_reference_line, &anchor_points);
	  smoother_->SetAnchorPoints(anchor_points);
	  if (!smoother_->Smooth(raw_reference_line, reference_line)) {			// smoother_->Smooth才是平滑算法的实现
	    AERROR << "Failed to smooth reference line with anchor points";
	    return false;
	  }
	  if (!IsReferenceLineSmoothValid(raw_reference_line, *reference_line)) {
	    AERROR << "The smoothed reference line error is too large";
	    return false;
	  }
	  return true;
	}

	目前Apollo的Planning模块中内置了三个ReferenceLine平滑器,具体使用哪一个平滑器由ReferenceLineProvider在初始化的时候读取配置文件来决定

	CHECK(common::util::GetProtoFromFile(FLAGS_smoother_config_filename, &smoother_config_))

	modules/planning/conf/qp_spline_smoother_config.pb.txt //使用的是QpSplineReferenceLineSmoother

	modules/planning/common/planning_gflags.cc
	DEFINE_string(smoother_config_filename, "/apollo/modules/planning/conf/qp_spline_smoother_config.pb.txt", "The configuration file for qp_spline smoother");


	ReferenceLineSmoother算法:参考线平滑器使用了二次规划（Quadratic programming ）和样条插值（Spline interpolation）算法



8. 创建ReferenceLineInfo
	ReferenceLine主要是静态数据（路径点和限速）的存储，而ReferenceLineInfo中会包含动态信息（障碍物）和更多逻辑
	ReferenceLineInfo由Frame根据ReferenceLine和RouteSegments创建得到。在每个Planning计算循环的开始，都会创建和初始化一个新的Frame，而Frame初始化的时候就会创建ReferenceLineInfo。当有多个ReferenceLine的时候，则意味着需要变道

	bool Frame::CreateReferenceLineInfo(
	    const std::list<ReferenceLine> &reference_lines,
	    const std::list<hdmap::RouteSegments> &segments) {
	  reference_line_info_.clear();
	  auto ref_line_iter = reference_lines.begin();
	  auto segments_iter = segments.begin();
	  while (ref_line_iter != reference_lines.end()) {
	    if (segments_iter->StopForDestination()) {
	      is_near_destination_ = true;
	    }
	    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_, *ref_line_iter, *segments_iter);
	    ++ref_line_iter;
	    ++segments_iter;
	  }

	  if (reference_line_info_.size() == 2) {
	    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());
	    common::SLPoint first_sl;
	    if (!reference_line_info_.front().reference_line().XYToSL(xy_point, &first_sl)) {
	      return false;
	    }
	    common::SLPoint second_sl;
	    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,  &second_sl)) {
	      return false;
	    }
	    const double offset = first_sl.l() - second_sl.l();
	    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
	    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
	  }


9. 操作ReferenceLineInfo

	主要有下面两个地方会操作ReferenceLineInfo：
	
	/modules/planning/traffic_rules：该目录下是交通规则的实现。不同Rule会向ReferenceLineInfo添加不同的数据，例如：障碍物，红绿灯等等。交通规则的应用是在TrafficDecider::Execute方法中执行的。

	/modules/planning/tasks：该目录下是许多的决策器和优化器，这是Apollo EM Planner算法的核心。不过前面已经说过，由于篇幅所限，本文不会涉及这些内容。


   planning/traffic_rules/traffic_rule.h
  每个交通规则都会实现下面这个方法来完成其逻辑：
  virtual common::Status ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) = 0;


  对于ReferenceLineInfo的操作，主要是修改该类的以下三个字段：

	PathData path_data_：包含了路径相关的数据，逻辑实现位于modules/planning/common/path/中。
	SpeedData speed_data_：包含了速度相关的数据，逻辑实现位于modules/planning/common/speed/。路径和速度最终将组合起来使用，以生成行车轨迹（见下文）。
	PathDecision path_decision_：这个字段中包含了障碍物的决策信息


	障碍物在Planning模块中通过apollo::planning::Obstacle描述。

	障碍物分为横向障碍物和纵向障碍物。横向障碍物将可能导致车辆的nudge行为。而纵向障碍物可能导致车辆出现：stop，yield，follow，overtake行为。这几个行为的优先级从左到右依次递减。

	预测模块对于同一个障碍物可能会有多个预测轨迹。此时在Planning模块中，会多个apollo::planning::Obstacle对象分别对应每一个轨迹。


10. 轨迹结构
	
	就和高精地图通过一系列的点来描述道路的中心线一样，车辆的行驶轨迹也是由一系列的点来描述的。具体的结构在proto文件中定义
	modules/common/proto/pnc_point.proto

	message PathPoint {				//path_point：PathPoint类型数据。描述了一个点的位置，曲率，朝向，所属车道等信息
	  // coordinates
	  optional double x = 1;
	  optional double y = 2;
	  optional double z = 3;

	  // direction on the x-y plane
	  optional double theta = 4;
	  // curvature on the x-y planning
	  optional double kappa = 5;
	  // accumulated distance from beginning of the path
	  optional double s = 6;

	  // derivative of kappa w.r.t s.
	  optional double dkappa = 7;
	  // derivative of derivative of kappa w.r.t s.
	  optional double ddkappa = 8;
	  // The lane ID where the path point is on
	  optional string lane_id = 9;

	  // derivative of x and y w.r.t parametric parameter t in CosThetareferenceline
	  optional double x_derivative = 10;
	  optional double y_derivative = 11;
	}

	message Path {
	  optional string name = 1;
	  repeated PathPoint path_point = 2;
	}

	message TrajectoryPoint {
	  // path point
	  optional PathPoint path_point = 1;
	  // linear velocity
	  optional double v = 2;  // in [m/s]
	  // linear acceleration
	  optional double a = 3;
	  // relative time from beginning of the trajectory //描述车辆达到该点的相对时间（以轨迹的开始为起点）
	  optional double relative_time = 4;
	  // longitudinal jerk //加速度的导数，也称之为jerk
	  optional double da = 5;
	  // The angle between vehicle front wheel and vehicle longitudinal axis  //车辆的前轮方向
	  optional double steer = 6;
	}



	TrajectoryPoint 仅仅是一个点。而一条轨迹一定是由许多个点构成的。因此，描述轨迹的类DiscretizedTrajectory继承自std::vector<common::TrajectoryPoint>
	class DiscretizedTrajectory : public std::vector<common::TrajectoryPoint>

	DiscretizedTrajectory还有一个子类PublishableTrajectory
	modules/planning/common/trajectory/publishable_trajectory.h
	PublishableTrajectory相较于DiscretizedTrajectory来说，最主要的提供了下面这个方法：explicit PublishableTrajectory(const ADCTrajectory& trajectory_pb);

	这个方法是将Planning模块中的轨迹数据结构导出到可发布的状态。这里的ADCTrajectory是Planning最终往外发出的数据结构，它也是在proto文件中定义的:modules/planning/proto/planning.proto



11. 生成轨迹

	从参考线到生成轨迹是由ReferenceLineInfo::CombinePathAndSpeedProfile方法完成的
	在EM Planner中，路径和速度是分开优化的，这里是在将这两个优化的结果（记录在ReferenceLineInfo的path_data_和speed_data_中）合并成最终的结果


	modules/planning/common/reference_line_info.cc
	bool ReferenceLineInfo::CombinePathAndSpeedProfile( const double relative_time, const double start_s, DiscretizedTrajectory* ptr_discretized_trajectory) {
	  CHECK(ptr_discretized_trajectory != nullptr);
	  // use varied resolution to reduce data load but also provide enough data
	  // point for control module
	  const double kDenseTimeResoltuion = FLAGS_trajectory_time_min_interval;
	  const double kSparseTimeResolution = FLAGS_trajectory_time_max_interval;
	  const double kDenseTimeSec = FLAGS_trajectory_time_high_density_period;

	  if (path_data_.discretized_path().empty()) {
	    AERROR << "path data is empty";
	    return false;
	  }

	  if (speed_data_.empty()) {
	    AERROR << "speed profile is empty";
	    return false;
	  }

	  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.TotalTime();	//根据speed_data_中记录的总时间和间隔以确定需要在轨迹中添加多少个点
	       cur_rel_time += (cur_rel_time < kDenseTimeSec ? kDenseTimeResoltuion : kSparseTimeResolution)) {
	    common::SpeedPoint speed_point;
	    if (!speed_data_.EvaluateByTime(cur_rel_time, &speed_point)) {
	      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
	      return false;
	    }

	    if (speed_point.s() > path_data_.discretized_path().Length()) {	//根据path_data_中的数据获取相应的点以确定轨迹点的位置
	      break;
	    }
	    common::PathPoint path_point =  path_data_.GetPathPointWithPathS(speed_point.s());
	    path_point.set_s(path_point.s() + start_s);

	    common::TrajectoryPoint trajectory_point;							//创建一个轨迹点TrajectoryPoint并设置位置和速度，加速度和相对时间等信息
	    trajectory_point.mutable_path_point()->CopyFrom(path_point);
	    trajectory_point.set_v(speed_point.v());
	    trajectory_point.set_a(speed_point.a());
	    trajectory_point.set_relative_time(speed_point.t() + relative_time);
	    ptr_discretized_trajectory->AppendTrajectoryPoint(trajectory_point);  //在轨迹中添加一个轨迹点
	  }
	  return true;
	}




12. 有了轨迹之后，便可以发出供车辆行驶了

/modules/planning/on_lane_planning.cc

	Status OnLanePlanning::Plan(
	    const double current_time_stamp,
	    const std::vector<TrajectoryPoint>& stitching_trajectory,
	    ADCTrajectory* const ptr_trajectory_pb) {
	  auto* ptr_debug = ptr_trajectory_pb->mutable_debug();
	  if (FLAGS_enable_record_debug) {
	    ptr_debug->mutable_planning_data()->mutable_init_point()->CopyFrom(
	        stitching_trajectory.back());
	    frame_->mutable_open_space_info()->set_debug(ptr_debug);
	    frame_->mutable_open_space_info()->sync_debug_instance();
	  }

	  auto status = planner_->Plan(stitching_trajectory.back(), frame_.get(),ptr_trajectory_pb);		// planner_->Plan是执行每个Planner的主体算法逻辑
	                               

	  ptr_debug->mutable_planning_data()->set_front_clear_distance( EgoInfo::Instance()->front_clear_distance() );

	  if (frame_->open_space_info().is_on_open_space_trajectory()) {
	    const auto& publishable_trajectory =  frame_->open_space_info().publishable_trajectory_data().first;
	    const auto& publishable_trajectory_gear = frame_->open_space_info().publishable_trajectory_data().second;
	    publishable_trajectory.PopulateTrajectoryProtobuf(ptr_trajectory_pb);
	    ptr_trajectory_pb->set_gear(publishable_trajectory_gear);

	    // TODO(QiL): refine engage advice in open space trajectory optimizer.
	    auto* engage_advice = ptr_trajectory_pb->mutable_engage_advice();
	    engage_advice->set_advice(EngageAdvice::KEEP_ENGAGED);
	    engage_advice->set_reason("Keep engage while in parking");

	    // TODO(QiL): refine the export decision in open space info
	    ptr_trajectory_pb->mutable_decision()
	        ->mutable_main_decision()
	        ->mutable_parking()
	        ->set_status(MainParking::IN_PARKING);

	    if (FLAGS_enable_record_debug) {
	      // ptr_debug->MergeFrom(frame_->open_space_info().debug_instance());
	      frame_->mutable_open_space_info()->RecordDebug(ptr_debug);
	      ADEBUG << "Open space debug information added!";
	      // call open space info load debug
	      // TODO(Runxin): create a new flag to enable openspace chart
	      ExportOpenSpaceChart(ptr_trajectory_pb->debug(), *ptr_trajectory_pb,
	                           ptr_debug);
	    }
	  } else {
	    const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();  //在有多个ReferenceLineInfo的情况下，选取最合适的一条。
	    if (!best_ref_info) {
	      std::string msg("planner failed to make a driving plan");
	      AERROR << msg;
	      if (last_publishable_trajectory_) {
	        last_publishable_trajectory_->Clear();
	      }
	      return Status(ErrorCode::PLANNING_ERROR, msg);
	    }
	    // Store current frame stitched path for possible speed fallback in next
	    // frames
	    DiscretizedPath current_frame_planned_path;
	    for (const auto& trajectory_point : stitching_trajectory) {
	      current_frame_planned_path.push_back(trajectory_point.path_point());
	    }
	    const auto& best_ref_path = best_ref_info->path_data().discretized_path();
	    std::copy(best_ref_path.begin() + 1, best_ref_path.end(),
	              std::back_inserter(current_frame_planned_path));
	    frame_->set_current_frame_planned_path(current_frame_planned_path);

	    if (FLAGS_export_chart) {
	      ExportOnLaneChart(best_ref_info->debug(), ptr_debug);
	    } else {
	      ptr_debug->MergeFrom(best_ref_info->debug());
	      ExportReferenceLineDebug(ptr_debug);
	    }
	    ptr_trajectory_pb->mutable_latency_stats()->MergeFrom(
	        best_ref_info->latency_stats());
	    // set right of way status
	    ptr_trajectory_pb->set_right_of_way_status(
	        best_ref_info->GetRightOfWayStatus());
	    for (const auto& id : best_ref_info->TargetLaneId()) {
	      ptr_trajectory_pb->add_lane_id()->CopyFrom(id);
	    }

	    ptr_trajectory_pb->set_trajectory_type(best_ref_info->trajectory_type());

	    if (FLAGS_enable_rss_info) {
	      *ptr_trajectory_pb->mutable_rss_info() = best_ref_info->rss_info();
	    }

	    best_ref_info->ExportDecision(ptr_trajectory_pb->mutable_decision());

	    // Add debug information.
	    if (FLAGS_enable_record_debug) {
	      auto* reference_line = ptr_debug->mutable_planning_data()->add_path();
	      reference_line->set_name("planning_reference_line");
	      const auto& reference_points =
	          best_ref_info->reference_line().reference_points();
	      double s = 0.0;
	      double prev_x = 0.0;
	      double prev_y = 0.0;
	      bool empty_path = true;
	      for (const auto& reference_point : reference_points) {
	        auto* path_point = reference_line->add_path_point();
	        path_point->set_x(reference_point.x());
	        path_point->set_y(reference_point.y());
	        path_point->set_theta(reference_point.heading());
	        path_point->set_kappa(reference_point.kappa());
	        path_point->set_dkappa(reference_point.dkappa());
	        if (empty_path) {
	          path_point->set_s(0.0);
	          empty_path = false;
	        } else {
	          double dx = reference_point.x() - prev_x;
	          double dy = reference_point.y() - prev_y;
	          s += std::hypot(dx, dy);
	          path_point->set_s(s);
	        }
	        prev_x = reference_point.x();
	        prev_y = reference_point.y();
	      }
	    }

	    last_publishable_trajectory_.reset(new PublishableTrajectory(
	        current_time_stamp, best_ref_info->trajectory()));

	    ADEBUG << "current_time_stamp: " << std::to_string(current_time_stamp);

	    last_publishable_trajectory_->PrependTrajectoryPoints(
	        std::vector<TrajectoryPoint>(stitching_trajectory.begin(),
	                                     stitching_trajectory.end() - 1));

	    last_publishable_trajectory_->PopulateTrajectoryProtobuf(ptr_trajectory_pb);

	    best_ref_info->ExportEngageAdvice(
	        ptr_trajectory_pb->mutable_engage_advice());
	  }

	  return status;
	}




	planner_->Plan是执行每个Planner的主体算法逻辑。
在有多个ReferenceLineInfo的情况下，选取最合适的一条。
记录ReferenceLineInfo包含的lane id。
将轨迹的数据格式转换成PublishableTrajectory类型并记录到last_publishable_trajectory_中。
导出成最终格式：ADCTrajectory。Apollo 3.5已经改用新的框架Cyber RT，最终轨迹的发出是在Cyber Component的实现PlanningComponent发出。相关代码如下：
  ADCTrajectory adc_trajectory_pb;
  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);
  auto start_time = adc_trajectory_pb.header().timestamp_sec();
  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

  // modify trajectory relative time due to the timestamp change in header
  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
    p.set_relative_time(p.relative_time() + dt);
  }
  planning_writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));