 Reference:
  https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/em_planner.md



1.


 EM规划器：EMPlanner
EM规划器，也是规划模块Planning的核心，主要的任务是根据：

感知模块Perception和预测模块Prediction得到的障碍物以及未来时间(e.g. 5s内)的运动轨迹
当前路况，禁停区、减速带、停车标志、人行横道等等
来最终规划无人车的最优行驶路径。从参考线提供器ReferenceLineProvider可以看到，当无人车面临变道时，就会存在多条参考线(变道时间可随机扰动，变道早可能进入车道1，变道晚就可能进入车道2)。那么EM规划器就需要计算每条参考线上的最优行驶路径，最后进行综合，得到开销(也就是cost)最小的一条路径。EM规划器对无人车的控制规划进行拆分，分为2类：

路径规划(也就是行驶位置, path-time)
速度规划(speed)
那么对于路径规划和速度规划，可以有两种方法，分别为：

动态规划(dynamic programming)
二次规划(quadratic programming + Spline interpolation，这个和参考线平滑很相似)
上述两种规划方案，Apollo采用的是第一种动态规划法，这种方法的原理很简单，我们这里以速度规划为例，进行简单的流程简述：

step 1：初始化规划起点，可以是车辆当前坐标。然后每隔一段距离(例如10m)规划一次位置，一共向前规划如40m。

step 2：在下个位置侧向进行一些点采样(例如7个)，也就是计算在车道不同宽度位置采样。然后计算每个采样点到前一层所有点的cost，就可以得到其实规划点到当前采样点的最短路径与开销cost。

step 3：前进到最小cost对应的采样点，然后重复2对下个位置横向采样并计算开销cost。最终从终点开始，反向选择一条cost最小的路径。

使用数学的形式来表示，进一步理解动态规划的思想。假设规划起始点为s，需要向前规划M米，那么可以每个d米进行一次采样(也就是纵向采样)，每次采样位置可以横向选择若干个点(横向采样)，表示无人车可以前进到这个车道宽度位置，最终找到一条从起始点到终点的最优行驶路径。

为了更加生动形象的理解，我们以神经网络为类比模型。规划起始点s就是神经网络的输入，只有一个点；规划终止点x其实就是神经网络的输出，但是可以有若干点，这些点在参考线上的长度一致，但车道宽度可以不一致，也就是说可以规划到路中央，也可以规划到路边。纵向每隔d米采样其实就是类似于一个个隐藏层，每个位置横向采样就是该隐藏层的不同神经元。网络中相邻层之间神经元的连接权值就是从前一个点行驶到另一个点所需要的开销cost。那么计算一条最优路径path(s)可以表示为：

$$ mincost(path(s,x^l)) = min_i min_j (mincost(path(a,x_j^{l-1})) + cost(x_j^{l-1}, x_i^{l})) $$

公式中$ mincost(path(s,x^l)) $表示从规划起始点s到规划终点(也就是level层输出的最小开销)，只需要取输出层中具有最小开销的点即可，也就对应$min_i$，其中i从0开始到输出层的数量。起始点到终点i个神经元的最小开销，可以分别计算起始点到倒数第二层每个神经元的最小开销 $mincost(path(a,x_j^{l-1}))$，在加上这个神经元到终点i神经元的开销$cost(x_j^{l-1}, x_i^{l})$，最后取倒数第二层中j个神经元计算最小值，即为整个最优路径的cost，这就是动态规划的原理。神经网络中每个点都保存着起始点到该点的最小开销cost，也就是$mincost(path(a,b)$。

在计算开销cost的过程中，需要考略三个因素：无人车位置(最好就是验证参考线前进，横向不停地调整会增加cost)，静态障碍物以及动态障碍物(这两类已经在Frame中完成设置，已经考虑到感知物体和路况)。

下面我们就对路径规划和速度规划做一个详细的了解。代码中使用了动态规划法对路径进行规划；同时使用动态规划与二次规划+样条线插值对速度进行规划

planner_type : EM
em_planner_config {
    task : DP_POLY_PATH_OPTIMIZER
    task : PATH_DECIDER
    task : DP_ST_SPEED_OPTIMIZER
    task : SPEED_DECIDER
    task : QP_SPLINE_ST_SPEED_OPTIMIZER
}


2. 路径规划器--DpPolyPathOptimizer
	apollo/modules/planning/planner/em/em_planner.cc

	Status EMPlanner::Plan( const TrajectoryPoint& planning_start_point, Frame* frame ) {
	  bool has_drivable_reference_line = false;
	  bool disable_low_priority_path = false;
	  auto status =  Status(ErrorCode::PLANNING_ERROR, "reference line not drivable");

	  for (auto& reference_line_info : frame->reference_line_info()) {    // 先分别对每条参考线进行规划
	    if (disable_low_priority_path) {
	      reference_line_info.SetDrivable(false);
	    }
	    if (!reference_line_info.IsDrivable()) {
	      continue;
	    }
	    auto cur_status =  PlanOnReferenceLine(planning_start_point, frame, &reference_line_info); // 规划主函数

	    if (cur_status.ok() && reference_line_info.IsDrivable()) {
	      has_drivable_reference_line = true;
	      if ( FLAGS_prioritize_change_lane && reference_line_info.IsChangeLanePath() && reference_line_info.Cost() < kStraightForwardLineCost ) {
	        disable_low_priority_path = true;
	      }
	    } else {
	      reference_line_info.SetDrivable(false);
	    }
	  }
	  return has_drivable_reference_line ? Status::OK() : status;
	}




Status EMPlanner::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
	 for (auto& optimizer : tasks_) {			 // 对每条参考线分别进行所有任务的规划，包括速度规划、路径位置规划、位置决策器规划等。
    const double start_timestamp = Clock::NowInSeconds();
    ret = optimizer->Execute(frame, reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << optimizer->Name()
             << "], Error message: " << ret.error_message();
      break;
    }


}	    



 apollo/planning/tasks/dp_poly_path/dp_poly_path_optimizer.cc
   apollo::common::Status PathOptimizer::Execute(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret = Process(
      reference_line_info->speed_data(), reference_line_info->reference_line(),   // 最终会调用路径规划类的Process函数
      frame->PlanningStartPoint(), reference_line_info->mutable_path_data());
  RecordDebugInfo(reference_line_info->path_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  return ret;
}



apollo/planning/tasks/dp_poly_path/dp_poly_path_optimizer.cc

Status DpPolyPathOptimizer::Process(const SpeedData &speed_data,
                                    const ReferenceLine &,
                                    const common::TrajectoryPoint &init_point,
                                    PathData *const path_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process().";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }
  CHECK_NOTNULL(path_data);
  DPRoadGraph dp_road_graph(config_, *reference_line_info_, speed_data);
  dp_road_graph.SetDebugLogger(reference_line_info_->mutable_debug());

  if (!dp_road_graph.FindPathTunnel(   //// 寻找参考线最优的行驶路径，由DPRoadGraph::FindPathTunnel完成。
          init_point,
          reference_line_info_->path_decision()->path_obstacles().Items(),
          path_data)) {
    AERROR << "Failed to find tunnel in road graph";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph path generation");
  }

  return Status::OK();
}


从上面的代码结构，可以进一步验证开始我们的想法，规划的方法是先独立对所有参考线进行单独规划，每条参考线可以得到一个最优的行进路线；最后综合所有参考线的最优行进路线，选择"全局"最优的路线



下面就对DPRoadGraph::FindPathTunnel 函数进行解析，大体的逻辑也很简单，主要分3步：

Step A. 计算起始点的累计距离s，侧方相对偏移l，侧向速度dl和侧向速度ddl

Step B. 获取当前参考线下最优的前进路线

Step C. 将最优前进路线封装成path_data


apollo/modules/planning/tasks/dp_poly_path/dp_road_graph.cc

	bool DPRoadGraph::FindPathTunnel( const common::TrajectoryPoint &init_point, const std::vector<const PathObstacle *> &obstacles, PathData *const path_data) {
	  CHECK_NOTNULL(path_data);
	  init_point_ = init_point;

	  // Step A. 计算起始点的累计距离s，侧方相对偏移l，侧向速度dl和侧向速度ddl
	  if ( !reference_line_.XYToSL( {init_point_.path_point().x(), init_point_.path_point().y()}, &init_sl_point_)) {
	    AERROR << "Fail to create init_sl_point from : " << init_point.DebugString();
	    return false;
	  }

	  if (!CalculateFrenetPoint(init_point_, &init_frenet_frame_point_)) {
	    AERROR << "Fail to create init_frenet_frame_point_ from : " << init_point_.DebugString();
	    return false;
	  }

	// Step B. 获取当前参考线下最优的前进路线
	  std::vector<DPRoadGraphNode> min_cost_path;
	  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {
	    AERROR << "Fail to generate graph!";
	    return false;
	  }

	  // Step C. 将最优前进路线封装成path_data
	  std::vector<common::FrenetFramePoint> frenet_path;
	  float accumulated_s = init_sl_point_.s();
	  const float path_resolution = config_.path_resolution();

	  for (std::size_t i = 1; i < min_cost_path.size(); ++i) {
	    const auto &prev_node = min_cost_path[i - 1];
	    const auto &cur_node = min_cost_path[i];

	    const float path_length = cur_node.sl_point.s() - prev_node.sl_point.s();
	    float current_s = 0.0;
	    const auto &curve = cur_node.min_cost_curve;
	    while (current_s + path_resolution / 2.0 < path_length) {
	      const float l = curve.Evaluate(0, current_s);
	      const float dl = curve.Evaluate(1, current_s);
	      const float ddl = curve.Evaluate(2, current_s);
	      common::FrenetFramePoint frenet_frame_point;
	      frenet_frame_point.set_s(accumulated_s + current_s);
	      frenet_frame_point.set_l(l);
	      frenet_frame_point.set_dl(dl);
	      frenet_frame_point.set_ddl(ddl);
	      frenet_path.push_back(std::move(frenet_frame_point));
	      current_s += path_resolution;
	    }
	    if (i == min_cost_path.size() - 1) {
	      accumulated_s += current_s;
	    } else {
	      accumulated_s += path_length;
	    }
	  }
	  FrenetFramePath tunnel(frenet_path);
	  path_data->SetReferenceLine(&reference_line_);
	  path_data->SetFrenetPath(tunnel);
	  return true;
	}


	步骤A和C没什么难点，B步骤稍微有点繁琐，下面我们将逐步进行分析。在计算当前参考线下最优的前进路线时，正如开始所说的，可以每隔一段距离，就在车道横向采样一些点，表示无人车在车道不同宽度行驶时坐标，然后计算两个距离之间cost最小的坐标对，
	这就代表当前位置到下一位置的最优路径。这样就需要一步步解决以下问题：


	// Q1：如何采样坐标点？--SamplePathWaypoints函数
	这个问题在开始已经说过，规划首先就需要考虑：

规划未来多久的距离？
答案是未来近40m的距离，可以通过观察代码:

apollo/modules/planning/tasks/dp_poly_path/dp_road_graph.cc

  const float kMinSampleDistance = 40.0;
  const float total_length = std::fmin( init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance), reference_line_.Length()); //具体还需要考虑规划起始点的无人车速度以及参考线的长度。

  // Q2 每隔多久进行一次采样(纵向采样)？每次采样车道横向不同位置采样几个点(横向采样)？
答案也比较简单，大约每个10-20m，对车道不同宽度进行采样。

const size_t num_sample_per_level =  FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level()  : config_.sample_points_num_each_level(); ///// navigator_sample_num_each_level: 3  sample_points_num_each_level: 7

  const bool has_sidepass = HasSidepass();

  constexpr float kSamplePointLookForwardTime = 4.0;
   // 将step_length裁剪到[20,40]米之间
  const float step_length = common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,  config_.step_length_min(), config_.step_length_max());

// 每隔level_distance进行一次横向采样，纵向采样的间隔差不多10-20m
  const float level_distance = (init_point.v() > FLAGS_max_stop_speed) ? step_length : step_length / 2.0;
  float accumulated_s = init_sl_point_.s();
  float prev_s = accumulated_s;

  //上面代码有一个词--level，其实就是纵向采样点，每个level中会横向采样若干点。当然，可以更紧凑的纵向与横向采样，但是在计算cost会增加过大的计算量，所以需要经过权衡


  // Q3 纵向&&横向怎么样进行采样？

  首先如果无人车当前状态正在寻找停车点，也就是PULL_OVER状态，那么采样点其实只要设置为PULL_OVER的计算得到的起始点即可，当前前提条件是需要进入到他的可操作区域

  if ( status->planning_state().has_pull_over() && status->planning_state().pull_over().in_pull_over() ) {
    status->mutable_planning_state()->mutable_pull_over()->set_status( PullOverStatus::IN_OPERATION );
    const auto &start_point = status->planning_state().pull_over().start_point();
    SLPoint start_point_sl;
    if (!reference_line_.XYToSL(start_point, &start_point_sl)) {
      AERROR << "Fail to change xy to sl.";
      return false;
    }
       if (init_sl_point_.s() > start_point_sl.s()) {  // 表示无人车已进入PULL_OVER可操作区域
      const auto &stop_point =
          status->planning_state().pull_over().stop_point();
      SLPoint stop_point_sl;
      if (!reference_line_.XYToSL(stop_point, &stop_point_sl)) {
        AERROR << "Fail to change xy to sl.";
        return false;
      }
      std::vector<common::SLPoint> level_points(1, stop_point_sl);  // 这时候只要设置停车点为下一时刻(纵向)的采样点即可，那么横向采样点就只有一个，就是停车位置
      points->emplace_back(level_points);
      return true;
    }

    然后循环进行纵向+横向采样


     for (std::size_t i = 0; accumulated_s < total_length; ++i) {
		    accumulated_s += level_distance;
		    if (accumulated_s + level_distance / 2.0 > total_length) {
		      accumulated_s = total_length;
		    }
		    const float s = std::fmin(accumulated_s, total_length);
		    constexpr float kMinAllowedSampleStep = 1.0;
		    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
		      continue;
		    }
		    prev_s = s;

		    // 计算纵向每个位置，有效的左右边界，kBoundaryBuff是无人车与车道线边界线的间距。需要保持无人车在车道内行驶。
		    double left_width = 0.0;
		    double right_width = 0.0;
		    reference_line_.GetLaneWidth(s, &left_width, &right_width);

		    constexpr float kBoundaryBuff = 0.20;
		    const float eff_right_width = right_width - half_adc_width - kBoundaryBuff;   // 计算右有效宽度
		    const float eff_left_width = left_width - half_adc_width - kBoundaryBuff;		// 计算左有效宽度

		    // the heuristic shift of L for lane change scenarios
		    const double delta_dl = 1.2 / 20.0;
		    const double kChangeLaneDeltaL = common::math::Clamp(
		        level_distance * (std::fabs(init_frenet_frame_point_.dl()) + delta_dl),
		        1.2, 3.5);

		    float kDefaultUnitL = kChangeLaneDeltaL / (num_sample_per_level - 1);   // 每个位置上横向采样时，采样点之间的距离，差不多0.2m

		    if ( reference_line_info_.IsChangeLanePath() &&  !reference_line_info_.IsSafeToChangeLane() ) {    // 如果当前参考线是变道，且变道不安全(无人车前后一定距离内有障碍物),，那么增加采样点间隔，这样可以下一时刻减少变道的时间。
		      kDefaultUnitL = 1.0; 
		    }

		    const float sample_l_range = kDefaultUnitL * (num_sample_per_level - 1); // 横向采样的宽度
		    float sample_right_boundary = -eff_right_width;	// 横向采样的宽度
		    float sample_left_boundary = eff_left_width;		 // 计算FLU坐标系下，左边界

		    const float kLargeDeviationL = 1.75;

		    if (reference_line_info_.IsChangeLanePath() ||  std::fabs(init_sl_point_.l()) > kLargeDeviationL) {  // 修正左右边界，如果参考线需要变道，并且已经便宜车道了，那么就将横向采样区间向变道方向平移。
		      sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());	// 左变道，修正左边界
		      sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());		// 右变道，修正右边界

		      if (init_sl_point_.l() > eff_left_width) { // 左变道，修正右边界，向左平移，因为左变道车道线右边的区域不用考虑
		        sample_right_boundary = std::fmax( sample_right_boundary, init_sl_point_.l() - sample_l_range);
		      }
		      if (init_sl_point_.l() < eff_right_width) {  // 右变道，修正左边界，向右平移，因为右变道车道线左边的区域不用考虑
		        sample_left_boundary = std::fmin(sample_left_boundary,  init_sl_point_.l() + sample_l_range);
		      }
		    }

		    	 // 具体采样
		    //最后进行每个纵向位置的横向区间采样，采样分两个多类情况
		    std::vector<float> sample_l;

		    if ( reference_line_info_.IsChangeLanePath() && !reference_line_info_.IsSafeToChangeLane() ) { //情况1：如果当前参考线需要变道，并且变道不安全，那么横向采样点就设置为第二条参考线的位置，直接走第二条参考线。
		      sample_l.push_back(reference_line_info_.OffsetToOtherReferenceLine());
		    } else if (has_sidepass) {
		      // currently only left nudge is supported. Need road hard boundary for both sides
		    	// 情况2：如果当前参考线需要侧方绕行(SIDEPASS)，即从旁边超车。若是可以进行左边超车，那么横向采样点设置为左边界+超车距离；右边超车，横向采样点设置为右边界+超车距离
		      switch (sidepass_.type()) {
		        case ObjectSidePass::LEFT: {
		          sample_l.push_back(eff_left_width + config_.sidepass_distance());
		          break;
		        }
		        case ObjectSidePass::RIGHT: {
		          sample_l.push_back(-eff_right_width - config_.sidepass_distance());
		          break;
		        }
		        default:
		          break;
		      }
		    } else {
		      common::util::uniform_slice( sample_right_boundary, sample_left_boundary, num_sample_per_level - 1, &sample_l ); // 情况3：正常行驶情况下，从横向区间[sample_right_boundary , sample_left_boundary]大小为sample_l_range进行均匀采样
		    }

		    //计算每个横向采样点的相对偏移距离l和累计距离s，封装成一个level，最后所有level封装成way_points
		    std::vector<common::SLPoint> level_points;
		    planning_internal::SampleLayerDebug sample_layer_debug;
		    for (size_t j = 0; j < sample_l.size(); ++j) {		// 纵向采样
		      common::SLPoint sl = common::util::MakeSLPoint(s, sample_l[j]);
		      sample_layer_debug.add_sl_point()->CopyFrom(sl);
		      level_points.push_back(std::move(sl));
		    }
		    if (!reference_line_info_.IsChangeLanePath() && has_sidepass) {  // 对于不变道但是要超车的情况，额外增加一个车道线中心采样点
		      auto sl_zero = common::util::MakeSLPoint(s, 0.0);
		      sample_layer_debug.add_sl_point()->CopyFrom(sl_zero);
		      level_points.push_back(std::move(sl_zero));
		    }

		    if (!level_points.empty()) {  //  // level不为空，封装进最终的way+points
		      planning_debug_->mutable_planning_data() ->mutable_dp_poly_graph() ->add_sample_layer() ->CopyFrom(sample_layer_debug);
		      points->emplace_back(level_points);
		    }
		  }
		  return true;
		}



		// Q4  如何计算两个level之间两两横向采样点之间的开销？--UpdateNode函数和TrajectoryCost类

		采样其实没有把规划起始点加进去，代码中做了一个修正
		/apollo/modules/planning/tasks/dp_poly_path/dp_road_graph.cc

		 path_waypoints.insert( path_waypoints.begin(), std::vector<common::SLPoint>{init_sl_point_} );

障碍物处理
在规划中，需要考虑到障碍物的轨迹信息，也就是需要考虑未来每个时刻，障碍物出现的位置，只要将障碍物每个时间点出现的位置，作为开销计算项即可。

  TrajectoryCost trajectory_cost( config_, reference_line_, reference_line_info_.IsChangeLanePath(),  obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_);

  未来每个时间点计算方法也很简单：

  apollo/modules/planning/tasks/dp_poly_path/trajectory_cost.cc

  const float total_time = std::min(heuristic_speed_data_.TotalTime(), FLAGS_prediction_total_time);	//FLAGS_prediction_total_time其实就是PRediction模块中障碍物轨迹预测总时间，5s
  num_of_time_stamps_ = static_cast<uint32_t>( std::floor(total_time / config.eval_time_interval()));	// val_time_interval其实就是Prediction模块中障碍物两个预测轨迹点的时间差，0.1s。所以一共差不多50个位置点
  																										// 最后对参考线上的每个障碍物在每个时间点设定位置标定框。


  // 对于每个障碍物，进行如下条件判断
    for (const auto *ptr_path_obstacle : obstacles) {
    	// 情况1：如果是无人车可忽略的障碍物或者迫使无人车停车的障碍物，就不需要考虑。因为前者对无人车前进无影响；后者情况无人车智能停车，根本不需要前进了
	    if (ptr_path_obstacle->IsIgnore()) {
	      continue;
	    } else if (ptr_path_obstacle->LongitudinalDecision().has_stop()) {
	      continue;
	    }
	    const auto &sl_boundary = ptr_path_obstacle->PerceptionSLBoundary();

	    const float adc_left_l =  init_sl_point_.l() + vehicle_param_.left_edge_to_center();
	    const float adc_right_l = init_sl_point_.l() - vehicle_param_.right_edge_to_center();

	    if (adc_left_l + FLAGS_lateral_ignore_buffer < sl_boundary.start_l() ||
	        adc_right_l - FLAGS_lateral_ignore_buffer > sl_boundary.end_l()) {
	      continue;
	    }

	    const auto *ptr_obstacle = ptr_path_obstacle->obstacle();
	    bool is_bycycle_or_pedestrian =
	        (ptr_obstacle->Perception().type() ==
	             perception::PerceptionObstacle::BICYCLE ||
	         ptr_obstacle->Perception().type() ==
	             perception::PerceptionObstacle::PEDESTRIAN);

	        // 情况3：如果障碍物是虚拟障碍物，那么无人车可以毫无影响的前进，所以该类障碍物也可以忽略。
	    if (Obstacle::IsVirtualObstacle(ptr_obstacle->Perception())) {
	      // Virtual obstacle
	      continue;
	    } else if (Obstacle::IsStaticObstacle(ptr_obstacle->Perception()) ||  is_bycycle_or_pedestrian) {	// 情况4：如果障碍物是静止的，那么将其加入静止障碍物队列
	      static_obstacle_sl_boundaries_.push_back(std::move(sl_boundary));
	    } else {
	      std::vector<Box2d> box_by_time;
	      for (uint32_t t = 0; t <= num_of_time_stamps_; ++t) {  // 计算每个时间点的位置，转换成标定框加入动态障碍物队列
	        TrajectoryPoint trajectory_point =
	            ptr_obstacle->GetPointAtTime(t * config.eval_time_interval());

	        Box2d obstacle_box = ptr_obstacle->GetBoundingBox(trajectory_point);
	        constexpr float kBuff = 0.5;
	        Box2d expanded_obstacle_box =
	            Box2d(obstacle_box.center(), obstacle_box.heading(),
	                  obstacle_box.length() + kBuff, obstacle_box.width() + kBuff);
	        box_by_time.push_back(expanded_obstacle_box);
	      }
	      dynamic_obstacle_boxes_.push_back(std::move(box_by_time));  // 情况5：如果障碍物时运动的，那么就需要计算障碍物在该时间点的位置，并将其加入动态障碍物队列
	    }
	  }
	}



	// Q5	level之间两两计算cost

	apollo/modules/planning/tasks/dp_poly_path/dp_road_graph.cc

	bool DPRoadGraph::GenerateMinCostPath( const std::vector<const PathObstacle *> &obstacles, std::vector<DPRoadGraphNode> *min_cost_path) {
		  CHECK(min_cost_path != nullptr);

		  std::vector<std::vector<common::SLPoint>> path_waypoints;

		  if (!SamplePathWaypoints(init_point_, &path_waypoints) || path_waypoints.size() < 1) {
		    AERROR << "Fail to sample path waypoints! reference_line_length = " << reference_line_.Length();
		    return false;
		  }
		  path_waypoints.insert(path_waypoints.begin(),std::vector<common::SLPoint>{init_sl_point_});
		  const auto &vehicle_config = common::VehicleConfigHelper::instance()->GetConfig();

		  TrajectoryCost trajectory_cost( config_, reference_line_, reference_line_info_.IsChangeLanePath(), obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_);

		  std::list<std::list<DPRoadGraphNode>> graph_nodes; // 这是最后的前向遍历图，类似于神经网络结构，N个level，每个level若干横向采样点，两层level之间的采样点互相连接。
		  graph_nodes.emplace_back();		//    而且网络中的每个node，都保存了，从规划起始点到该节点的最小cost，以及反向连接链(cost最小对应的parent)
		  graph_nodes.back().emplace_back(init_sl_point_, nullptr, ComparableCost());   // 输入层(规划起始点)加入网络图
		  auto &front = graph_nodes.front().front();		// 规划起始点：init_point
		  size_t total_level = path_waypoints.size();		 // 网络层数，level数量

		  for (std::size_t level = 1; level < path_waypoints.size(); ++level) {  // 网络两两level之间计算连接cost
		    const auto &prev_dp_nodes = graph_nodes.back();		 // 前一层level
		    const auto &level_points = path_waypoints[level];  // 当前层level中的所有横向采样点(类似神经元)

		    graph_nodes.emplace_back();

		    for (size_t i = 0; i < level_points.size(); ++i) {  // 计算当前层level中与前一层所有计算的连接权值，也就是cost
		      const auto &cur_point = level_points[i];

		      graph_nodes.back().emplace_back(cur_point, nullptr);
		      auto &cur_node = graph_nodes.back().back();
		      if (FLAGS_enable_multi_thread_in_dp_poly_path) {
		        PlanningThreadPool::instance()->Push(std::bind(
		            &DPRoadGraph::UpdateNode, this, std::ref(prev_dp_nodes), level,
		            total_level, &trajectory_cost, &(front), &(cur_node)));

		      } else {  // 计算前一层prev_dp_nodes和当前层的节点cur_node的开销cost，取prev_dp_nodes中与cur_node开销cost最小的节点，设置为最优路径
		        UpdateNode(prev_dp_nodes, level, total_level, &trajectory_cost, &front, &cur_node);
		      }
		    }
		    if (FLAGS_enable_multi_thread_in_dp_poly_path) {
		      PlanningThreadPool::instance()->Synchronize();
		    }
		  }

		  // find best path
		  DPRoadGraphNode fake_head;
		  for (const auto &cur_dp_node : graph_nodes.back()) {
		    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
		                         cur_dp_node.min_cost);
		  }

		  const auto *min_cost_node = &fake_head;
		  while (min_cost_node->min_cost_prev_node) {
		    min_cost_node = min_cost_node->min_cost_prev_node;
		    min_cost_path->push_back(*min_cost_node);
		  }
		  if (min_cost_node != &graph_nodes.front().front()) {
		    return false;
		  }

		  std::reverse(min_cost_path->begin(), min_cost_path->end());

		  for (const auto &node : *min_cost_path) {
		    ADEBUG << "min_cost_path: " << node.sl_point.ShortDebugString();
		    planning_debug_->mutable_planning_data()
		        ->mutable_dp_poly_graph()
		        ->add_min_cost_point()
		        ->CopyFrom(node.sl_point);
		  }
		  return true;
		}




		经过注释，可以很容易的理解整个网络神经元(节点)之间连接cost的流程，那么Update函数中，如何计算前一层和当前层的某个节点之间的开销。其实比较简单，举个例子。现有：

a. 前一层prev_dp_nodes中的某个节点prev_dp_node，它有4个属性：累计距离s0，横向偏移距离l0，横向速度dl0，横向加速度ddl0。

b. 当前层某个节点cur_node，同样4个属性：累计距离s1，横向偏移距离l1，横向速度dl1，横向加速度ddl1。

两个节点之间累积距离就是上述提到level_distance，大约10-20m。那么接下来的工作就是个参考线平滑一样，使用六次多项式进行两个点之间的多项式拟合，y=f(s)，自变量是累计距离差s，因变量是横向便宜距离l。经过定义两个点可以数值化为：

prev_dp_node:

l0 = f(0) = prev_dp_node.sl_point.l()

dl0 = f'(0)

ddl0 = f''(0)

cur_node:

delta_s = cur_node.cur_point.s() - prev_dp_node.prev_sl_point.s()

l1 = f(delta_s) = cur_node.cur_point.l()

dl1 = f'(delta_s) = 0

ddl1 = f''(delta_s) = 0

匹配过程与Prediction障碍物轨迹预测

那么cur_node与prev_dp_node相连得到的cost为：

const auto cost = trajectory_cost->Calculate(curve, prev_sl_point.s(), cur_point.s(),level, total_level) +  prev_dp_node.min_cost;
cur_node->UpdateCost(&prev_dp_node, curve, cost);
其中UpdateCost是刷新最小cost，这样可以得到与cur_node相连最小的cost，以及对应的prev_dp_node。




// Q6  两两node之间的cost如何计算？包含多少项？

两两level中不同node之间cost计算分为三个部分：路径开销PathCost，静态障碍物开销StaticObstacleCost和动态障碍物开销DynamicObstacleCost
apollo/modules/planning/tasks/dp_poly_path/trajectory_cost.cc


// TODO(All): optimize obstacle cost calculation time
ComparableCost TrajectoryCost::Calculate( const QuinticPolynomialCurve1d &curve,const float start_s, const float end_s,const uint32_t curr_level,const uint32_t total_level ) {
  ComparableCost total_cost;
  // path cost
  total_cost +=  CalculatePathCost(curve, start_s, end_s, curr_level, total_level);
  // static obstacle cost
  total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);
  // dynamic obstacle cost
  total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
  return total_cost;
}


路径开销PathCost:

已经拟合出了一条前一个node到后一个node的5次多项式，这就是前进的路线中侧方距离l的轨迹。路径开销需要对这条轨迹做评估，具体的评估方法也比较简单。
我们已经可以的知道两个node之间的前进距离level_distance大约为10-20米，也就是函数的区间长度为10-20米，我们只需要对区间进行采样，每1米采样一个点，就可以得到约10-20个路径点的侧方距离l，侧方速度dl以及侧方加速度ddl数据。
再对这若干组数据评估cost相加即为这条path的开销。

ComparableCost TrajectoryCost::CalculatePathCost(
    const QuinticPolynomialCurve1d &curve, const float start_s,
    const float end_s, const uint32_t curr_level, const uint32_t total_level) {
  ComparableCost cost;
  float path_cost = 0.0;
  std::function<float(const float)> quasi_softmax = [this](const float x) {
    const float l0 = this->config_.path_l_cost_param_l0();
    const float b = this->config_.path_l_cost_param_b();
    const float k = this->config_.path_l_cost_param_k();
    return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
  };

  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  const float width = vehicle_config.vehicle_param().width();

  for ( float curve_s = 0.0; curve_s < (end_s - start_s); curve_s += config_.path_resolution() ) { //// path_resolution: 1.0m，每1m采样一个点
    const float l = curve.Evaluate(0, curve_s);		// 根据拟合的5次多项式，计算该点的侧方距离

    path_cost += l * l * config_.path_l_cost() * quasi_softmax(std::fabs(l));   // A. 侧方距离l开销，path_l_cost：6.5

    double left_width = 0.0;
    double right_width = 0.0;
    reference_line_->GetLaneWidth(curve_s + start_s, &left_width, &right_width);

    constexpr float kBuff = 0.2;
    if (!is_change_lane_path_ && (l + width / 2.0 + kBuff > left_width ||  l - width / 2.0 - kBuff < -right_width)) {	 // 确认是否已经偏离参考线 
      cost.cost_items[ComparableCost::OUT_OF_BOUNDARY] = true;
    }

    const float dl = std::fabs(curve.Evaluate(1, curve_s));
    path_cost += dl * dl * config_.path_dl_cost();		// B. 侧方速度开销，path_dl_cost：8000

    const float ddl = std::fabs(curve.Evaluate(2, curve_s));
    path_cost += ddl * ddl * config_.path_ddl_cost();	// C. 侧方加速度开销，path_ddl_cost：5
  }
  path_cost *= config_.path_resolution();

  if (curr_level == total_level) {
    const float end_l = curve.Evaluate(0, end_s - start_s);
    path_cost +=
        std::sqrt(end_l - init_sl_point_.l() / 2.0) * config_.path_end_l_cost();   // path_end_l_cost：10000
  }
  cost.smoothness_cost = path_cost;
  return cost;
}


结论1：无论是l，dl和ddl哪个开销，数值越大，cost也就越大。这也表明，Apollo其实鼓励无人车沿着参考线前进，因为沿着参考线前进，l，dl和ddl都是为0的。

结论2：从开销系数来看，Apollo对于侧方距离l和侧方加速度ddl相对来说比较宽松，对侧方速度限制极为严格。但是需要注意一点，但从系数其实无法正确的得到这个结论，因为还要看三者对应的数值范围，如果数值范围小，同时参数小，那么这一项很容易就被忽略了，为了解决这个问题，可以提升其系数，增加权重。

结论3：代码额外增加了起始点和规划终点的侧方偏移，保证不必要的偏移，正常不变道情况下最好是沿着同一车道前进，不变道。


// 静态障碍物开销StaticObstacleCost
静态障碍物开销其实思路和路径开销一样，在这条多项式曲线上采样，计算每个采样点和所有障碍物的标定框是否重叠，重叠cost就比较大

ComparableCost TrajectoryCost::CalculateStaticObstacleCost(  const QuinticPolynomialCurve1d &curve, const float start_s, const float end_s) {
  ComparableCost obstacle_cost;
  for (float curr_s = start_s; curr_s <= end_s; curr_s += config_.path_resolution()) {  // 拟合曲线采样
    const float curr_l = curve.Evaluate(0, curr_s - start_s);
    for (const auto &obs_sl_boundary : static_obstacle_sl_boundaries_) {
      obstacle_cost += GetCostFromObsSL(curr_s, curr_l, obs_sl_boundary);
    }
  }
  obstacle_cost.safety_cost *= config_.path_resolution();
  return obstacle_cost;
}


// 情况1：障碍物和无人车侧方向上的距离大于一个阈值(lateral_ignore_buffer，默认3m)，那么cost就是0，忽略障碍物

情况2：如果障碍物在无人车后方，cost为0，可忽略。

否则就计算cost

ComparableCost TrajectoryCost::GetCostFromObsSL(
    const float adc_s, const float adc_l, const SLBoundary &obs_sl_boundary) {
  const auto &vehicle_param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();

  ComparableCost obstacle_cost;

  const float adc_front_s = adc_s + vehicle_param.front_edge_to_center();
  const float adc_end_s = adc_s - vehicle_param.back_edge_to_center();
  const float adc_left_l = adc_l + vehicle_param.left_edge_to_center();
  const float adc_right_l = adc_l - vehicle_param.right_edge_to_center();

  if (adc_left_l + FLAGS_lateral_ignore_buffer < obs_sl_boundary.start_l() ||
      adc_right_l - FLAGS_lateral_ignore_buffer > obs_sl_boundary.end_l()) {
    return obstacle_cost;
  }

  bool no_overlap = ((adc_front_s < obs_sl_boundary.start_s() ||
                      adc_end_s > obs_sl_boundary.end_s()) ||  // longitudinal
                     (adc_left_l + FLAGS_static_decision_nudge_l_buffer <
                          obs_sl_boundary.start_l() ||
                      adc_right_l - FLAGS_static_decision_nudge_l_buffer >
                          obs_sl_boundary.end_l()));  // lateral

  if (!no_overlap) {
    obstacle_cost.cost_items[ComparableCost::HAS_COLLISION] = true;
  }

  // if obstacle is behind ADC, ignore its cost contribution.
  if (adc_front_s > obs_sl_boundary.end_s()) {
    return obstacle_cost;
  }

  const float delta_l = std::fabs(
      adc_l - (obs_sl_boundary.start_l() + obs_sl_boundary.end_l()) / 2.0);  // 静态障碍物中心和无人车中心侧方向上的距离

  const double kSafeDistance = 1.0;
  if (delta_l < kSafeDistance) {		
    obstacle_cost.safety_cost +=
        config_.obstacle_collision_cost() *
        Sigmoid(config_.obstacle_collision_distance() - delta_l);  // A. 侧方向距离造成的cost系数，obstacle_collision_cost：1e8,  obstacle_collision_distance：0.5
  }

  const float delta_s = std::fabs(
      adc_s - (obs_sl_boundary.start_s() + obs_sl_boundary.end_s()) / 2.0);   // 静态障碍物中心和无人车中心前方向上的距离
  obstacle_cost.safety_cost +=
      config_.obstacle_collision_cost() *
      Sigmoid(config_.obstacle_collision_distance() - delta_s);   // B. 前方向距离造成的cost  obstacle_collision_cost: 1e8
  return obstacle_cost;
}


那么经过分析就可以得到两个结论：

结论1：静态障碍物cost计算就分为侧方向cost和前方向cost，计算方法是一样的

结论2：从两个方向计算的方法来看，我们可以看到Sigmoid(·)这个函数是单调递减的，在0.5(obstacle_collision_distance)时取0.5，越大取值越小。所以Apollo鼓励无人车在侧方向和前方向上与障碍物保持一定距离，如0.5m以上



//动态障碍物开销DynamicObstacleCost

ComparableCost TrajectoryCost::CalculateDynamicObstacleCost(
    const QuinticPolynomialCurve1d &curve, const float start_s,
    const float end_s) const {
  ComparableCost obstacle_cost;
  float time_stamp = 0.0;

  for (size_t index = 0; index < num_of_time_stamps_; ++index, time_stamp += config_.eval_time_interval()) {  // 障碍物每隔eval_time_interval(默认0.1s)会得到一个运动坐标，分别计算对所有时间点坐标的cost
    common::SpeedPoint speed_point;
    heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);
    float ref_s = speed_point.s() + init_sl_point_.s();
    if (ref_s < start_s) {
      continue;
    }
    if (ref_s > end_s) {
      break;
    }

    const float s = ref_s - start_s;  // s on spline curve
    const float l = curve.Evaluate(0, s);
    const float dl = curve.Evaluate(1, s);

    const common::SLPoint sl = common::util::MakeSLPoint(ref_s, l);
    const Box2d ego_box = GetBoxFromSLPoint(sl, dl);
    for (const auto &obstacle_trajectory : dynamic_obstacle_boxes_) {  // 障碍物每隔eval_time_interval(默认0.1s)会得到一个运动坐标，分别计算对所有时间点坐标的cost
      obstacle_cost +=
          GetCostBetweenObsBoxes(ego_box, obstacle_trajectory.at(index));
    }
  }
  constexpr float kDynamicObsWeight = 1e-6;
  obstacle_cost.safety_cost *=
      (config_.eval_time_interval() * kDynamicObsWeight);
  return obstacle_cost;
}


区别静态障碍物，动态障碍物会在这个时间段内运动，在前面我们已经对障碍物进行时间段运动采样，得到了障碍物每隔0.1s的坐标位置，那么只要需要计算每个采样点和这些运动的坐标位置cost，求和就可以每个动态障碍物的cost。


ComparableCost TrajectoryCost::GetCostBetweenObsBoxes(
    const Box2d &ego_box, const Box2d &obstacle_box) const {
  ComparableCost obstacle_cost;

  const float distance = obstacle_box.DistanceTo(ego_box);
  if (distance > config_.obstacle_ignore_distance()) {
    return obstacle_cost;
  }

  obstacle_cost.safety_cost +=
      config_.obstacle_collision_cost() *
      Sigmoid(config_.obstacle_collision_distance() - distance);  // A. 计算碰撞cost   obstacle_collision_cost: 1e8  // obstacle_collision_distance: 0.5
  obstacle_cost.safety_cost +=
      20.0 * Sigmoid(config_.obstacle_risk_distance() - distance); // B. 计算风险cost  obstacle_risk_distance: 2.0
  return obstacle_cost; 
}


从代码可以得到如下结论：

结论1：动态障碍物计算需要考虑到每个时间点上障碍物的位置，每个时间点每个障碍物的cost计算分为碰撞cost和风险cost。

结论2：从计算公式可以看到，一样是Sigmoid，碰撞cost系数高达1e8，风险cost系数为20，所以Apollo对障碍物碰撞惩罚是非常高的，同样对存在风险的位置也有较高的惩罚。碰撞要求障碍物和无人车在约0.5m以内；风险则定义在2m以内。




// Q7 如何生成一条最小cost的路径？
当计算完每个node的最优路径(起始规划点到改点具有最小的cost)，那么就可以计算从起始规划点到终点的最优路径。首先终点不止一个，也就是所有终点采样点的累计距离s是一致的，但是侧方相对距离l是在一个区间内的，所以：



bool DPRoadGraph::GenerateMinCostPath(
	    const std::vector<const PathObstacle *> &obstacles,
    std::vector<DPRoadGraphNode> *min_cost_path) {


// 
step 1：选择最后一个level中，具有最小cost的node最为规划终点
  // find best path
  DPRoadGraphNode fake_head;
  for (const auto &cur_dp_node : graph_nodes.back()) {
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,     cur_dp_node.min_cost);  // 在最后一个level中查找
  }

// step 2: 根据父指针向前遍历，得到至后向前的一条路径，倒置一下就是cost最小的路径

  const auto *min_cost_node = &fake_head;
  while (min_cost_node->min_cost_prev_node) {
    min_cost_node = min_cost_node->min_cost_prev_node;
    min_cost_path->push_back(*min_cost_node);
  }
  if (min_cost_node != &graph_nodes.front().front()) {
    return false;
  }

  std::reverse(min_cost_path->begin(), min_cost_path->end());

}

额外补充一点，如果和比较两个节点的cost，除了cost有对应的值，还有三个优先级状态:

HAS_COLLISION: 存在碰撞
OUT_OF_BOUNDARY：超出边界
OUT_OF_LANE：超出车道
三个优先级依次从高到低。在比较两个cost(ComparableCost)时，首先比较cost优先级，如果优先级不一样，那么优先级高的cost大(不看cost值大小)；如果优先级一样，才比较cost值大小。



3. 路径决策器--PathDecider
这里的PathDecider与ReferenceLineInfo中的PathDecision很相似，PathDecision在Frame类初始化的过程中为每个PathObstacle添加标签，这个标签代表该障碍物的存在会对无人车造成什么影响。在初始化过程中，代码中仅仅利用路况以及路况中的障碍物位置，初步设定了障碍物的标签，例如人行横道路况下，在人行横道上的障碍物会无人车的影响，那么可以忽略，那么需要构建虚拟停车墙。但对于那些不在特殊路况下的障碍物(车辆虽然在道路线，但既不在停车区也不在禁停区交叉口等特殊区域)，Frame初始化的工作就没有为这些障碍物分配标签。

通过路径规划器已经为无人车规划好了一条行驶路线，那么根据这条规划路线和障碍物属性(位置)，就可以再一次更新障碍物的标签。例如有些障碍物在行驶路线上，那么就需要分这些障碍物分配停车Stop或者绕行SidePass等标签。

另外一个注意点，因为规划的路线只有位置信息，没有时间信息，所以路径决策器PathDecider仅仅只能为静态障碍物设定标签，无法为动态障碍物设定标签(动态障碍物需要在速度决策器SpeedDecider中更新标签)。

更新的过程其实比较简单，主要遵循一下原则：

非行人或者非机动车且非静态障碍物，忽略
前方和侧方已经存在忽略标签的障碍物，忽略
障碍物存在前方停车标签，忽略
障碍物存在侧方绕行标签，忽略
障碍物所在区域在禁停区，忽略
障碍物在规划路线以外，忽略
如果障碍物和无人车侧方距离大于一个阈值(半车距离+3m)，那么障碍物侧方增加忽略标签
否则如果障碍物和无人车侧方距离小于一个阈值(半车距离+0.5m)，那么就必须设置为停车
否者障碍物和无人车侧方距离在[半车距离+0.5m, 半车距离+3m]之间，允许左右微调，就将障碍物标签设置为微调



4. 速度规划器(动态规划)--DpStSpeedOptimizer
在路径规划中，已经给出了一条从规划起始点到规划终点开销cost最小的路径，如下图的蓝色星星组成的路径，每个蓝色的路径点都有相对于参考线的累计距离s和侧方相对距离l。
那么剩下最后一个问题：每个规划点的速度怎么设定？换句话说无人车应该在什么时间点到达轨迹点？这就需要考虑障碍物在每个时间间隔的位置。