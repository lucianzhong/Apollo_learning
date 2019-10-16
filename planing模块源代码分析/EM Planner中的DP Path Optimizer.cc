
https://zhuanlan.zhihu.com/p/78158531

Apollo 3.5

Process()----> FindPathTunnel() -----> GenerateMinCostPath() ------->  UpdateNode()------> Calculate(0)



DP-Path的主要思路是以自车当前位置为起点，沿着车道横纵向（横向为L或d，纵向为s，这里为了字母更清晰，横向统一以d表示）采样一些点，横向采样的一组点叫做level，点封装成node后，分别计算不同level间的node的cost，就构成了graph。
利用DP更新node的最小cost，便找到了代价最小的一条路径。听起来很像传统的图搜索方法找最短路径，区别在于cost包含了平滑、无碰的指标。




modules/planning/tasks/dp_poly_path/dp_poly_path_optimizer.cc




DpPolyPathOptimizer::DpPolyPathOptimizer(): PathOptimizer("DpPolyPathOptimizer") {}

bool DpPolyPathOptimizer::Init(const PlanningConfig &config) {
  config_ = config.em_planner_config().dp_poly_path_config();
  is_init_ = true;
  return true;
}

Status DpPolyPathOptimizer::Process(const SpeedData &speed_data, const ReferenceLine &,  const common::TrajectoryPoint &init_point, PathData *const path_data) {
  if (!is_init_) {
    AERROR << "Please call Init() before Process().";
    return Status(ErrorCode::PLANNING_ERROR, "Not inited.");
  }
  CHECK_NOTNULL(path_data);
  DPRoadGraph dp_road_graph(config_, *reference_line_info_, speed_data);
  dp_road_graph.SetDebugLogger(reference_line_info_->mutable_debug());

  if (!dp_road_graph.FindPathTunnel( init_point, reference_line_info_->path_decision()->path_obstacles().Items(), path_data )) {  // FindPathTunnel()主要分为3部分：先设置相关前提条件，然后查找代价最小路径，最后对每段代价最小路径采样以构造FrenetFramePath类的实例，并存入path_data中。
    AERROR << "Failed to find tunnel in road graph";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph path generation");
  }
  return Status::OK();
}


// DP函数入口在DpPolyPathOptimizer::Process()，该函数构造DpRoadGraph类的一个实例，所有构造graph、计算cost、search的过程都在dp_road_graph.FindPathTunnel()中。




modules/planning/tasks/dp_poly_path/dp_road_graph.cc

// FindPathTunnel()主要分为3部分：先设置相关前提条件，然后查找代价最小路径，最后对每段代价最小路径采样以构造FrenetFramePath类的实例，并存入path_data中。
//FindPathTunnel()的结果是依据若干level之间分段5次多项式的采样点，
//保存在path_data.frenet_path_（SL系）和path_data.discretized_path_（XY系）中

bool DpRoadGraph::FindPathTunnel(const common::TrajectoryPoint &init_point, const std::vector<const Obstacle *> &obstacles, PathData *const path_data) {
  CHECK_NOTNULL(path_data);

  init_point_ = init_point;
  if (!reference_line_.XYToSL(
          {init_point_.path_point().x(), init_point_.path_point().y()},
          &init_sl_point_)) {
    AERROR << "Fail to create init_sl_point from : "
           << init_point.DebugString();
    return false;
  }

  init_frenet_frame_point_ = reference_line_.GetFrenetPoint(init_point_.path_point()); //起始点所对应的在参考线上的点

  waypoint_sampler_->Init(&reference_line_info_, init_sl_point_,init_frenet_frame_point_);
  waypoint_sampler_->SetDebugLogger(planning_debug_);

  std::vector<DpRoadGraphNode> min_cost_path;
  if (!GenerateMinCostPath(obstacles, &min_cost_path)) {  // 查找代价最小路径的核心在于GenerateMinCostPath()，也是分为3部分：先采样，然后构造graph，最后查找从起点（自车当前位置）到终点（尽可能远的某个采样点）的代价最小路径。
    AERROR << "Fail to generate graph!";
    return false;
  }
  std::vector<common::FrenetFramePoint> frenet_path;
  double accumulated_s = min_cost_path.front().sl_point.s();
  const double path_resolution = config_.path_resolution();

  for (size_t i = 1; i < min_cost_path.size(); ++i) {
    const auto &prev_node = min_cost_path[i - 1];
    const auto &cur_node = min_cost_path[i];

    const double path_length = cur_node.sl_point.s() - prev_node.sl_point.s();
    double current_s = 0.0;
    const auto &curve = cur_node.min_cost_curve;
     //对每一段curve采样
    while (current_s + path_resolution / 2.0 < path_length) {
      const double l = curve.Evaluate(0, current_s);
      const double dl = curve.Evaluate(1, current_s);
      const double ddl = curve.Evaluate(2, current_s);
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
  path_data->SetReferenceLine(&reference_line_);
  path_data->SetFrenetPath(FrenetFramePath(std::move(frenet_path)));
  return true;
}




// 查找代价最小路径的核心在于GenerateMinCostPath()，也是分为3部分：先采样，然后构造graph，最后查找从起点（自车当前位置）到终点（尽可能远的某个采样点）的代价最小路径。
bool DpRoadGraph::GenerateMinCostPath(
    const std::vector<const Obstacle *> &obstacles,
    std::vector<DpRoadGraphNode> *min_cost_path) {
  CHECK(min_cost_path != nullptr);

  std::vector<std::vector<common::SLPoint>> path_waypoints;
    //对所在车道横纵向采样，结果存入path_waypoints，其中的每个元素是从横向一个level上采样得到的小点集
  if (!waypoint_sampler_->SamplePathWaypoints(init_point_, &path_waypoints) ||  path_waypoints.size() < 1) {
    AERROR << "Fail to sample path waypoints! reference_line_length = " << reference_line_.Length();
    return false;
  }
  const auto &vehicle_config = common::VehicleConfigHelper::Instance()->GetConfig();

  TrajectoryCost trajectory_cost(config_, reference_line_, reference_line_info_.IsChangeLanePath(), obstacles, vehicle_config.vehicle_param(), speed_data_, init_sl_point_, reference_line_info_.AdcSlBoundary());

  //构造graph，graph_nodes中的每个元素是依据横向一个level上采样得到点集构成的
  std::list<std::list<DpRoadGraphNode>> graph_nodes;

  // find one point from first row
  const auto &first_row = path_waypoints.front();
  size_t nearest_i = 0;
    //在第一行（横向）中寻找最靠近起始点的node，忽略了其他点
  //如果以当前所在点为起始点的话，就没必要在所在横向上采样其他点了
  for (size_t i = 1; i < first_row.size(); ++i) {
    if (std::fabs(first_row[i].l() - init_sl_point_.l()) <
        std::fabs(first_row[nearest_i].l() - init_sl_point_.l())) {
      nearest_i = i;
    }
  }
  graph_nodes.emplace_back();
  graph_nodes.back().emplace_back(first_row[nearest_i], nullptr,
                                  ComparableCost());
  auto &front = graph_nodes.front().front();
  size_t total_level = path_waypoints.size();

  for (size_t level = 1; level < path_waypoints.size(); ++level) {
    const auto &prev_dp_nodes = graph_nodes.back();
    const auto &level_points = path_waypoints[level];

    graph_nodes.emplace_back();
    std::vector<std::future<void>> results;

    for (size_t i = 0; i < level_points.size(); ++i) {
      const auto &cur_point = level_points[i];

      graph_nodes.back().emplace_back(cur_point, nullptr);

      auto msg = std::make_shared<RoadGraphMessage>(
          prev_dp_nodes, level, total_level, &trajectory_cost, &front,
          &(graph_nodes.back().back()));

      if (FLAGS_enable_multi_thread_in_dp_poly_path) {
        results.emplace_back(cyber::Async(&DpRoadGraph::UpdateNode, this, msg));
      } else {
        UpdateNode(msg);   //更新node的cost，更新后，该node保存了起点到该点的最小代价、prev node、prev node ---> node间的5次polynomial curve
      }
    }
    if (FLAGS_enable_multi_thread_in_dp_poly_path) {
      for (auto &result : results) {
        result.get();
      }
    }
  }

  //以上只是把2 node间的代价计算出来了，并没有寻找代价最小的路径
  // find best path
  DpRoadGraphNode fake_head;
  for (const auto &cur_dp_node : graph_nodes.back()) {
  	    //在最后的level采样点中，寻找代价最小的节点，作为一次规划的终点，即fake_head的prev
    //其实其他level的采样点很可能代价更小，这里为什么从最后的level选呢？
    //因为规划的长度和时间越长越好，规划越长，相当于自车看到的越远
    fake_head.UpdateCost(&cur_dp_node, cur_dp_node.min_cost_curve,
                         cur_dp_node.min_cost);
  }

  const auto *min_cost_node = &fake_head;
   //Todo:min_cost_path没有把fake_head保存进去，fake_head->min_cost_prev_node才是真正的终点
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


// 采样由WaypointSampler::SamplePathWaypoints()完成。该函数冗长而清晰，在此不多介绍。需注意一点是函数内有很多预设值，实际调试时需注意其影响。

/modules/planning/tasks/optimizers/road_graph/waypoint_sampler.cc
bool WaypointSampler::SamplePathWaypoints(
    const common::TrajectoryPoint &init_point,
    std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK_NOTNULL(points);
  points->clear();
  points->insert(points->begin(), std::vector<common::SLPoint>{init_sl_point_});

  const double kMinSampleDistance = 40.0;
  const double total_length = std::fmin(
      init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance),
      reference_line_info_->reference_line().Length());
  const auto &vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double half_adc_width = vehicle_config.vehicle_param().width() / 2.0;
  const double num_sample_per_level =
      FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level()
                                : config_.sample_points_num_each_level();

  constexpr double kSamplePointLookForwardTime = 4.0;
  const double level_distance =
      common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,
                          config_.step_length_min(), config_.step_length_max());

  double accumulated_s = init_sl_point_.s();
  double prev_s = accumulated_s;

  constexpr size_t kNumLevel = 3;
  for (size_t i = 0; i < kNumLevel && accumulated_s < total_length; ++i) {
    accumulated_s += level_distance;
    if (accumulated_s + level_distance / 2.0 > total_length) {
      accumulated_s = total_length;
    }
    const double s = std::fmin(accumulated_s, total_length);
    constexpr double kMinAllowedSampleStep = 1.0;
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
      continue;
    }
    prev_s = s;

    double left_width = 0.0;
    double right_width = 0.0;
    reference_line_info_->reference_line().GetLaneWidth(s, &left_width,
                                                        &right_width);

    constexpr double kBoundaryBuff = 0.20;
    const double eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    const double eff_left_width = left_width - half_adc_width - kBoundaryBuff;

    // the heuristic shift of L for lane change scenarios
    const double delta_dl = 1.2 / 20.0;
    const double kChangeLaneDeltaL = common::math::Clamp(
        level_distance * (std::fabs(init_frenet_frame_point_.dl()) + delta_dl),
        1.2, 3.5);

    double kDefaultUnitL = kChangeLaneDeltaL / (num_sample_per_level - 1);
    if (reference_line_info_->IsChangeLanePath() &&
        LaneChangeDecider::IsClearToChangeLane(reference_line_info_)) {
      kDefaultUnitL = 1.0;
    }
    const double sample_l_range = kDefaultUnitL * (num_sample_per_level - 1);
    double sample_right_boundary = -eff_right_width;
    double sample_left_boundary = eff_left_width;

    constexpr double kLargeDeviationL = 1.75;
    constexpr double kTwentyMilesPerHour = 8.94;
    if (reference_line_info_->IsChangeLanePath() ||
        std::fabs(init_sl_point_.l()) > kLargeDeviationL) {
      if (EgoInfo::Instance()->start_point().v() > kTwentyMilesPerHour) {
        sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());
        sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());

        if (init_sl_point_.l() > eff_left_width) {
          sample_right_boundary = std::fmax(
              sample_right_boundary, init_sl_point_.l() - sample_l_range);
        }
        if (init_sl_point_.l() < eff_right_width) {
          sample_left_boundary = std::fmin(sample_left_boundary,
                                           init_sl_point_.l() + sample_l_range);
        }
      }
    }

    std::vector<double> sample_l;
    if (reference_line_info_->IsChangeLanePath() &&
        LaneChangeDecider::IsClearToChangeLane(reference_line_info_)) {
      sample_l.push_back(reference_line_info_->OffsetToOtherReferenceLine());
    } else {
      common::util::uniform_slice(
          sample_right_boundary, sample_left_boundary,
          static_cast<uint32_t>(num_sample_per_level - 1), &sample_l);
    }
    std::vector<common::SLPoint> level_points;
    planning_internal::SampleLayerDebug sample_layer_debug;
    for (size_t j = 0; j < sample_l.size(); ++j) {
      common::SLPoint sl = common::util::MakeSLPoint(s, sample_l[j]);
      sample_layer_debug.add_sl_point()->CopyFrom(sl);
      level_points.push_back(std::move(sl));
    }
    if (!level_points.empty()) {
      planning_debug_->mutable_planning_data()
          ->mutable_dp_poly_graph()
          ->add_sample_layer()
          ->CopyFrom(sample_layer_debug);
      points->emplace_back(level_points);
    }
  }
  return true;
}




modules/planning/tasks/optimizers/road_graph/dp_road_graph.cc

// DpRoadGraph::UpdateNode()也是主要做了3件事：在current node与任一prev node间、以及current node与first node间，构造5次polynomial curve；计算这2个node间的cost；更新current node的cost。

void DpRoadGraph::UpdateNode(const std::shared_ptr<RoadGraphMessage> &msg) {
  DCHECK_NOTNULL(msg);
  DCHECK_NOTNULL(msg->trajectory_cost);
  DCHECK_NOTNULL(msg->front);
  DCHECK_NOTNULL(msg->cur_node);
  for (const auto &prev_dp_node : msg->prev_nodes) {
    const auto &prev_sl_point = prev_dp_node.sl_point;
    const auto &cur_point = msg->cur_node->sl_point;
    double init_dl = 0.0;
    double init_ddl = 0.0;
    if (msg->level == 1) { //仅自车当前姿态有dl（角度朝向）,ddl，其余点的dl,ddl都是0
      init_dl = init_frenet_frame_point_.dl();
      init_ddl = init_frenet_frame_point_.ddl();
    }

    //纵向相邻level间的任意两点间，都是一条5次多项式曲线
    QuinticPolynomialCurve1d curve(prev_sl_point.l(), init_dl, init_ddl, cur_point.l(), 0.0, 0.0, cur_point.s() - prev_sl_point.s());

    if (!IsValidCurve(curve)) {
      continue;
    }
    const auto cost =
        msg->trajectory_cost->Calculate(curve, prev_sl_point.s(), cur_point.s(),
                                        msg->level, msg->total_level) +
        prev_dp_node.min_cost;

    msg->cur_node->UpdateCost(&prev_dp_node, curve, cost);
  }

  // try to connect the current point with the first point directly
  //自车当前点（即起始点）与后面任意采样点间都构造了一条5次多项式曲线
  if (reference_line_info_.IsChangeLanePath() && msg->level >= 2) {
    const double init_dl = init_frenet_frame_point_.dl();
    const double init_ddl = init_frenet_frame_point_.ddl();
    QuinticPolynomialCurve1d curve(
        init_sl_point_.l(), init_dl, init_ddl, msg->cur_node->sl_point.l(), 0.0,
        0.0, msg->cur_node->sl_point.s() - init_sl_point_.s());
    if (!IsValidCurve(curve)) {
      return;
    }
    const auto cost = msg->trajectory_cost->Calculate(
        curve, init_sl_point_.s(), msg->cur_node->sl_point.s(), msg->level,
        msg->total_level);
    msg->cur_node->UpdateCost(msg->front, curve, cost);
  }
}




// DpRoadGraph::UpdateNode()也是主要做了3件事：在current node与任一prev node间、以及current node与first node间，构造5次polynomial curve；计算这2个node间的cost；更新current node的cost。

modules/planning/tasks/optimizers/road_graph/trajectory_cost.cc

// TODO(All): optimize obstacle cost calculation time
ComparableCost TrajectoryCost::Calculate(const QuinticPolynomialCurve1d &curve,  const double start_s, const double end_s,  const uint32_t curr_level,  const uint32_t total_level) {
  ComparableCost total_cost;
  // path cost, smoothness cost
  total_cost += CalculatePathCost(curve, start_s, end_s, curr_level, total_level);

  // static obstacle cost
  total_cost += CalculateStaticObstacleCost(curve, start_s, end_s);

  // dynamic obstacle cost
  total_cost += CalculateDynamicObstacleCost(curve, start_s, end_s);
  return total_cost;
}


// node间的cost主要分为3部分：路径平滑、避开静态障碍物、避开动态障碍物，使用ComparableCost类描述。该类除包含常规的safety_cost（无碰）和smoothness_cost（平滑）外，还含有3个bool型的成员变量，以表示是否碰撞、是否超出边界、是否超出车道线。




modules/planning/tasks/dp_poly_path/comparable_cost.h

  /*
   * cost_items represents an array of factors that affect the cost,
   * The level is from most critical to less critical.
   * It includes:
   * (0) has_collision or out_of_boundary
   * (1) out_of_lane
   *
   * NOTICE: Items could have same critical levels
   */
  static const size_t HAS_COLLISION = 0;
  static const size_t OUT_OF_BOUNDARY = 1;
  static const size_t OUT_OF_LANE = 2;
  std::array<bool, 3> cost_items = {{false, false, false}};

  // cost from distance to obstacles or boundaries
  float safety_cost = 0.0f;
  // cost from deviation from lane center, path curvature etc
  float smoothness_cost = 0.0f;





  // 与EM官方论文不同，第一部分cost的计算，即代码中所称的path cost，既包含了论文中的Csmooth（只使用了一二阶导，没有使用三阶导），又包含了论文中的Cguidance（其实公式中的g(s)就是reference line，f(s)-g(s)就是横向坐标d）。除此之外，还考虑了起点与终点的横向偏移。

  ComparableCost TrajectoryCost::CalculatePathCost( const QuinticPolynomialCurve1d &curve, const double start_s, const double end_s, const uint32_t curr_level, const uint32_t total_level) {
  ComparableCost cost;
  double path_cost = 0.0;
  std::function<double(const double)> quasi_softmax = [this](const double x) {
    const double l0 = this->config_.path_l_cost_param_l0();
    const double b = this->config_.path_l_cost_param_b();
    const double k = this->config_.path_l_cost_param_k();
    return (b + std::exp(-k * (x - l0))) / (1.0 + std::exp(-k * (x - l0)));
  };

  for (double curve_s = 0.0; curve_s < (end_s - start_s);
    curve_s += config_.path_resolution()) {
    const double l = curve.Evaluate(0, curve_s);
 //0阶
    path_cost += l * l * config_.path_l_cost() * quasi_softmax(std::fabs(l));

    const double dl = std::fabs(curve.Evaluate(1, curve_s));
    if (IsOffRoad(curve_s + start_s, l, dl, is_change_lane_path_)) {
      cost.cost_items[ComparableCost::OUT_OF_BOUNDARY] = true;
    }
 //一阶
    path_cost += dl * dl * config_.path_dl_cost();
 //二阶
    const double ddl = std::fabs(curve.Evaluate(2, curve_s));
    path_cost += ddl * ddl * config_.path_ddl_cost();
  }
  path_cost *= config_.path_resolution();

  if (curr_level == total_level) {
    const double end_l = curve.Evaluate(0, end_s - start_s);
    path_cost +=
        std::sqrt(end_l - init_sl_point_.l() / 2.0) * config_.path_end_l_cost();
  }
  cost.smoothness_cost = path_cost;
  return cost;
}