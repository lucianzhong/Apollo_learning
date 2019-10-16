
// https://zhuanlan.zhihu.com/p/78330066
Apollo 3.5 


modules/planning/tasks/optimizers/qp_spline_path/qp_spline_path_optimizer.cc

(apollo3_5/modules/planning/tasks/optimizers/qp_spline_path/http://qp_spline_path_optimizer.cc) 的入口函数在该类的Process()，在Process()中构造了QpSplinePathGenerator类的实例path_generator，轨迹生成工作由path_generator.Generate()完成




modules/planning/tasks/optimizers/qp_spline_path/qp_spline_path_optimizer.cc
Status QpSplinePathOptimizer::Process(const SpeedData& speed_data,
                                      const ReferenceLine& reference_line,
                                      const common::TrajectoryPoint& init_point,
                                      PathData* const path_data) {
  QpSplinePathGenerator path_generator(spline_solver_.get(), reference_line,
                                       qp_spline_path_config_,
                                       reference_line_info_->AdcSlBoundary());
  path_generator.SetDebugLogger(reference_line_info_->mutable_debug());
  path_generator.SetChangeLane(reference_line_info_->IsChangeLanePath());

  double boundary_extension = 0.0;
  bool is_final_attempt = false;

 //第一次调用Generate()时，boundary_extension=0.0。

  bool ret = path_generator.Generate(
      reference_line_info_->path_decision()->obstacles().Items(), speed_data,
      init_point, boundary_extension, is_final_attempt, path_data);
  
  if (!ret) {
    AERROR << "failed to generate spline path with boundary_extension = 0.";

    boundary_extension = qp_spline_path_config_.cross_lane_lateral_extension();
    is_final_attempt = true;
  //若生成轨迹失败，则将boundary_extension增大为cross_lane_lateral_extension()（默认值1.2m)
  //以放松优化过程中的限制条件，使得更易求解，第二次调用Generate()
    ret = path_generator.Generate(
        reference_line_info_->path_decision()->obstacles().Items(), speed_data,
        init_point, boundary_extension, is_final_attempt, path_data);
    if (!ret) {
      const std::string msg =
          "failed to generate spline path at final attempt.";
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }

  return Status::OK();
}



QpSplinePathGenerator::Generate()中，重点关注以下函数：InitSpline()，QpFrenetFrame::Init()，AddConstraint()，AddKernel()，Solve()。

bool QpSplinePathGenerator::Generate(
    const std::vector<const Obstacle*>& obstacles, const SpeedData& speed_data,
    const common::TrajectoryPoint& init_point, const double boundary_extension,
    bool is_final_attempt, PathData* const path_data) {
  ...
  if (is_change_lane_path_) {
    ref_l_ = init_frenet_point_.l();
  }
  ...
  if (!InitSpline(start_s, end_s)) {    ...  }

  QpFrenetFrame qp_frenet_frame(reference_line_, speed_data, init_frenet_point_,
                                qp_spline_path_config_.time_resolution(),
                                evaluated_s_);
  if (!qp_frenet_frame.Init(obstacles)) {
    AERROR << "Fail to initialize qp frenet frame";
    return false;
  }
  qp_frenet_frame.LogQpBound(planning_debug_);

  if (!AddConstraint(qp_frenet_frame, boundary_extension)) {
    AERROR << "Fail to setup pss path constraint.";
    return false;
  }

  AddKernel();
  bool is_solved = Solve();
  ...
  // extract data
  ...
}





modules/planning/tasks/optimizers/qp_spline_path/qp_spline_path_generator.cc

//将纵向区间[start_s, end_s] 按照qp_spline_path_config_.max_spline_length()和qp_spline_path_config_.max_constraint_interval()的设置，
// 均匀分割后存入knots_和evaluated_s_，每一段都会对应一条qp_spline_path_config_.spline_order()次多项式曲线
bool QpSplinePathGenerator::InitSpline(const double start_s,
                                       const double end_s) {
  uint32_t number_of_spline = static_cast<uint32_t>(
      (end_s - start_s) / qp_spline_path_config_.max_spline_length() + 1.0);
  number_of_spline = std::max(1u, number_of_spline);
  common::util::uniform_slice(start_s, end_s, number_of_spline, &knots_);

  // spawn a new spline generator
  //产生number_of_spline条order阶spline
  spline_solver_->Reset(knots_, qp_spline_path_config_.spline_order());

  // set evaluated_s_
  uint32_t constraint_num = static_cast<uint32_t>(
      (end_s - start_s) / qp_spline_path_config_.max_constraint_interval() + 1);
  common::util::uniform_slice(start_s, end_s, constraint_num - 1,
                              &evaluated_s_);
  return (knots_.size() > 1) && !evaluated_s_.empty();
   //难道 max_spline_length 和 max_constraint_interval 可以不相等吗？
  //难道 constraint_num 和 number_of_spline 可以不相等吗？
}





modules/planning/tasks/optimizers/qp_spline_path/qp_frenet_frame.cc

//因为对轨迹的描述、方程表达、约束计算等都是在Frenet坐标系（SL坐标系）下进行的，
// 所有相关信息都在QpFrenetFrame类中转换到Frenet坐标
bool QpFrenetFrame::Init(const std::vector<const Obstacle*>& obstacles) {
  if (!CalculateDiscretizedVehicleLocation()) {
    AERROR << "Fail to calculate discretized vehicle location!";
    return false;
  }

  if (!CalculateHDMapBound()) {
    AERROR << "Calculate HDMap bound failed.";
    return false;
  }

  if (!CalculateObstacleBound(obstacles)) {
    AERROR << "Calculate obstacle bound failed!";
    return false;
  }
  return true;
}



// CalculateDiscretizedVehicleLocation()根据SpeedData按时间遍历计算了自车的纵向轨迹相关信息，存入vector<SpeedPoint>，
// 即discretized_vehicle_location_。目前，SpeedData的来源我还没有搞清楚。
bool QpFrenetFrame::CalculateDiscretizedVehicleLocation() {
  for (double relative_time = 0.0; relative_time < speed_data_.TotalTime();
       relative_time += time_resolution_) {
    SpeedPoint veh_point;
    if (!speed_data_.EvaluateByTime(relative_time, &veh_point)) {
      AERROR << "Fail to get speed point at relative time " << relative_time;
      return false;
    }
    veh_point.set_t(relative_time);
    discretized_vehicle_location_.push_back(std::move(veh_point));
  }
  return true;
}





// CalculateHDMapBound()计算了沿纵轴s均匀采样所得点集中各个点的左右边界
bool QpFrenetFrame::CalculateHDMapBound() {
  const double adc_half_width = vehicle_param_.width() / 2.0;
  for (uint32_t i = 0; i < hdmap_bound_.size(); ++i) {  //hdmap_bound_初始化为vector {evaluated_s_.size(), std::make_pair(-inf, inf)}
    double left_bound = 0.0;
    double right_bound = 0.0;
    bool suc = reference_line_.GetLaneWidth(evaluated_s_[i], &left_bound,
                                            &right_bound);
    if (!suc) {
      AWARN << "Extracting lane width failed at s = " << evaluated_s_[i];
      right_bound = FLAGS_default_reference_line_width / 2;
      left_bound = FLAGS_default_reference_line_width / 2;
    }
  //按照右手坐标系，自车前方为s轴正方向，hdmap_bound_[i].first是右侧边界，second是左侧边界
    hdmap_bound_[i].first = -right_bound + adc_half_width;
    hdmap_bound_[i].second = left_bound - adc_half_width;
//如果右边界>左边界，不合理，则缩短纵向s上限feasible_longitudinal_upper_bound_到当前考察点
    if (hdmap_bound_[i].first >= hdmap_bound_[i].second) {
      ADEBUG << "HD Map bound at " << evaluated_s_[i] << " is infeasible ("
             << hdmap_bound_[i].first << ", " << hdmap_bound_[i].second << ") ";
      ADEBUG << "left_bound: " << left_bound
             << ", right_bound: " << right_bound;

      feasible_longitudinal_upper_bound_ =
          std::min(evaluated_s_[i], feasible_longitudinal_upper_bound_);
      common::SLPoint sl;
      sl.set_s(evaluated_s_[i]);
      common::math::Vec2d xy;
      if (!reference_line_.SLToXY(sl, &xy)) {
        AERROR << "Fail to calculate HDMap bound at s: " << sl.s()
               << ", l: " << sl.l();
        return false;
      }
      ADEBUG << "evaluated point x: " << std::fixed << xy.x()
             << " y: " << xy.y();
      break;
    }
  }
  return true;
}



// CalculateObstacleBound()的处理分为2部分：静态障碍物和动态障碍物。首先判断障碍物是否有LateralDecision，若没有，则忽略，
// 因为对path没有影响（这个阶段的path重点考虑横向运动）。
// LateralDecision主要是指横向的nudge，具体decision相关信息可查阅apollo3_5/modules/planning/proto/decision.proto。
bool QpFrenetFrame::CalculateObstacleBound(
    const std::vector<const Obstacle*>& obstacles) {
  for (const auto ptr_obstacle : obstacles) {
    if (!ptr_obstacle->HasLateralDecision()) {
      continue;
    }
    if (ptr_obstacle->IsStatic()) {
      if (!MapStaticObstacleWithDecision(*ptr_obstacle)) {
        AERROR << "mapping obstacle with id [" << ptr_obstacle->Id()
               << "] failed in qp frenet frame.";
        return false;
      }
    } else {
      if (!MapDynamicObstacleWithDecision(*ptr_obstacle)) {
        AERROR << "mapping obstacle with id [" << ptr_obstacle->Id()
               << "] failed in qp frenet frame.";
        return false;
      }
    }
  }
  return true;
}





modules/planning/tasks/optimizers/qp_spline_path/qp_frenet_frame.cc

// MapStaticObstacleWithDecision()考虑了静态障碍物以及相应的横向避让措施对可行驶区域宽度的影响，
// 处理结果static_obstacle_bound_保存了evaluated_s_中各临界点处的横向可行驶范围。
// 主要思路是将静态障碍物的轮廓端点映射到Frenet坐标系，结合nudge的方向计算障碍物占据的横向范围，
// 由MapNudgePolygon()和MapNudgeLine()实现。
bool QpFrenetFrame::MapStaticObstacleWithDecision(const Obstacle& obstacle) {
  if (!obstacle.HasLateralDecision()) {
    ADEBUG << "obstacle has no lateral decision";
    return false;
  }
  const auto& decision = obstacle.LateralDecision();
  if (!decision.has_nudge()) {
    ADEBUG << "only support nudge decision now";
    return true;
  }
  if (!MapNudgePolygon(
          common::math::Polygon2d(obstacle.PerceptionBoundingBox()),
          decision.nudge(), &static_obstacle_bound_)) {
    AERROR << "fail to map polygon with id " << obstacle.Id()
           << " in qp frenet frame";
    return false;
  }
  return true;
}



bool QpFrenetFrame::MapNudgePolygon(
    const common::math::Polygon2d& polygon, const ObjectNudge& nudge,
    std::vector<std::pair<double, double>>* const bound_map) {
  std::vector<common::SLPoint> sl_corners;
  for (const auto& corner_xy : polygon.points()) {
    common::SLPoint corner_sl;
    if (!reference_line_.XYToSL(corner_xy, &corner_sl)) {
      AERROR << "Fail to map xy point " << corner_xy.DebugString() << " to "
             << corner_sl.DebugString();
      return false;
    }
    // shift box based on buffer
    // nudge decision buffer:
    // --- position for left nudge
    // --- negative for right nudge
     //nudge.distance_l()是带正负的，表示nudge的左右方向，同上
    corner_sl.set_l(corner_sl.l() + nudge.distance_l());
    sl_corners.push_back(std::move(corner_sl));
  }

  const auto corner_size = sl_corners.size();
  for (uint32_t i = 0; i < corner_size; ++i) {
    if (!MapNudgeLine(sl_corners[i], sl_corners[(i + 1) % corner_size],
                      nudge.type(), bound_map)) {
      AERROR << "Map box line (sl) " << sl_corners[i].DebugString() << "->"
             << sl_corners[(i + 1) % corner_size].DebugString();
      return false;
    }
  }
  return true;
}



bool QpFrenetFrame::MapNudgeLine(
    const common::SLPoint& start, const common::SLPoint& end,
    const ObjectNudge::Type nudge_type,
    std::vector<std::pair<double, double>>* const constraint) {
  DCHECK_NOTNULL(constraint);

  const common::SLPoint& near_point = (start.s() < end.s() ? start : end);
  const common::SLPoint& further_point = (start.s() < end.s() ? end : start);

   //impact_index表示evaluated_s_中受影响的区间范围，2个index可能是相等的
  std::pair<uint32_t, uint32_t> impact_index =
      FindInterval(near_point.s(), further_point.s());

  if (further_point.s() < start_s_ - vehicle_param_.back_edge_to_center() ||
      near_point.s() > end_s_ + vehicle_param_.front_edge_to_center()) {
    return true;
  }

  const double distance =
      std::max(further_point.s() - near_point.s(), common::math::kMathEpsilon);
  const double adc_half_width = vehicle_param_.width() / 2;

  DCHECK_GT(constraint->size(), impact_index.second);
  for (uint32_t i = impact_index.first; i <= impact_index.second; ++i) {
    double weight = std::fabs((evaluated_s_[i] - near_point.s())) / distance;
    weight = std::min(1.0, std::max(weight, 0.0));
    double boundary =
        near_point.l() * (1 - weight) + further_point.l() * weight;

    if (nudge_type == ObjectNudge::LEFT_NUDGE) {
      boundary += adc_half_width;
        //first是lower bound，second是upper bound
      //lower bound增大，即将自车可行驶区域向左移动，即left nudge
      //而upper bound不变，其初始化为INF
      (*constraint)[i].first = std::max(boundary, (*constraint)[i].first);
    } else {
      boundary -= adc_half_width;
      (*constraint)[i].second = std::min(boundary, (*constraint)[i].second);
    }
 //若可行驶区域宽度constraint太窄，则收缩feasible_longitudinal_upper_bound_
    //因为上面代码已经考虑了半车宽，故此处可以使用较小的数值0.3，车抽象成了点
    if ((*constraint)[i].second < (*constraint)[i].first + 0.3) {
      if (i > 0) {
        feasible_longitudinal_upper_bound_ =
            std::min(evaluated_s_[i - 1] - kEpsilonTol,
                     feasible_longitudinal_upper_bound_);
      } else {
        feasible_longitudinal_upper_bound_ = start_s_;
        return true;
      }

      ADEBUG << "current mapping constraint, sl point impact index "
             << "near_point: " << near_point.DebugString()
             << "further_point: " << further_point.DebugString()
             << "impact_index: " << impact_index << "(*constraint)[" << i << "]"
             << (*constraint)[i];
      break;
    }
  }

  return true;
}



本节主要讲QpSplinePathGenerator::AddConstraint()和Spline1dConstraint类

结合QP问题的形式，需要添加的约束分为等式约束和不等式约束。等式约束一定来自于已知的起点和终点处的状态条件以及joint point的n阶导连续性条件，不等式约束一定来自于沿着要求解方程的采样点性质。结合求s-l path的具体问题，约束又可以分为0阶path、1阶angle、2阶kappa、3阶方程约束。这些约束的具体细节，都封装在了Spline1dConstraint类中


modules/planning/tasks/optimizers/qp_spline_path/qp_spline_path_generator.cc















bool QpSplinePathGenerator::AddConstraint(const QpFrenetFrame& qp_frenet_frame,
                                          const double boundary_extension) {

  Spline1dConstraint* spline_constraint = spline_solver_->mutable_spline_constraint();
  
   //curve个数 * curve多项式参数个数
  const int dim = static_cast<int>((knots_.size() - 1) * (qp_spline_path_config_.spline_order() + 1));
  constexpr double param_range = 1e-4;

   //循环（curve个数）次，这是添加什么约束？待后面看看发挥什么作用？
  for (int i = qp_spline_path_config_.spline_order(); i < dim;i += qp_spline_path_config_.spline_order() + 1) {
    Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(1, dim);
    Eigen::MatrixXd bd = Eigen::MatrixXd::Zero(1, 1);
    mat(0, i) = -1;
    bd(0, 0) = -param_range;
    spline_constraint->AddInequalityConstraint(mat, bd);
    mat(0, i) = 1;
    bd(0, 0) = -param_range;
    spline_constraint->AddInequalityConstraint(mat, bd);
  }

  // add init status constraint, equality constraint
  //这里3个函数其实是添加了0，1，2阶导不等式约束 sample points for boundary
  const double kBoundaryEpsilon = 1e-4;
  spline_constraint->AddPointConstraintInRange(init_frenet_point_.s(),
                                               init_frenet_point_.l() - ref_l_,
                                               kBoundaryEpsilon);

  spline_constraint->AddPointDerivativeConstraintInRange(
      init_frenet_point_.s(), init_frenet_point_.dl(), kBoundaryEpsilon);
 //path二阶导其实是曲线的曲率kappa，U形急转弯不对kappa添加约束
  if (init_trajectory_point_.v() > qp_spline_path_config_.uturn_speed_limit()) {
    spline_constraint->AddPointSecondDerivativeConstraintInRange(
        init_frenet_point_.s(), init_frenet_point_.ddl(), kBoundaryEpsilon);
  }

  ADEBUG << "init frenet point: " << init_frenet_point_.ShortDebugString();


  // add end point constraint, equality constraint

  //lat_shift看不懂，为什么要与ref_l_反向？
  //推测：换道时有参考线切换，自车横向坐标在目标车道参考线坐标系下是ref_l_，目标点应该在目标车道参考线上采样，
  //所以目标点在当前车道参考线坐标系下横向坐标大概是fabs(ref_l_)，而正负一定相反
  //如果不变道，ref_l_=0，如果变道，ref_l_=init_frenet_point_.l();

  double lat_shift = -ref_l_;

  //如果是变道，则不约束一阶导即方向角，如果不是变道，则目标点的方向=0，即沿s轴
  if (is_change_lane_path_) {
    double lane_change_lateral_shift =  GetLaneChangeLateralShift(init_trajectory_point_.v());
    lat_shift = std::copysign( std::fmin(std::fabs(ref_l_), lane_change_lateral_shift), -ref_l_);
  }
  ADEBUG << "lat_shift = " << lat_shift;
  const double kEndPointBoundaryEpsilon = 1e-2;
  constexpr double kReservedDistance = 20.0;
  const double target_s =  std::fmin(qp_spline_path_config_.point_constraint_s_position(),
        kReservedDistance + init_frenet_point_.s() + init_trajectory_point_.v() * FLAGS_look_forward_time_sec);

// add end point constraint, equality constraint
  spline_constraint->AddPointConstraintInRange(target_s, lat_shift, kEndPointBoundaryEpsilon);

  if (!is_change_lane_path_) {
   spline_constraint->AddPointDerivativeConstraintInRange( evaluated_s_.back(), 0.0, kEndPointBoundaryEpsilon);
  }

  // add first derivative bound to improve lane change smoothness
  // add first derivative bound to improve lane change smoothness
  //dl_bound是0.1，arctan(0.1)=5.71度，这角度会不会太小了？
  std::vector<double> dl_lower_bound(evaluated_s_.size(), -FLAGS_dl_bound);
  std::vector<double> dl_upper_bound(evaluated_s_.size(), FLAGS_dl_bound);

  //添加不等式约束
  if (!spline_constraint->AddDerivativeBoundary(evaluated_s_, dl_lower_bound, dl_upper_bound)) {
    AERROR << "Fail to add second derivative boundary.";
    return false;
  }

  // kappa bound is based on the inequality:
  // kappa = d(phi)/ds <= d(phi)/dx = d2y/dx2  此式如何得来？
  std::vector<double> kappa_lower_bound(evaluated_s_.size(), -FLAGS_kappa_bound);
  std::vector<double> kappa_upper_bound(evaluated_s_.size(), FLAGS_kappa_bound);
  if (!spline_constraint->AddSecondDerivativeBoundary(evaluated_s_, kappa_lower_bound, kappa_upper_bound)) {
    AERROR << "Fail to add second derivative boundary.";
    return false;
  }

  // dkappa = d(kappa) / ds <= d3y/dx3
  std::vector<double> dkappa_lower_bound(evaluated_s_.size(), -FLAGS_dkappa_bound);
  std::vector<double> dkappa_upper_bound(evaluated_s_.size(),  FLAGS_dkappa_bound);
  if (!spline_constraint->AddThirdDerivativeBoundary( evaluated_s_, dkappa_lower_bound, dkappa_upper_bound)) {
    AERROR << "Fail to add third derivative boundary.";
    return false;
  }

  // add map bound constraint
  double lateral_buf = boundary_extension;
  if (is_change_lane_path_) {
    lateral_buf = qp_spline_path_config_.cross_lane_lateral_extension();
  }
  std::vector<double> boundary_low;
  std::vector<double> boundary_high;

  for (uint32_t i = 0; i < evaluated_s_.size(); ++i) {
    auto road_boundary = qp_frenet_frame.GetMapBound().at(i);
    auto static_obs_boundary = qp_frenet_frame.GetStaticObstacleBound().at(i);
    auto dynamic_obs_boundary = qp_frenet_frame.GetDynamicObstacleBound().at(i);

 //不明白这种情形指什么？
    if (evaluated_s_.at(i) - evaluated_s_.front() < qp_spline_path_config_.cross_lane_longitudinal_extension()) {
      //扩宽边界约束
      //右边界
      road_boundary.first = std::fmin(road_boundary.first, init_frenet_point_.l() - lateral_buf);
      //左边界
      road_boundary.second = std::fmax(road_boundary.second, init_frenet_point_.l() + lateral_buf);
    }

    boundary_low.emplace_back(common::util::MaxElement(
        std::vector<double>{road_boundary.first, static_obs_boundary.first, dynamic_obs_boundary.first}));
    boundary_high.emplace_back(common::util::MinElement(
        std::vector<double>{road_boundary.second, static_obs_boundary.second, dynamic_obs_boundary.second}));
    ADEBUG << "s[" << evaluated_s_[i] << "] boundary_low["
           << boundary_low.back() << "] boundary_high[" << boundary_high.back()
           << "] road_boundary_low[" << road_boundary.first
           << "] road_boundary_high[" << road_boundary.second
           << "] static_obs_boundary_low[" << static_obs_boundary.first
           << "] static_obs_boundary_high[" << static_obs_boundary.second
           << "] dynamic_obs_boundary_low[" << dynamic_obs_boundary.first
           << "] dynamic_obs_boundary_high[" << dynamic_obs_boundary.second
           << "].";
  }

  if (planning_debug_) {
    apollo::planning_internal::SLFrameDebug* sl_frame =
        planning_debug_->mutable_planning_data()->mutable_sl_frame()->Add();
    for (size_t i = 0; i < evaluated_s_.size(); ++i) {
      sl_frame->mutable_aggregated_boundary_s()->Add(evaluated_s_[i]);
      sl_frame->mutable_aggregated_boundary_low()->Add(boundary_low[i]);
      sl_frame->mutable_aggregated_boundary_high()->Add(boundary_high[i]);
    }
  }

//不等式约束
  const double start_l = ref_l_;
//求横向相对坐标
  std::for_each(boundary_low.begin(), boundary_low.end(), [start_l](double& d) { d -= start_l; });
  std::for_each(boundary_high.begin(), boundary_high.end(), [start_l](double& d) { d -= start_l; });
  if (!spline_constraint->AddBoundary(evaluated_s_, boundary_low, boundary_high)) {
    AERROR << "Add boundary constraint failed";
    return false;
  }

  // add spline joint third derivative constraint
  //等式约束
  if (knots_.size() >= 3 && !spline_constraint->AddThirdDerivativeSmoothConstraint()) {
    AERROR << "Add spline joint derivative constraint failed!";
    return false;
  }
  return true;
}




从上往下看AddConstraint()，首先是spline_constraint->AddInequalityConstraint()。其中又内部调用了AffineConstraint::AddConstraint()。AffineConstraint::AddConstraint()是最底层的函数，简明清晰。其他函数之所以复杂，正在于要构造这个函数的2个输入参数：constraint_matrix 和 constraint_boundary。

modules/planning/math/smoothing_spline/spline_1d_constraint.cc

bool Spline1dConstraint::AddInequalityConstraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {
  return inequality_constraint_.AddConstraint(constraint_matrix, constraint_boundary);
}




modules/planning/math/smoothing_spline/affine_constraint.cc

bool AffineConstraint::AddConstraint(
    const Eigen::MatrixXd& constraint_matrix,
    const Eigen::MatrixXd& constraint_boundary) {

  // 假设添加的约束是 Ax=b，constraint_matrix就是A，constraint_boundary就是b，x就是未知的多项式系数
  // 自然A的一行对应的Ax的计算结果就是b的一行，所以 A.rows==b.rows。
  // b的元素就是目标方程的计算结果，是一个实数，自然 b.cols==1
  if (constraint_matrix.rows() != constraint_boundary.rows()) {
    AERROR << "Fail to add constraint because constraint matrix rows != " "constraint boundary rows.";
    AERROR << "constraint matrix rows = " << constraint_matrix.rows();
    AERROR << "constraint boundary rows = " << constraint_boundary.rows();
    return false;
  }

  if (constraint_matrix_.rows() == 0) {
    constraint_matrix_ = constraint_matrix;
    constraint_boundary_ = constraint_boundary;
    return true;
  }
  if (constraint_matrix_.cols() != constraint_matrix.cols()) {
    AERROR << "constraint_matrix_ cols and constraint_matrix cols do not match.";
    AERROR << "constraint_matrix_.cols() = " << constraint_matrix_.cols();
    AERROR << "constraint_matrix.cols() = " << constraint_matrix.cols();
    return false;
  }
  if (constraint_boundary.cols() != 1) {
    AERROR << "constraint_boundary.cols() should be 1.";
    return false;
  }

  Eigen::MatrixXd n_matrix(constraint_matrix_.rows() + constraint_matrix.rows(), constraint_matrix_.cols());
  Eigen::MatrixXd n_boundary(constraint_boundary_.rows() + constraint_boundary.rows(), 1);

  n_matrix << constraint_matrix_, constraint_matrix;
  n_boundary << constraint_boundary_, constraint_boundary;
  constraint_matrix_ = n_matrix;
  constraint_boundary_ = n_boundary;
  return true;
}



spline_constraint->AddPointConstraintInRange()，添加0阶不等式约束，即考察目标函数f(x)与 [fx-range, fx+range] 的不等式关系。间接调用的Spline1dConstraint::AddBoundary()是理解constraint的核心，它就是在构造QP形式中Ax>=b约束中的A和b。



bool Spline1dConstraint::AddPointConstraintInRange(const double x,
                                                   const double fx,
                                                   const double range) {
  return AddConstraintInRange(
      std::bind(&Spline1dConstraint::AddBoundary, this, _1, _2, _3), x, fx,
      range);
}




bool Spline1dConstraint::AddConstraintInRange(AddConstraintInRangeFunc func,
                                              const double x, const double val,
                                              const double range) {
  if (range < 0.0) {
    return false;
  }
  std::vector<double> x_vec;
  x_vec.push_back(x);

  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
  lower_bound.push_back(val - range);
  upper_bound.push_back(val + range);
  return func(x_vec, lower_bound, upper_bound);
}






bool Spline1dConstraint::AddBoundary(const std::vector<double>& x_coord,
                                     const std::vector<double>& lower_bound,
                                     const std::vector<double>& upper_bound) {
  std::vector<double> filtered_lower_bound;
  std::vector<double> filtered_upper_bound;
  std::vector<double> filtered_lower_bound_x;
  std::vector<double> filtered_upper_bound_x;

  if (x_knots_.size() < 2) {
    return false;
  }

  if (!FilterConstraints(x_coord, lower_bound, upper_bound,
                         &filtered_lower_bound_x, &filtered_lower_bound,
                         &filtered_upper_bound_x, &filtered_upper_bound)) {
    return false;
  }

  // emplace affine constraints
  const uint32_t num_params = spline_order_ + 1;
  //inequality_constraint矩阵中绝大多数元素都是0，只有x_coord坐落的一段curve对应元素有值
  Eigen::MatrixXd inequality_constraint = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(), //行
      (x_knots_.size() - 1) * num_params); //列

  Eigen::MatrixXd inequality_boundary = Eigen::MatrixXd::Zero(
      filtered_upper_bound.size() + filtered_lower_bound.size(), 1);  


  //由AddPointConstraintInRange()推来，其实就是已知val=f(x0)，range>0，
  //x_coord=filtered_lower_bound_x=filtered_upper_bound_x，
  //filtered_lower_bound=lower_bound=val-range，
  //filtered_upper_bound=upper_bound=val+range，
  //由val-range<val<val+range ==> f(filtered_lower_bound_x) >= filtered_lower_bound
  //且 f(filtered_upper_bound_x) <= filtered_upper_bound


//这个for循环对应f(filtered_lower_bound_x) >= filtered_lower_bound的情况，恰好符合QP中Ax>=b的形式    
  for (uint32_t i = 0; i < filtered_lower_bound.size(); ++i) {
    //FindIndex()返回x_knots_中第一个大于入参的元素的前一个位置，即入参位于index～index+1之间
    uint32_t index = FindIndex(filtered_lower_bound_x[i]);
//corrected_x是每一段的相对起点的x坐标
    const double corrected_x = filtered_lower_bound_x[i] - x_knots_[index];
    double coef = 1.0;

    //j=0~num_params，依次是corrected_x的0~num_params次项
    //计算[1, x, x^2, x^3, x^4, x^5] * [a, b, c, d, e, f].T，[a, b, c, d, e, f]是5次curve的系数
    for (uint32_t j = 0; j < num_params; ++j) {
       //对特定行、特定curve的参数列更新，因为给定一个s，其根据定义域只对应一段curve
      inequality_constraint(i, j + index * num_params) = coef;
      coef *= corrected_x;
    }
    inequality_boundary(i, 0) = filtered_lower_bound[i];
  }
 //这个for循环对应f(filtered_upper_bound_x) <= filtered_upper_bound的情况，为符合QP中Ax>=b的形式
  //将Ax<=b ==> -Ax>=-b，故coef赋初值-1，与上部分比，全为负数
  for (uint32_t i = 0; i < filtered_upper_bound.size(); ++i) {
    uint32_t index = FindIndex(filtered_upper_bound_x[i]);
    const double corrected_x = filtered_upper_bound_x[i] - x_knots_[index];
    double coef = -1.0;
    for (uint32_t j = 0; j < num_params; ++j) {
      // 注意行列坐标
      inequality_constraint(i + filtered_lower_bound.size(),
                            j + index * num_params) = coef;
      coef *= corrected_x;
    }
    inequality_boundary(i + filtered_lower_bound.size(), 0) =
        -filtered_upper_bound[i];
  }

  return inequality_constraint_.AddConstraint(inequality_constraint,
                                              inequality_boundary);
}





本节主要分析optimization中的kernel，即要优化的目标，其实就是cost function


设定kernel函数，是由QpSplinePathGenerator::AddKernel()实现的。