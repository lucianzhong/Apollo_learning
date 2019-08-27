 Reference:
 https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/frame.md

// apollo 3.0


1.  在Frame类中，主要的工作还是对障碍物预测轨迹(由Predition模块得到的未来5s内障碍物运动轨迹)、无人车参考线(ReferenceLineProvider类提供)以及当前路况(停车标志、人行横道、减速带等)信息进行融合。
	实际情况下，能影响无人车运动的不一定只有障碍物，同时还有各个路况，举个例子：

	a. 障碍物影响
		情况1：无人车车后的障碍物，对无人车没太大影响，可以忽略
		情况2：无人车前面存在障碍物，无人车就需要停车或者超车

	b. 路况影响
		情况3：前方存在禁停区或者交叉路口(不考虑信号灯)，那么无人车在参考线上行驶，禁停区区域不能停车
		情况4：前方存在人行横道，若有人，那么需要停车；若没人，那么无人车可以驶过

	所以综上所述，其实这章节最重要的工作就是结合路况和障碍物轨迹，给每个障碍物(为了保持一致，路况也需要封装成障碍物形式)一个标签，这个标签表示该障碍物存在情况下对无人车的影响，例如有些障碍物可忽略，有些障碍物会促使无人车超车，有些障碍物促使无人车需要停车等。

		1.障碍物信息的获取策略
		2.无人车参考线ReferenceLineInof初始化(感知模块障碍物获取)
		3.依据交通规则对障碍物设定标签(原始感知障碍物&&路况障碍物)



2.  // 障碍物信息的获取策略--滞后预测(Lagged Prediction)

	主要的工作是获取Prediction模块发布的障碍物预测轨迹数据，并且进行后处理工作

	Prediction模块发布的数据格式：
	// apollo/modules/prediction/proto/prediction_obstacle.proto

	message Trajectory {
	  optional double probability = 1;  // probability of this trajectory //障碍物该轨迹运动方案的概率
	  repeated apollo.common.TrajectoryPoint trajectory_point = 2;
	}

	message PredictionObstacle {
	  optional apollo.perception.PerceptionObstacle perception_obstacle = 1;
	  optional double timestamp = 2;  // GPS time in seconds
	  // the length of the time for this prediction (e.g. 10s)
	  optional double predicted_period = 3;
	  // can have multiple trajectories per obstacle
	  repeated Trajectory trajectory = 4;
	}

	message PredictionObstacles {
	  // timestamp is included in header
	  optional apollo.common.Header header = 1;
	  // make prediction for multiple obstacles
	  repeated PredictionObstacle prediction_obstacle = 2;
	  // perception error code
	  optional apollo.common.ErrorCode perception_error_code = 3;
	  // start timestamp
	  optional double start_timestamp = 4;
	  // end timestamp
	  optional double end_timestamp = 5;
	}

	可以使用prediction_obstacles.prediction_obstacle()形式获取所有障碍物的轨迹信息，对于每个障碍物 prediction_obstacle ，可以使用prediction_obstacle.trajectory()获取他所有可能运动方案/轨迹

	可以使用const auto& prediction = *(AdapterManager::GetPrediction());来获取Adapter中所有已发布的历史消息，最常见的肯定是取最近发布的PredictionObstacles(prediction.GetLatestObserved())，
	但是Apollo中采用更为精确地障碍物预测获取方式--滞后预测(Lagged Prediction)，除了使用Prediction模块最近一次发布的信息，同时还是用历史信息中的障碍物轨迹预测数据


	// apollo/modules/planning/planning.cc

	Status Planning::InitFrame(const uint32_t sequence_num, const TrajectoryPoint& planning_start_point, const double start_time, const VehicleState& vehicle_state) {
	  frame_.reset(new Frame(sequence_num, planning_start_point, start_time, vehicle_state, reference_line_provider_.get()));
	  auto status = frame_->Init();
	  if (!status.ok()) {
	    AERROR << "failed to init frame:" << status.ToString();
	    return status;
	  }
	  return Status::OK();
	}

	// apollo/modules/planning/common/frame.cc
	Status Frame::Init() {
	  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
	  vehicle_state_ = common::VehicleStateProvider::instance()->vehicle_state();
	  const auto &point = common::util::MakePointENU(vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
	  if (std::isnan(point.x()) || std::isnan(point.y())) {
	    AERROR << "init point is not set";
	    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
	  }
	  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha  << FLAGS_align_prediction_time;
	  // prediction
	  if (FLAGS_enable_prediction && AdapterManager::GetPrediction() && !AdapterManager::GetPrediction()->Empty()) {
	    if (FLAGS_enable_lag_prediction && lag_predictor_) {
	      lag_predictor_->GetLaggedPrediction(&prediction_);  // 滞后预测策略，获取障碍物轨迹信息
	    } else {
	      prediction_.CopyFrom(AdapterManager::GetPrediction()->GetLatestObserved());  // 不采用滞后预测策略，直接取最近一次Prediction模块发布的障碍物信息
	    }
	    if (FLAGS_align_prediction_time) {
	      AlignPredictionTime(vehicle_state_.timestamp(), &prediction_);
	    }
	    for (auto &ptr : Obstacle::CreateObstacles(prediction_)) {
	      AddObstacle(*ptr);
	    }
	  }
	  const auto *collision_obstacle = FindCollisionObstacle();
	  if (collision_obstacle) {
	    std::string err_str = "Found collision with obstacle: " + collision_obstacle->Id();
	    apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
	    buffer.ERROR(err_str);
	    return Status(ErrorCode::PLANNING_ERROR, err_str);
	  }
	  if (!CreateReferenceLineInfo()) {
	    AERROR << "Failed to init reference line info";
	    return Status(ErrorCode::PLANNING_ERROR, "failed to init reference line info");
	  }
	  return Status::OK();
	}




3.  采用滞后预测策略获取障碍物轨迹信息的主要步骤可分为：
	// apollo/modules/planning/common/lag_prediction.cc

	void LagPrediction::GetLaggedPrediction(PredictionObstacles* obstacles) const {
	  obstacles->mutable_prediction_obstacle()->Clear();
	  if (!AdapterManager::GetPrediction() || AdapterManager::GetPrediction()->Empty()) {
	    return;
	  }
	  const auto& prediction = *(AdapterManager::GetPrediction());
	  if (!AdapterManager::GetLocalization() ||  AdapterManager::GetLocalization()->Empty()) {  // no localization
	    obstacles->CopyFrom(prediction.GetLatestObserved());
	    return;
	  }

	   // Step A. 最近一次发布的障碍物轨迹预测信息处理
	  const auto adc_position =  AdapterManager::GetLocalization()->GetLatestObserved().pose().position();
	  const auto latest_prediction = (*prediction.begin());						// // 记录最近一次Prediction模块发布的信息
	  const double timestamp = latest_prediction->header().timestamp_sec();			// 最近一次发布的时间戳
	  std::unordered_set<int> protected_obstacles;
	  for (const auto& obstacle : latest_prediction->prediction_obstacle()) {		// 获取最近一次发布的数据中，每个障碍物的运动轨迹信息
	    const auto& perception = obstacle.perception_obstacle();
	    if (perception.confidence() < FLAGS_perception_confidence_threshold &&   perception.type() != PerceptionObstacle::VEHICLE) {  // 障碍物置信度必须大于0.5，获取必须是车辆VEHICLE类，否则不作处理
	      continue;
	    }
	    double distance = common::util::DistanceXY(perception.position(), adc_position);
	    if (distance < FLAGS_lag_prediction_protection_distance) {				// 障碍物与车辆之间的距离小于30m，才设置有效
	      protected_obstacles.insert(obstacle.perception_obstacle().id());
	      // add protected obstacle
	      AddObstacleToPrediction(0.0, obstacle, obstacles);
	    }
	  }

	  // Step B 过往发布的历史障碍物轨迹预测信息处理
	  std::unordered_map<int, LagInfo> obstacle_lag_info;
	  int index = 0;  // data in begin() is the most recent data
	  for (auto it = prediction.begin(); it != prediction.end(); ++it, ++index) {		// 对于每一次发布的信息进行处理
	    for (const auto& obstacle : (*it)->prediction_obstacle()) {		// 获取该次发布的历史数据中，每个障碍物的运动轨迹信息
	      const auto& perception = obstacle.perception_obstacle();
	      auto id = perception.id();
	      if (perception.confidence() < FLAGS_perception_confidence_threshold &&  perception.type() != PerceptionObstacle::VEHICLE) {  // 障碍物置信度必须大于0.5，获取必须是车辆VEHICLE类，否则不作处理
	        continue;
	      }			
	      if (protected_obstacles.count(id) > 0) {    // 如果障碍物在最近一次发布的信息中出现了，那就忽略，因为只考虑最新的障碍物信息
	        continue;  // don't need to count the already added protected obstacle
	      }
	      auto& info = obstacle_lag_info[id];
	      ++info.count;				//// 记录障碍物在所有历史信息中出现的次数
	      if ((*it)->header().timestamp_sec() > info.last_observed_time) {   // 保存最近一次出现的信息，因为只考虑最新的障碍物信息
	        info.last_observed_time = (*it)->header().timestamp_sec();
	        info.last_observed_seq = index;
	        info.obstacle_ptr = &obstacle;
	      }
	    }
	  }

	  obstacles->mutable_header()->CopyFrom(latest_prediction->header());
	  obstacles->mutable_header()->set_module_name("lag_prediction");
	  obstacles->set_perception_error_code(latest_prediction->perception_error_code());
	  obstacles->set_start_timestamp(latest_prediction->start_timestamp());
	  obstacles->set_end_timestamp(latest_prediction->end_timestamp());
	  bool apply_lag = std::distance(prediction.begin(), prediction.end()) >= static_cast<int32_t>(min_appear_num_);
	  for (const auto& iter : obstacle_lag_info) {
	    if (apply_lag && iter.second.count < min_appear_num_) {		 // 历史信息中如果障碍物出现次数小于min_appear_num_/3次，次数太少，可忽略。
	      continue;
	    }
	    if (apply_lag && iter.second.last_observed_seq > max_disappear_num_) {   // 历史信息中如果障碍物最近一次发布距离现在过远，可忽略。
	      continue;
	    }
	    AddObstacleToPrediction(timestamp - iter.second.last_observed_time, *(iter.second.obstacle_ptr), obstacles);
	  }
	}

	所以最后做一个总结，对于历史发布数据，如何判断这些障碍物轨迹信息是否有效。两个步骤：

	步骤1：记录历史发布数据中每个障碍物出现的次数(在最近依次发布中出现的障碍物忽略，因为不是最新的数据了)，必须满足两个条件：
		障碍物置信度(Perception模块CNN分割获得)必须大于0.5，或者障碍物是车辆类
		障碍物与车辆之间的距离小于30m

	步骤2：对于步骤1中得到的障碍物信息，进行筛选，信息有效需要满足两个条件：
		信息队列中历史数据大于3(min_appear_num_)，并且每个障碍物出现次数大于3(min_appear_num_)
		信息队列中历史数据大于3(min_appear_num_)，并且障碍物信息上一次发布距离最近一次发布不大于5(max_disappear_num_)，需要保证数据的最近有效性。



4.  // 无人车与障碍物相对位置的设置--ReferenceLineInfo类初始化

	从障碍物信息的获取策略--滞后预测(Lagged Prediction)中可以得到障碍物短期(未来5s)内的运动轨迹；从ReferenceLineProvider类中我们可以得到车辆的理想规划轨迹
	下一步就是将障碍物的轨迹信息加入到这条规划好的参考线ReferenceLine中，确定在什么时间点，无人车能前进到什么位置，需要保证在这个时间点上，障碍物与无人车不相撞。这个工作依旧是在Frame::Init()中完成，主要是完成ReferenceLineInfo类的生成，这个类综合了障碍物预测轨迹与无人车规划轨迹的信息，同时也是最后路径规划的基础类

	// apollo/modules/planning/common/frame.cc

	Status Frame::Init() {
		  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
		  vehicle_state_ = common::VehicleStateProvider::instance()->vehicle_state();
		  const auto &point = common::util::MakePointENU(vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
		  if (std::isnan(point.x()) || std::isnan(point.y())) {
		    AERROR << "init point is not set";
		    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
		  }
		  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha << FLAGS_align_prediction_time;

		 // prediction
		 // Step A prediction，障碍物预测轨迹信息获取，采用滞后预测策略
		  if (FLAGS_enable_prediction && AdapterManager::GetPrediction() && !AdapterManager::GetPrediction()->Empty()) {
		    if (FLAGS_enable_lag_prediction && lag_predictor_) {
		      lag_predictor_->GetLaggedPrediction(&prediction_);
		    } else {
		      prediction_.CopyFrom(AdapterManager::GetPrediction()->GetLatestObserved());
		    }
		    if (FLAGS_align_prediction_time) {
		      AlignPredictionTime(vehicle_state_.timestamp(), &prediction_);
		    }
		    for (auto &ptr : Obstacle::CreateObstacles(prediction_)) {
		      AddObstacle(*ptr);
		    }
		  }

		 // Step B.1 检查当前时刻(relative_time=0.0s)，无人车位置和障碍物位置是否重叠(相撞)，如果是，可以直接退出
		  const auto *collision_obstacle = FindCollisionObstacle();
		  if (collision_obstacle) {
		    std::string err_str = "Found collision with obstacle: " + collision_obstacle->Id();
		    apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
		    buffer.ERROR(err_str);
		    return Status(ErrorCode::PLANNING_ERROR, err_str);
		  }

		  // // Step B.2 如果当前时刻不冲突，检查未来时刻无人车可以在参考线上前进的位置，ReferenceLineInfo生成
		  if (!CreateReferenceLineInfo()) {
		    AERROR << "Failed to init reference line info";
		    return Status( ErrorCode::PLANNING_ERROR, "failed to init reference line info");
		  }
		  return Status::OK();
		}


	如何完成 ReferenceLineInfo 类的初始化工作，其实比较简单，主要有两个过程：

		1.根据无人车规划路径ReferenceLine实例化ReferenceLineInfo类，数量与ReferenceLine一致
		2.根据障碍物轨迹初始化ReferenceLineInfo::path_decision_

	
	
	apollo/modules/planning/common/frame.cc

		bool Frame::CreateReferenceLineInfo() {
		  // Step A 从ReferenceLineProvider中获取无人车的短期内规划路径ReferenceLine，并进行收缩操作   // 1. 实例化ReferenceLineInfo类:
		  std::list<ReferenceLine> reference_lines;
		  std::list<hdmap::RouteSegments> segments;
		  if (!reference_line_provider_->GetReferenceLines( &reference_lines, &segments )) {
		    AERROR << "Failed to create reference line";
		    return false;
		  }
		  DCHECK_EQ(reference_lines.size(), segments.size());



		  auto forword_limit =  ReferenceLineProvider::LookForwardDistance(vehicle_state_);

		  for (auto &ref_line : reference_lines) {
		    if (!ref_line.Shrink(Vec2d(vehicle_state_.x(), vehicle_state_.y()),
		                         FLAGS_look_backward_distance, forword_limit)) {
		      AERROR << "Fail to shrink reference line.";
		      return false;
		    }
		  }
		  for (auto &seg : segments) {
		    if (!seg.Shrink(Vec2d(vehicle_state_.x(), vehicle_state_.y()),
		                    FLAGS_look_backward_distance, forword_limit)) {
		      AERROR << "Fail to shrink routing segments.";
		      return false;
		    }
		  }

		   // 对于每条ReferenceLine实例化生成一个对应的ReferenceLineInfo
		  // 就是从ReferenceLineProvider中提取无人车短期内的规划路径，如果不了解，可以查看(组件)指引线提供器: ReferenceLineProvider。然后由一条ReferenceLine&&segments、车辆状态和规划起始点生成对应的ReferenceLineInfo
		  reference_line_info_.clear();
		  auto ref_line_iter = reference_lines.begin();
		  auto segments_iter = segments.begin();
		  while (ref_line_iter != reference_lines.end()) {
		    if (segments_iter->StopForDestination()) {
		      is_near_destination_ = true;
		    }
		    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
		                                      *ref_line_iter, *segments_iter);
		    ++ref_line_iter;
		    ++segments_iter;
		  }

		  if (FLAGS_enable_change_lane_decider &&
		      !change_lane_decider_.Apply(&reference_line_info_)) {
		    AERROR << "Failed to apply change lane decider";
		    return false;
		  }

		  if (reference_line_info_.size() == 2) {
		    common::math::Vec2d xy_point(vehicle_state_.x(), vehicle_state_.y());
		    common::SLPoint first_sl;
		    if (!reference_line_info_.front().reference_line().XYToSL(xy_point,
		                                                              &first_sl)) {
		      return false;
		    }
		    common::SLPoint second_sl;
		    if (!reference_line_info_.back().reference_line().XYToSL(xy_point,
		                                                             &second_sl)) {
		      return false;
		    }
		    const double offset = first_sl.l() - second_sl.l();
		    reference_line_info_.front().SetOffsetToOtherReferenceLine(offset);
		    reference_line_info_.back().SetOffsetToOtherReferenceLine(-offset);
		  }

		  // Step B RerfenceLineInfo初始化 // 2 初始化ReferenceLineInfo类
		  bool has_valid_reference_line = false;
		  for (auto &ref_info : reference_line_info_) {
		    if (!ref_info.Init(obstacles())) {
		      AERROR << "Failed to init reference line";
		      continue;
		    } else {
		      has_valid_reference_line = true;
		    }
		  }
		  return has_valid_reference_line;
		}


/// file in apollo/modules/planning/common/reference_line_info.cc
ReferenceLineInfo::ReferenceLineInfo(const common::VehicleState& vehicle_state,
                                     const TrajectoryPoint& adc_planning_point,
                                     const ReferenceLine& reference_line,
                                     const hdmap::RouteSegments& segments)
    : vehicle_state_(vehicle_state),
      adc_planning_point_(adc_planning_point),
      reference_line_(reference_line),
      lanes_(segments) {}







// 2. 初始化ReferenceLineInfo类
		/apollo/modules/planning/common/reference_line_info.cc

		bool ReferenceLineInfo::Init(const std::vector<const Obstacle*>& obstacles) {
		  const auto& param = VehicleConfigHelper::GetConfig().vehicle_param();
		  const auto& path_point = adc_planning_point_.path_point();
		  Vec2d position(path_point.x(), path_point.y());
		  Vec2d vec_to_center(
		      (param.front_edge_to_center() - param.back_edge_to_center()) / 2.0,
		      (param.left_edge_to_center() - param.right_edge_to_center()) / 2.0);
		  Vec2d center(position + vec_to_center.rotate(path_point.theta()));
		  Box2d box(center, path_point.theta(), param.length(), param.width());
		  if (!reference_line_.GetSLBoundary(box, &adc_sl_boundary_)) {
		    AERROR << "Failed to get ADC boundary from box: " << box.DebugString();
		    return false;
		  }

		  // 检查无人车是否在参考线上  //需要满足无人车对应的边界框start_s和end_s在参考线[0, total_length]区间内
		  if (adc_sl_boundary_.end_s() < 0 ||
		      adc_sl_boundary_.start_s() > reference_line_.Length()) {
		    AWARN << "Vehicle SL " << adc_sl_boundary_.ShortDebugString()
		          << " is not on reference line:[0, " << reference_line_.Length()
		          << "]";
		  }

		  // 检查无人车是否离参考线过远  需要满足无人车start_l和end_l在[-kOutOfReferenceLineL, kOutOfReferenceLineL]区间内，其中kOutOfReferenceLineL取10
		  constexpr double kOutOfReferenceLineL = 10.0;  // in meters
		  if (adc_sl_boundary_.start_l() > kOutOfReferenceLineL ||
		      adc_sl_boundary_.end_l() < -kOutOfReferenceLineL) {
		    AERROR << "Ego vehicle is too far away from reference line.";
		    return false;
		  }
		  is_on_reference_line_ = reference_line_.IsOnRoad(adc_sl_boundary_);

		  // 将障碍物信息加入到ReferenceLineInfo类中
		  if (!AddObstacles(obstacles)) {
		    AERROR << "Failed to add obstacles to reference line";
		    return false;
		  }

		  if (hdmap::GetSpeedControls()) {
		    auto* speed_controls = hdmap::GetSpeedControls();
		    for (const auto& speed_control : speed_controls->speed_control()) {
		      reference_line_.AddSpeedLimit(speed_control);
		    }
		  }

		  // set lattice planning target speed limit;
		  SetCruiseSpeed(FLAGS_default_cruise_speed);
		  is_safe_to_change_lane_ = CheckChangeLane();
		  is_inited_ = true;
		  return true;
		}



		除了将障碍物信息加入到类中，还有一个重要的工作就是确定某个时间点无人车能前进到的位置,
		可以看到这个过程其实就是根据障碍物的轨迹(某个相对时间点，障碍物运动到哪个坐标位置)，并结合无人车查询得到的理想路径，得到某个时间点low_t和high_t无人车行驶距离的下界low_s-adc_start_s和上界high_s-adc_start_s		

		/// file in apollo/modules/planning/common/reference_line_info.cc
		// AddObstacle is thread safe
		PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
		  if (!obstacle) {
		    AERROR << "The provided obstacle is empty";
		    return nullptr;
		  }

		   // 封装成PathObstacle并加入PathDecision
		  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));
		  if (!path_obstacle) {
		    AERROR << "failed to add obstacle " << obstacle->Id();
		    return nullptr;
		  }

		  // 计算障碍物框的start_s, end_s, start_l和end_l
		  SLBoundary perception_sl;
		  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(), &perception_sl)) {
		    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
		    return path_obstacle;
		  }
		  path_obstacle->SetPerceptionSlBoundary(perception_sl);


		   // 计算障碍物是否对无人车行驶有影响：无关障碍物满足以下条件：
  //    1. 障碍物在ReferenceLine以外，忽略
  //    2. 车辆和障碍物都在车道上，但是障碍物在无人车后面，忽略
		  if (IsUnrelaventObstacle(path_obstacle)) {
		  	// 忽略障碍物
		    ObjectDecisionType ignore;
		    ignore.mutable_ignore();
		    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
		                                      ignore);
		    path_decision_.AddLongitudinalDecision("reference_line_filter",
		                                           obstacle->Id(), ignore);
		    ADEBUG << "NO build reference line st boundary. id:" << obstacle->Id();
		  } else {
		  	// 构建障碍物在参考线上的边界框// 构建障碍物在参考线上的边界框
		    ADEBUG << "build reference line st boundary. id:" << obstacle->Id();
		    path_obstacle->BuildReferenceLineStBoundary(reference_line_,
		                                                adc_sl_boundary_.start_s());

		    ADEBUG << "reference line st boundary: "
		           << path_obstacle->reference_line_st_boundary().min_t() << ", "
		           << path_obstacle->reference_line_st_boundary().max_t()
		           << ", s_max: " << path_obstacle->reference_line_st_boundary().max_s()
		           << ", s_min: "
		           << path_obstacle->reference_line_st_boundary().min_s();
		  }
		  return path_obstacle;
		}




		// apollo/modules/planning/common/path_obstacle.cc

void PathObstacle::BuildReferenceLineStBoundary(const ReferenceLine& reference_line, const double adc_start_s) {
  const auto& adc_param = VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  const double adc_width = adc_param.width();
  if (obstacle_->IsStatic() || obstacle_->Trajectory().trajectory_point().empty()) {
    std::vector<std::pair<STPoint, STPoint>> point_pairs;
    double start_s = perception_sl_boundary_.start_s();
    double end_s = perception_sl_boundary_.end_s();
    if (end_s - start_s < kStBoundaryDeltaS) {
      end_s = start_s + kStBoundaryDeltaS;
    }
    if (!reference_line.IsBlockRoad(obstacle_->PerceptionBoundingBox(),
                                    adc_width)) {
      return;
    }
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, 0.0),
                             STPoint(end_s - adc_start_s, 0.0));
    point_pairs.emplace_back(STPoint(start_s - adc_start_s, FLAGS_st_max_t),
                             STPoint(end_s - adc_start_s, FLAGS_st_max_t));
    reference_line_st_boundary_ = StBoundary(point_pairs);
  } else {
    if (BuildTrajectoryStBoundary(reference_line, adc_start_s,
                                  &reference_line_st_boundary_)) {
      ADEBUG << "Found st_boundary for obstacle " << id_;
      ADEBUG << "st_boundary: min_t = " << reference_line_st_boundary_.min_t()
             << ", max_t = " << reference_line_st_boundary_.max_t()
             << ", min_s = " << reference_line_st_boundary_.min_s()
             << ", max_s = " << reference_line_st_boundary_.max_s();
    } else {
      ADEBUG << "No st_boundary for obstacle " << id_;
    }
  }
}



	// apollo/modules/planning/common/path_obstacle.cc
	bool PathObstacle::BuildTrajectoryStBoundary(
    const ReferenceLine& reference_line, const double adc_start_s,
    StBoundary* const st_boundary) {
  const auto& object_id = obstacle_->Id();
  const auto& perception = obstacle_->Perception();
  if (!IsValidObstacle(perception)) {
    AERROR << "Fail to build trajectory st boundary because object is not "
              "valid. PerceptionObstacle: "
           << perception.DebugString();
    return false;
  }
  const double object_width = perception.width();
  const double object_length = perception.length();
  const auto& trajectory_points = obstacle_->Trajectory().trajectory_point();
  if (trajectory_points.empty()) {
    AWARN << "object " << object_id << " has no trajectory points";
    return false;
  }
  const auto& adc_param =
      VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  const double adc_length = adc_param.length();
  const double adc_half_length = adc_length / 2.0;
  const double adc_width = adc_param.width();
  common::math::Box2d min_box({0, 0}, 1.0, 1.0, 1.0);
  common::math::Box2d max_box({0, 0}, 1.0, 1.0, 1.0);
  std::vector<std::pair<STPoint, STPoint>> polygon_points;

  SLBoundary last_sl_boundary;
  int last_index = 0;
   //Step 1. 首先还是对障碍物轨迹点两两选择，每两个点可以构建上图中的 object_moving_box 以及object_boundary。


  for (int i = 1; i < trajectory_points.size(); ++i) {
    ADEBUG << "last_sl_boundary: " << last_sl_boundary.ShortDebugString();

    const auto& first_traj_point = trajectory_points[i - 1];
    const auto& second_traj_point = trajectory_points[i];
    const auto& first_point = first_traj_point.path_point();
    const auto& second_point = second_traj_point.path_point();

    double total_length =
        object_length + common::util::DistanceXY(first_point, second_point);

    common::math::Vec2d center((first_point.x() + second_point.x()) / 2.0,
                               (first_point.y() + second_point.y()) / 2.0);
    common::math::Box2d object_moving_box(center, first_point.theta(),
                                          total_length, object_width);
    SLBoundary object_boundary;
    // NOTICE: this method will have errors when the reference line is not
    // straight. Need double loop to cover all corner cases.
    const double distance_xy =
        common::util::DistanceXY(trajectory_points[last_index].path_point(),
                                 trajectory_points[i].path_point());
    if (last_sl_boundary.start_l() > distance_xy ||
        last_sl_boundary.end_l() < -distance_xy) {
      continue;
    }

    const double mid_s =
        (last_sl_boundary.start_s() + last_sl_boundary.end_s()) / 2.0;
    const double start_s = std::fmax(0.0, mid_s - 2.0 * distance_xy);
    const double end_s = (i == 1) ? reference_line.Length()
                                  : std::fmin(reference_line.Length(),
                                              mid_s + 2.0 * distance_xy);
     // 计算object_boundary，由object_moving_box旋转一个heading得到，记录障碍物形式段的start_s, end_s, start_l和end_l
    if (!reference_line.GetApproximateSLBoundary(object_moving_box, start_s,
                                                 end_s, &object_boundary)) {
      AERROR << "failed to calculate boundary";
      return false;
    }

    // update history record
    last_sl_boundary = object_boundary;
    last_index = i;

//Step 2. 判断障碍物和车辆水平Lateral距离，如果障碍物在参考线两侧，那么障碍物可以忽略；如果障碍物在参考线后面，也可忽略
    // skip if object is entirely on one side of reference line.
    constexpr double kSkipLDistanceFactor = 0.4;
    const double skip_l_distance =
        (object_boundary.end_s() - object_boundary.start_s()) *
            kSkipLDistanceFactor +
        adc_width / 2.0;

    if (std::fmin(object_boundary.start_l(), object_boundary.end_l()) >   // 障碍物在参考线左侧，那么无人车可以直接通过障碍物，可忽略障碍物
            skip_l_distance ||
        std::fmax(object_boundary.start_l(), object_boundary.end_l()) <      // 障碍物在参考线右侧，那么无人车可以直接通过障碍物，可忽略障碍物
            -skip_l_distance) {
      continue;
    }

    if (object_boundary.end_s() < 0) {  // skip if behind reference line  // 障碍物在参考线后面，可忽略障碍物
      continue;
    }
    constexpr double kSparseMappingS = 20.0;
    const double st_boundary_delta_s =
        (std::fabs(object_boundary.start_s() - adc_start_s) > kSparseMappingS)
            ? kStBoundarySparseDeltaS
            : kStBoundaryDeltaS;
    const double object_s_diff =
        object_boundary.end_s() - object_boundary.start_s();
    if (object_s_diff < st_boundary_delta_s) {
      continue;
    }


    // Step 3. 计算low_t和high_t时刻的行驶上下界边界框
    const double delta_t =
        second_traj_point.relative_time() - first_traj_point.relative_time();
    double low_s = std::max(object_boundary.start_s() - adc_half_length, 0.0);
    bool has_low = false;
    double high_s =
        std::min(object_boundary.end_s() + adc_half_length, FLAGS_st_max_s);
    bool has_high = false;
    while (low_s + st_boundary_delta_s < high_s && !(has_low && has_high)) {
      if (!has_low) {  //// 采用渐进逼近的方法，逐渐计算边界框的下界
        auto low_ref = reference_line.GetReferencePoint(low_s);
        has_low = object_moving_box.HasOverlap(
            {low_ref, low_ref.heading(), adc_length, adc_width});
        low_s += st_boundary_delta_s;
      }
      if (!has_high) {   // 采用渐进逼近的方法，逐渐计算边界框的上界
        auto high_ref = reference_line.GetReferencePoint(high_s);
        has_high = object_moving_box.HasOverlap(
            {high_ref, high_ref.heading(), adc_length, adc_width});
        high_s -= st_boundary_delta_s;
      }
    }
    if (has_low && has_high) {
      low_s -= st_boundary_delta_s;
      high_s += st_boundary_delta_s;
      double low_t =
          (first_traj_point.relative_time() +
           std::fabs((low_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      polygon_points.emplace_back(   // 计算low_t时刻的上下界
          std::make_pair(STPoint{low_s - adc_start_s, low_t},
                         STPoint{high_s - adc_start_s, low_t}));
      double high_t =
          (first_traj_point.relative_time() +
           std::fabs((high_s - object_boundary.start_s()) / object_s_diff) *
               delta_t);
      if (high_t - low_t > 0.05) {
        polygon_points.emplace_back(   // 计算high_t时刻的上下界
            std::make_pair(STPoint{low_s - adc_start_s, high_t},
                           STPoint{high_s - adc_start_s, high_t}));
      }
    }
  }

  // Step 4. 计算完所有障碍物轨迹段的上下界框以后，根据时间t进行排序
  if (!polygon_points.empty()) {
    std::sort(polygon_points.begin(), polygon_points.end(),
              [](const std::pair<STPoint, STPoint>& a,
                 const std::pair<STPoint, STPoint>& b) {
                return a.first.t() < b.first.t();
              });
    auto last = std::unique(polygon_points.begin(), polygon_points.end(),
                            [](const std::pair<STPoint, STPoint>& a,
                               const std::pair<STPoint, STPoint>& b) {
                              return std::fabs(a.first.t() - b.first.t()) <
                                     kStBoundaryDeltaT;
                            });
    polygon_points.erase(last, polygon_points.end());
    if (polygon_points.size() > 2) {
      *st_boundary = StBoundary(polygon_points);
    }
  } else {
    return false;
  }
  return true;
}


总结一下无人车参考线ReferenceLineInof初始化(加入障碍物轨迹信息)**这步的功能，给定了无人车的规划轨迹ReferenceLine和障碍物的预测轨迹PredictionObstacles，
这个过程其实就是计算障碍物在无人车规划轨迹上的重叠部分的位置s，以及驶到这个重叠部分的时间点t，为第三部分每个障碍物在每个车辆上的初步决策进行规划。

给出了每个障碍物在所有的参考线上的重叠部分的位置与时间点，这些重叠部分就是无人车需要考虑的规划矫正问题，防止交通事故。因为障碍物运动可能跨越多个ReferenceLine，所以这里需要对于每个障碍物进行标定，是否可忽略，是否会超车等




4. 依据交通规则对障碍物设定标签

	交通规则判定情况一共11种，在文件modules/planning/conf/traffic_rule_config.pb.txt中定义

	后车情况处理--BACKSIDE_VEHICLE
	变道情况处理--CHANGE_LANE
	人行横道情况处理--CROSSWALK
	目的地情况处理--DESTINATION
	前车情况处理--FRONT_VEHICLE
	禁停区情况处理--KEEP_CLEAR
	寻找停车点状态--PULL_OVER
	参考线结束情况处理--REFERENCE_LINE_END
	重新路由查询情况处理--REROUTING
	信号灯情况处理--SIGNAL_LIGHT
	停车情况处理--STOP_SIGN


	// apollo/modules/planning/planning.cc
	void Planning::RunOnce() {
  const uint32_t frame_num = AdapterManager::GetPlanning()->GetSeqNum() + 1;
  status = InitFrame(frame_num, stitching_trajectory.back(), start_timestamp, vehicle_state);

  for (auto& ref_line_info : frame_->reference_line_info()) {
    TrafficDecider traffic_decider;
    traffic_decider.Init(traffic_rule_configs_);
    auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);
    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
      ref_line_info.SetDrivable(false);
      continue;
    }
  }
}



// apollo/modules/planning/tasks/traffic_decider/traffic_decider.cc


Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  for (const auto &rule_config : rule_configs_.config()) {		// 对于每条参考线进行障碍物的决策。
    if (!rule_config.enabled()) {
      ADEBUG << "Rule " << rule_config.rule_id() << " not enabled";
      continue;
    }
    auto rule = s_rule_factory.CreateObject(rule_config.rule_id(), rule_config);
    if (!rule) {
      AERROR << "Could not find rule " << rule_config.DebugString();
      continue;
    }
    rule->ApplyRule(frame, reference_line_info);
    ADEBUG << "Applied rule "
           << TrafficRuleConfig::RuleId_Name(rule_config.rule_id());
  }

  // Creeper::instance()->Run(frame, reference_line_info);

  BuildPlanningTarget(reference_line_info);
  return Status::OK();
}




4.1 // 后车情况处理--BACKSIDE_VEHICLE

apollo/modules/planning/tasks/traffic_decider/backside_vehicle.cc

Status BacksideVehicle::ApplyRule( 
    Frame* const, ReferenceLineInfo* const reference_line_info) {
  auto* path_decision = reference_line_info->path_decision();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  if (reference_line_info->Lanes()
          .IsOnSegment()) {  // The lane keeping reference line.
    MakeLaneKeepingObstacleDecision(adc_sl_boundary, path_decision);
  }
  return Status::OK();
}





void BacksideVehicle::MakeLaneKeepingObstacleDecision(
    const SLBoundary& adc_sl_boundary, PathDecision* path_decision) {
  ObjectDecisionType ignore;  // 从这里可以看到，对于后车的处理主要是忽略
  ignore.mutable_ignore();
  const double adc_length_s =
      adc_sl_boundary.end_s() - adc_sl_boundary.start_s();  // 计算"车长""
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {   // 对于每个与参考线有重叠的障碍物进行规则设置
    if (path_obstacle->PerceptionSLBoundary().end_s() >=
        adc_sl_boundary.end_s()) {  // don't ignore such vehicles.   前车在这里不做考虑，由前车情况处理FRONT_VEHICLE来完成
      continue;
    }

    // 考线上没有障碍物运动轨迹，直接忽略
    if (path_obstacle->reference_line_st_boundary().IsEmpty()) {
      path_decision->AddLongitudinalDecision("backside_vehicle/no-st-region",
                                             path_obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/no-st-region",
                                        path_obstacle->Id(), ignore);
      continue;
    }

    // 忽略从无人车后面来的车辆
    // Ignore the car comes from back of ADC
    if (path_obstacle->reference_line_st_boundary().min_s() < -adc_length_s) {
    	// 通过计算min_s，也就是障碍物轨迹距离无人车最近的距离，小于半车长度，说明车辆在无人车后面，可暂时忽略。
      path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc",
                                             path_obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc",
                                        path_obstacle->Id(), ignore);
      continue;
    }


    //忽略后面不会超车的车辆  n// 4m
    const double lane_boundary =
        config_.backside_vehicle().backside_lane_width();
    if (path_obstacle->PerceptionSLBoundary().start_s() <
        adc_sl_boundary.end_s()) {
      if (path_obstacle->PerceptionSLBoundary().start_l() > lane_boundary ||
          path_obstacle->PerceptionSLBoundary().end_l() < -lane_boundary) {
        continue;
      }
      path_decision->AddLongitudinalDecision("backside_vehicle/sl < adc.end_s",
                                             path_obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/sl < adc.end_s",
                                        path_obstacle->Id(), ignore);
      continue;
    }
  }
}



从代码中可以看到，第一个if可以选择那些在无人车后(至少不超过无人车)的车辆，第二个if，可以计算车辆与无人车的横向距离，如果超过阈值，那么就说明车辆横向距离无人车足够远，有可能会进行超车，这种情况不能忽略，留到后面的交通规则去处理；
若小于这个阈值，则可以间接说明不太会超车，后车可能跟随无人车前进。



4.2 变道情况处理--CHANGE_LANE

在变道情况下第一步要找到那些需要警惕的障碍物(包括跟随这辆和超车车辆)，这些障碍物轨迹是会影响无人车的变道轨迹，然后设置每个障碍物设立一个超车的警示(障碍物和无人车的位置与速度等信息)供下一步规划阶段参考。
Apollo对于每条参考线每次只考虑距离无人车最近的能影响变道的障碍物，同时设置那些超车的障碍物。

// apollo/modules/planning/tasks/traffic_decider/change_lane.cc

Status ChangeLane::ApplyRule(Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  // The reference line is not a change lane reference line, skip
	//// 如果是直行道，不需要变道，则忽略
  if (reference_line_info->Lanes().IsOnSegment()) {
    return Status::OK();
  }
  guard_obstacles_.clear();
  overtake_obstacles_.clear();
  if (!FilterObstacles(reference_line_info)) {
    return Status(common::PLANNING_ERROR, "Failed to filter obstacles");
  }
  //// 创建警示障碍物类
  if (config_.change_lane().enable_guard_obstacle() &&
      !guard_obstacles_.empty()) {
    for (const auto path_obstacle : guard_obstacles_) {
      auto* guard_obstacle = frame->Find(path_obstacle->Id());
      if (guard_obstacle &&
          CreateGuardObstacle(reference_line_info, guard_obstacle)) {
        AINFO << "Created guard obstacle: " << guard_obstacle->Id();
      }
    }
  }


    // 设置超车标志
  if (!overtake_obstacles_.empty()) {
    auto* path_decision = reference_line_info->path_decision();
    const auto& reference_line = reference_line_info->reference_line();
    for (const auto* path_obstacle : overtake_obstacles_) {
      auto overtake = CreateOvertakeDecision(reference_line, path_obstacle);
      path_decision->AddLongitudinalDecision(
          TrafficRuleConfig::RuleId_Name(Id()), path_obstacle->Id(), overtake);
    }
  }
  return Status::OK();
}



bool ChangeLane::FilterObstacles(ReferenceLineInfo* reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();
  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
  const auto& path_decision = reference_line_info->path_decision();
  const PathObstacle* first_guard_vehicle = nullptr;
  constexpr double kGuardForwardDistance = 60;
  double max_s = 0.0;
  const double min_overtake_time = config_.change_lane().min_overtake_time();
  //超车&警示障碍物计算
  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
    const auto* obstacle = path_obstacle->obstacle();
    //没有轨迹的障碍物忽略
    if (!obstacle->HasTrajectory()) {
      continue;
    }
    // 无人车前方的车辆忽略，对变道没有影响
    if (path_obstacle->PerceptionSLBoundary().start_s() >
        adc_sl_boundary.end_s()) {
      continue;
    }

    //跟车在一定距离(10m)内，将其标记为超车障碍物
    if (path_obstacle->PerceptionSLBoundary().end_s() <
        adc_sl_boundary.start_s() -
            std::max(config_.change_lane().min_overtake_distance(),
                     obstacle->Speed() * min_overtake_time)) {
      overtake_obstacles_.push_back(path_obstacle);
    }

    // 障碍物速度很小或者障碍物最后不在参考线上，对变道没影响，可忽略
    const auto& last_point =
        *(obstacle->Trajectory().trajectory_point().rbegin());
    if (last_point.v() < config_.change_lane().min_guard_speed()) {
      continue;
    }
    if (!reference_line.IsOnRoad(last_point.path_point())) {
      continue;
    }

    //障碍物最后一规划点在参考线上，但是超过了无人车一定距离，对变道无影响
    SLPoint last_sl;
    if (!reference_line.XYToSL(last_point.path_point(), &last_sl)) {
      continue;
    }
    if (last_sl.s() < 0 ||
        last_sl.s() > adc_sl_boundary.end_s() + kGuardForwardDistance) {
      continue;
    }
    if (last_sl.s() > max_s) {
      max_s = last_sl.s();
      first_guard_vehicle = path_obstacle;
    }
  }
  if (first_guard_vehicle) {
    guard_obstacles_.push_back(first_guard_vehicle);
  }
  return true;
}




// 创建警示障碍物类

创建警示障碍物类本质其实就是预测障碍物未来一段时间内的运动轨迹，代码在ChangeLane::CreateGuardObstacle中很明显的给出了障碍物轨迹的预测方法。预测的轨迹是在原始轨迹上进行拼接，即在最后一个轨迹点后面再次进行预测，这次预测的假设是，障碍物沿着参考线形式。


bool ChangeLane::CreateGuardObstacle(
    const ReferenceLineInfo* reference_line_info, Obstacle* obstacle) {
  if (!obstacle || !obstacle->HasTrajectory()) {
    return false;
  }
  const auto& last_point =
      *(obstacle->Trajectory().trajectory_point().rbegin());

  const double kStepDistance = obstacle->PerceptionBoundingBox().length();
  double extend_v =
      std::max(last_point.v(), config_.change_lane().min_guard_speed());
  const double time_delta = kStepDistance / extend_v;  			//预测频率: 两个点之间的距离为障碍物长度，所以两个点之间的相对时间差为：time_delta = kStepDistance / extend_v
  const auto& reference_line = reference_line_info->reference_line();
  const double end_s = std::min(reference_line.Length(),
                                reference_line_info->AdcSlBoundary().end_s() +
                                    config_.change_lane().guard_distance());    //预测长度：障碍物预测轨迹重点到无人车前方100m(config_.change_lane().guard_distance())这段距离
  SLPoint sl_point;
  if (!reference_line.XYToSL(last_point.path_point(), &sl_point)) {
    return false;
  }
  double s = last_point.path_point().s() + kStepDistance;
  double ref_s = sl_point.s() + kStepDistance;
  for (double t = last_point.relative_time() + time_delta; ref_s < end_s;
       ref_s += kStepDistance, s += kStepDistance, t += time_delta) {
    auto ref_point = reference_line.GetNearestReferencePoint(ref_s);

    Vec2d xy_point;
    if (!reference_line.SLToXY(common::util::MakeSLPoint(ref_s, sl_point.l()),
                               &xy_point)) {
      return false;
    }

    auto* tp = obstacle->AddTrajectoryPoint();
    tp->set_a(0.0);
    tp->set_v(extend_v);		//障碍物速度假定：这段距离内，默认障碍物速度和最后一个轨迹点速度一致extend_v，并且验证参考线前进
    tp->set_relative_time(t);
    tp->mutable_path_point()->set_x(xy_point.x());
    tp->mutable_path_point()->set_y(xy_point.y());
    tp->mutable_path_point()->set_theta(ref_point.heading());

    // this is an approximate estimate since we do not use it.
    tp->mutable_path_point()->set_s(s);
    tp->mutable_path_point()->set_kappa(ref_point.kappa());
  }
  return true;
}



// 创建障碍物超车标签

ObjectDecisionType ChangeLane::CreateOvertakeDecision(
    const ReferenceLine& reference_line,
    const PathObstacle* path_obstacle) const {
  ObjectDecisionType overtake;
  overtake.mutable_overtake();
  const double speed = path_obstacle->obstacle()->Speed();
  double distance = std::max(speed * config_.change_lane().min_overtake_time(),  // 设置变道过程中，障碍物运动距离
                             config_.change_lane().min_overtake_distance());
  overtake.mutable_overtake()->set_distance_s(distance);
  double fence_s = path_obstacle->PerceptionSLBoundary().end_s() + distance;
  auto point = reference_line.GetReferencePoint(fence_s);  // 设置变道完成后，障碍物在参考线上的位置
  overtake.mutable_overtake()->set_time_buffer(
      config_.change_lane().min_overtake_time());  // 设置变道需要的最小时间
  overtake.mutable_overtake()->set_distance_s(distance);    // 设置变道过程中，障碍物前进的距离
  overtake.mutable_overtake()->set_fence_heading(point.heading());
  overtake.mutable_overtake()->mutable_fence_point()->set_x(point.x());  // 设置变道完成后，障碍物的坐标
  overtake.mutable_overtake()->mutable_fence_point()->set_y(point.y());
  overtake.mutable_overtake()->mutable_fence_point()->set_z(0.0);
  return overtake;
}



4.3 //人行横道情况处理--CROSSWALK

	// apollo/modules/planning/traffic_rules/crosswalk.cc

	Status Crosswalk::ApplyRule(Frame* const frame,
                            ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  //  // 检查是否存在人行横道区域
  if (!FindCrosswalks(reference_line_info)) {
    PlanningContext::Instance()->MutablePlanningStatus()->clear_crosswalk();
    return Status::OK();
  }
 // 为每个障碍物做标记，障碍物存在时，无人车应该停车还是直接驶过
  MakeDecisions(frame, reference_line_info);
  return Status::OK();
}




检查在每个人行横道区域内，是否存在障碍物需要无人车停车让行
如果车辆已经驶过人行横道一部分了，那么就忽略，不需要停车

 // skip crosswalk if master vehicle body already passes the stop line
    if (adc_front_edge_s - crosswalk_overlap->end_s >
        config_.crosswalk().min_pass_s_distance()) {
      if (mutable_crosswalk_status->has_crosswalk_id() &&
          mutable_crosswalk_status->crosswalk_id() == crosswalk_id) {
        mutable_crosswalk_status->clear_crosswalk_id();
        mutable_crosswalk_status->clear_stop_time();
      }

      ADEBUG << "SKIP: crosswalk_id[" << crosswalk_id
             << "] crosswalk_overlap_end_s[" << crosswalk_overlap->end_s
             << "] adc_front_edge_s[" << adc_front_edge_s
             << "]. adc_front_edge passes crosswalk_end_s + buffer.";
      continue;
    }


遍历每个人行道区域的障碍物(行人和非机动车)，并将人行横道区域扩展，提高安全性。

如果障碍物不在扩展后的人行横道内，则忽略。(可能在路边或者其他区域)


 // expand crosswalk polygon
  // note: crosswalk expanded area will include sideway area
  Vec2d point(perception_obstacle.position().x(),
              perception_obstacle.position().y());
  const Polygon2d crosswalk_exp_poly =
      crosswalk_ptr->polygon().ExpandByDistance(
          config_.crosswalk().expand_s_distance());
  bool in_expanded_crosswalk = crosswalk_exp_poly.IsPointIn(point);

  if (!in_expanded_crosswalk) {
    ADEBUG << "skip: obstacle_id[" << obstacle_id << "] type["
           << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
           << "]: not in crosswalk expanded area";
    return false;
  }


计算障碍物到参考线的横向距离obstacle_l_distance ，是否在参考线上(附近)is_on_road ，障碍物轨迹是否与参考线相交is_path_cross

如果横向距离大于疏松距离。如果轨迹相交，那么需要停车；反之直接驶过(此时轨迹相交无所谓，因为横向距离比较远)



if (obstacle_l_distance >= config_.crosswalk().stop_loose_l_distance()) {
    // (1) when obstacle_l_distance is big enough(>= loose_l_distance),  //如果横向距离大于疏松距离。如果轨迹相交，那么需要停车；反之直接驶过(此时轨迹相交无所谓，因为横向距离比较远
    //     STOP only if paths crosses
    if (is_path_cross) {
      stop = true;
      ADEBUG << "need_stop(>=l2): obstacle_id[" << obstacle_id << "] type["
             << obstacle_type_name << "] crosswalk_id[" << crosswalk_id << "]";
    }
  } else if (obstacle_l_distance <=
             config_.crosswalk().stop_strict_l_distance()) {
    if (is_on_road) {
      // (2) when l_distance <= strict_l_distance + on_road  //如果横向距离小于紧凑距离。如果障碍物在参考线或者轨迹相交，那么需要停车；反之直接驶过
      //     always STOP
      if (obstacle_sl_point.s() > adc_end_edge_s) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] s[" << obstacle_sl_point.s()
               << "] adc_end_edge_s[ " << adc_end_edge_s << "] crosswalk_id["
               << crosswalk_id << "] ON_ROAD";
      }
    } else {
      // (3) when l_distance <= strict_l_distance
      //     + NOT on_road(i.e. on crosswalk/median etc)
      //     STOP if paths cross
      if (is_path_cross) {
        stop = true;
        ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
               << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
               << "] PATH_CRSOSS";
      } else {
        // (4) when l_distance <= strict_l_distance   //如果横向距离在紧凑距离和疏松距离之间，直接停车
        //     + NOT on_road(i.e. on crosswalk/median etc)
        //     STOP if he pedestrian is moving toward the ego vehicle
        const auto obstacle_v = Vec2d(perception_obstacle.velocity().x(),
                                      perception_obstacle.velocity().y());
        const auto adc_path_point =
            Vec2d(EgoInfo::Instance()->start_point().path_point().x(),
                  EgoInfo::Instance()->start_point().path_point().y());
        const auto ovstacle_position =
            Vec2d(perception_obstacle.position().x(),
                  perception_obstacle.position().y());
        auto obs_to_adc = adc_path_point - ovstacle_position;
        const double kEpsilon = 1e-6;
        if (obstacle_v.InnerProd(obs_to_adc) > kEpsilon) {
          stop = true;
          ADEBUG << "need_stop(<=l1): obstacle_id[" << obstacle_id << "] type["
                 << obstacle_type_name << "] crosswalk_id[" << crosswalk_id
                 << "] MOVING_TOWARD_ADC";
        }
      }
    }
  } else {
    // (4) when l_distance is between loose_l and strict_l
    //     use history decision of this crosswalk to smooth unsteadiness

    // TODO(all): replace this temp implementation
    if (is_path_cross) {
      stop = true;
    }
    ADEBUG << "need_stop(between l1 & l2): obstacle_id[" << obstacle_id
           << "] type[" << obstacle_type_name << "] obstacle_l_distance["
           << obstacle_l_distance << "] crosswalk_id[" << crosswalk_id
           << "] USE_PREVIOUS_DECISION";
  }


干脆直接驶过)。计算加速的公式还是比较简单

0−v2=2as
s为当前到车辆停止驶过的距离，物理公式，由函数 util::GetADCStopDeceleration 完成。


// 对那些影响无人车行驶的障碍物构建虚拟墙障碍物类以及设置停车标签

什么是构建虚拟墙类，其实很简单，单一的障碍物是一个很小的框，那么无人车在行驶过程中必须要与障碍物保持一定的距离，那么只要以障碍物为中心，构建一个长度为0.1，宽度为车道宽度的虚拟墙，只要保证无人车和这个虚拟墙障碍物不相交，就能确保安全。


  // create virtual stop wall
  std::string virtual_obstacle_id =
      CROSSWALK_VO_ID_PREFIX + crosswalk_overlap->object_id;
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, crosswalk_overlap->start_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle[" << virtual_obstacle_id << "]";
    return -1;
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    AERROR << "Failed to create path_obstacle for: " << virtual_obstacle_id;
    return -1;
  }


  虽然看起来函数跳转比较多，但是其实与二中的障碍物PredictionObstacle封装成Obstacle一样，无非是多加了一个区域框Box2d而已。最后就是对这些虚拟墙添加停车标志

  // build stop decision
  const double stop_s =
      crosswalk_overlap->start_s - config_.crosswalk().stop_distance();    // 计算停车位置的累计距离，stop_distance：1m，人行横道前1m处停车
  auto stop_point = reference_line.GetReferencePoint(stop_s);
  double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_CROSSWALK);
  stop_decision->set_distance_s(-config_.crosswalk().stop_distance());
  stop_decision->set_stop_heading(stop_heading); // 设置停车点的角度/方向
  stop_decision->mutable_stop_point()->set_x(stop_point.x());  // 设置停车点的坐标
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  for (auto pedestrian : pedestrians) {
    stop_decision->add_wait_for_obstacle(pedestrian); // 设置促使无人车停车的障碍物id
  }

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);




4.4 // 目的地情况处理--DESTINATION

到达目的地情况下，障碍物促使无人车采取的行动无非是靠边停车或者寻找合适的停车点(距离目的地还有一小段距离，不需要激励停车)。决策规则比较简单：

	1.主要的决策逻辑


// apollo/modules/planning/tasks/traffic_decider/destination.cc

int Destination::BuildStopDecision(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
// 若当前正处于PULL_OVER状态，则继续保持
  auto* planning_state = GetPlanningStatus()->mutable_planning_state();
  if (planning_state->has_pull_over() &&
      planning_state->pull_over().in_pull_over()) {
    PullOver(nullptr);
    ADEBUG << "destination: continue PULL OVER";
    return 0;
  }

// 检查车辆当前是否需要进入ULL_OVER，主要参考和目的地之间的距离以及是否允许使用PULL_OVER
  const auto& routing =
      AdapterManager::GetRoutingResponse()->GetLatestObserved();
  if (routing.routing_request().waypoint_size() < 2) {
    AERROR << "routing_request has no end";
    return -1;
  }

  const auto& routing_end = *routing.routing_request().waypoint().rbegin();
  double dest_lane_s = std::max(
      0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
      config_.destination().stop_distance());

  common::PointENU dest_point;
  if (CheckPullOver(reference_line_info, routing_end.id(),
                    dest_lane_s, &dest_point)) {
    PullOver(&dest_point);
    ADEBUG << "destination: PULL OVER";
  } else {
    Stop(frame, reference_line_info, routing_end.id(), dest_lane_s);
    ADEBUG << "destination: STOP at current lane";
  }

  return 0;
}


	2. CheckPullOver检查机制(Apollo在DESTINATION中不启用PULL_OVER)
	// apollo/modules/planning/tasks/traffic_decider/destination.cc

bool Destination::CheckPullOver(
    ReferenceLineInfo* const reference_line_info,
    const std::string lane_id,
    const double lane_s,
    common::PointENU* dest_point) {
  CHECK_NOTNULL(reference_line_info);

  // 若在目的地情况下不启用PULL_OVER机制，则返回false
  if (!config_.destination().enable_pull_over()) {
    return false;
  }
  // 若目的地不在参考线上，返回false
  const auto dest_lane = HDMapUtil::BaseMapPtr()->GetLaneById(
      hdmap::MakeMapId(lane_id));
  if (!dest_lane) {
    ADEBUG << "Failed to find lane[" << lane_id << "]";
    return false;
  }

  const auto& reference_line = reference_line_info->reference_line();

  // check dest OnRoad
  double dest_lane_s = std::max(
      0.0, lane_s - FLAGS_virtual_stop_wall_length -
      config_.destination().stop_distance());
  *dest_point = dest_lane->GetSmoothPoint(dest_lane_s);
  if (!reference_line.IsOnRoad(*dest_point)) {
    return false;
  }

  // 若无人车与目的地距离太远，则返回false
  // check dest within pull_over_plan_distance
  common::SLPoint dest_sl;
  if (!reference_line.XYToSL({dest_point->x(), dest_point->y()}, &dest_sl)) {
    ADEBUG << "failed to project the dest point to the other reference line";
    return false;
  }
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double distance_to_dest = dest_sl.s() - adc_front_edge_s;
  ADEBUG << "adc_front_edge_s[" << adc_front_edge_s
      << "] distance_to_dest[" << distance_to_dest
      << "] dest_lane[" << lane_id << "] dest_lane_s[" << dest_lane_s << "]";

  if (distance_to_dest > config_.destination().pull_over_plan_distance()) {
    // to far, not sending pull-over yet
    return false;
  }


  return true;
}




	3.障碍物PULL_OVER和STOP标签设定

int Destination::PullOver(common::PointENU* const dest_point) {
  auto* planning_state = GetPlanningStatus()->mutable_planning_state();
  if (!planning_state->has_pull_over() ||
      !planning_state->pull_over().in_pull_over()) {
    planning_state->clear_pull_over();
    auto pull_over = planning_state->mutable_pull_over();
    pull_over->set_in_pull_over(true);
    pull_over->set_reason(PullOverStatus::DESTINATION);
    pull_over->set_status_set_time(Clock::NowInSeconds());

    if (dest_point) {
      pull_over->mutable_inlane_dest_point()->set_x(dest_point->x());
      pull_over->mutable_inlane_dest_point()->set_y(dest_point->y());
    }
  }

  return 0;
}


Stop标签设定和人行横道情况处理中STOP一致，创建虚拟墙，并封装成新的PathObstacle加入该ReferenceLineInfo的PathDecision中。





4.5 //  前车情况处理--FRONT_VEHICLE


	1. 跟车等待机会超车处理
	当前车是运动的，而且超车条件良好，那么就存在超车的可能，这里定义的超车指代的需要无人车调整横向距离以后超车，直接驶过前方障碍物的属于正常行驶，可忽略障碍物。要完成超车行为需要经过4个步骤：

		正常驾驶(DRIVE)
		等待超车(WAIT)
		超车(SIDEPASS)
		正常行驶(DRIVE)
	若距离前过远，那么无人车就处理第一个阶段正常行驶，若距离比较近，但是不满足超车条件，那么无人车将一直跟随前车正常行驶；若距离近但是无量速度很小(堵车)，那么就需要等待；若满足条件了，就立即进入到超车过程，超车完成就又回到正常行驶状态。




Step 1. 检查超车条件--FrontVehicle::FindPassableObstacle
	// apollo/modules/planning/tasks/traffic_decider/front_vehicle.cc

	/**
	 * @brief: a blocked obstacle is a static obstacle being blocked by
	 *         other obstacles or traffic rules => not passable
	 */
	std::string FrontVehicle::FindPassableObstacle(
	    ReferenceLineInfo* const reference_line_info) {
	  CHECK_NOTNULL(reference_line_info);

	  std::string passable_obstacle_id;
	  const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();
	  auto* path_decision = reference_line_info->path_decision();
	  for (const auto* path_obstacle : path_decision->path_obstacles().Items()) {
	    const PerceptionObstacle& perception_obstacle =
	        path_obstacle->obstacle()->Perception();
	    const std::string& obstacle_id = std::to_string(perception_obstacle.id());
	    PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
	    std::string obstacle_type_name =
	        PerceptionObstacle_Type_Name(obstacle_type);

	    // 是否是虚拟或者静态障碍物。若是，那么就需要采取停车策略，超车部分对这些不作处理
	    if (path_obstacle->obstacle()->IsVirtual() || !path_obstacle->obstacle()->IsStatic()) {
	      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name << "] VIRTUAL or NOT STATIC. SKIP";
	      continue;
	    }

	    // 检车障碍物与无人车位置关系，障碍物在无人车后方，直接忽略
	    const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
	    if (obstacle_sl.start_s() <= adc_sl_boundary.end_s()) {
	      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name << "] behind ADC. SKIP";
	      continue;
	    }

	    // 检查横向与纵向距离，若纵向距离过远则可以忽略，依旧进行正常行驶；若侧方距离过大，可直接忽略障碍物，直接正常向前行驶
	    const double side_pass_s_threshold = config_.front_vehicle().side_pass_s_threshold();
	    if (obstacle_sl.start_s() - adc_sl_boundary.end_s() >
	        side_pass_s_threshold) {
	      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
	             << "] outside of s_threshold. SKIP";
	      continue;
	    }

	    const double side_pass_l_threshold =
	        config_.front_vehicle().side_pass_l_threshold();
	    if (obstacle_sl.start_l() > side_pass_l_threshold ||
	        obstacle_sl.end_l() < -side_pass_l_threshold) {
	      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
	             << "] outside of l_threshold. SKIP";
	      continue;
	    }

	    bool is_blocked_by_others = false;
	    for (const auto* other_obstacle : path_decision->path_obstacles().Items()) {
	      if (other_obstacle->Id() == path_obstacle->Id()) {
	        continue;
	      }
	      if (other_obstacle->PerceptionSLBoundary().start_l() >
	              obstacle_sl.end_l() ||
	          other_obstacle->PerceptionSLBoundary().end_l() <
	              obstacle_sl.start_l()) {
	        // not blocking the backside vehicle
	        continue;
	      }

	      double delta_s = other_obstacle->PerceptionSLBoundary().start_s() -
	                       obstacle_sl.end_s();
	      if (delta_s < 0.0 || delta_s > side_pass_s_threshold) {
	        continue;
	      } else {
	        // TODO(All): fixed the segmentation bug for large vehicles, otherwise
	        // the follow line will be problematic.
	        // is_blocked_by_others = true; break;
	      }
	    }
	    if (!is_blocked_by_others) {
	      passable_obstacle_id = path_obstacle->Id();
	      break;
	    }
	  }
	  return passable_obstacle_id;
	}


// FrontVehicle::FindPassableObstacle函数最后会返回第一个找到的限制无人车行驶的障碍物




Step 2. 设定超车各阶段标志--FrontVehicle::ProcessSidePass

若上阶段处于SidePassStatus::UNKNOWN状态，设置为正常行驶

若上阶段处于SidePassStatus::DRIVE状态，并且障碍物阻塞(存在需要超车条件)，那么设置为等待超车；否则前方没有障碍物，继续正常行驶
// apollo/modules/planning/tasks/traffic_decider/front_vehicle.cc

bool FrontVehicle::ProcessSidePass(
    ReferenceLineInfo* const reference_line_info) {
  CHECK_NOTNULL(reference_line_info);

  // find obstacle being blocked, to process SIDEPASS
  std::string passable_obstacle_id = FindPassableObstacle(reference_line_info);

  auto* sidepass_status = GetPlanningStatus()->mutable_side_pass();
  if (!sidepass_status->has_status()) {
    sidepass_status->set_status(SidePassStatus::UNKNOWN);
  }
  auto status = sidepass_status->status();
  ADEBUG << "side_pass status: " << SidePassStatus_Status_Name(status);

  switch (status) {
    case SidePassStatus::UNKNOWN: {
      sidepass_status->set_status(SidePassStatus::DRIVE);
      break;
    }
    case SidePassStatus::DRIVE: {
      constexpr double kAdcStopSpeedThreshold = 0.1;  // unit: m/s
      const auto& adc_planning_point = reference_line_info->AdcPlanningPoint();
      if (!passable_obstacle_id.empty() &&
          adc_planning_point.v() < kAdcStopSpeedThreshold) {  // 前方有障碍物则需要等待
        sidepass_status->set_status(SidePassStatus::WAIT);
        sidepass_status->set_wait_start_time(Clock::NowInSeconds());
      }
      break;
    } 
    //若上阶段处于SidePassStatus::WAIT状态，若前方已无障碍物，设置为正常行驶；否则视情况而定：
//a. 前方已无妨碍物，设置为正常行驶

//b. 前方有障碍物，检查已等待的时间，超过阈值，寻找左右有效车道超车；若无有效车道，则一直堵车等待。
    case SidePassStatus::WAIT: {
      const auto& reference_line = reference_line_info->reference_line();
      const auto& adc_sl_boundary = reference_line_info->AdcSlBoundary();

      if (passable_obstacle_id.empty()) {
        sidepass_status->set_status(SidePassStatus::DRIVE);
        sidepass_status->clear_wait_start_time();
      } else {
        double wait_start_time = sidepass_status->wait_start_time();
        double wait_time = Clock::NowInSeconds() - wait_start_time;// 计算已等待的时间
        ADEBUG << "wait_start_time[" << wait_start_time << "] wait_time["
               << wait_time << "]";

        if (wait_time > config_.front_vehicle().side_pass_wait_time()) { // 超过阈值，寻找其他车道超车。side_pass_wait_time：30s
          // calculate if the left/right lane exist
          std::vector<hdmap::LaneInfoConstPtr> lanes;
          const double adc_s =
              (adc_sl_boundary.start_s() + adc_sl_boundary.end_s()) / 2.0;
          reference_line.GetLaneFromS(adc_s, &lanes);
          if (lanes.empty()) {
            AWARN << "No valid lane found at s[" << adc_s << "]";
            return false;
          }

          bool enter_sidepass_mode = false;
          ObjectSidePass::Type side = ObjectSidePass::LEFT;
          if (lanes.size() >= 2) {
            // currently do not sidepass when lanes > 2 (usually at junctions).
          } else {
            sidepass_status->set_status(SidePassStatus::DRIVE);
            sidepass_status->clear_wait_start_time();

            auto& lane = lanes.front()->lane();
            if (lane.left_neighbor_forward_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::LEFT;
            }
            if (!enter_sidepass_mode &&
                lane.right_neighbor_forward_lane_id_size() > 0) {
              bool has_city_driving = false;
              for (auto& id : lane.right_neighbor_forward_lane_id()) {
                if (HDMapUtil::BaseMap().GetLaneById(id)->lane().type() ==
                    hdmap::Lane::CITY_DRIVING) {
                  has_city_driving = true;
                  break;
                }
              }
              if (has_city_driving) {
                enter_sidepass_mode = true;
                side = ObjectSidePass::RIGHT;
              }
            }
            if (!enter_sidepass_mode &&
                lane.left_neighbor_reverse_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::LEFT;
            }
            if (!enter_sidepass_mode &&
                lane.right_neighbor_reverse_lane_id_size() > 0) {
              enter_sidepass_mode = true;
              side = ObjectSidePass::RIGHT;
            }
          }

         //  首先查询HDMap，计算当前ReferenceLine所在的车道Lane。若存在坐车道，那么设置做超车；若不存在坐车道，那么检查右车道，右车道如果是机动车道(CITY_DRIVING)，不是非机动车道(BIKING)，不是步行街(SIDEWALK)，也不是停车道(PAKING)，那么可以进行右超车。
          if (enter_sidepass_mode) {
            sidepass_status->set_status(SidePassStatus::SIDEPASS);
            sidepass_status->set_pass_obstacle_id(passable_obstacle_id);
            sidepass_status->clear_wait_start_time();
            sidepass_status->set_pass_side(side);// side知识左超车还是右超车。取值为ObjectSidePass::RIGHT或ObjectSidePass::LEFT
          }
        }
      }
      break;
    } //若上阶段处于SidePassStatus::SIDEPASS状态，若前方已无障碍物，设置为正常行驶；反之继续处于超车过程。
    case SidePassStatus::SIDEPASS: {
      if (passable_obstacle_id.empty()) {
        sidepass_status->set_status(SidePassStatus::DRIVE);
      }
      break;
    }
    default:
      break;
  }
  return true;
}

// 停车处理


// apollo/modules/planning/tasks/traffic_decider/pull_over.cc

   if (path_obstacle->obstacle()->IsVirtual() ||
        !path_obstacle->obstacle()->IsStatic()) {  //首先检查障碍物是不是静止物体(非1中的堵车情况)。若是虚拟的或者动态障碍物，则忽略，由超车模块处理
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] VIRTUAL or NOT STATIC. SKIP";
      continue;
    }

// apollo/modules/planning/tasks/traffic_decider/front_vehicle.cc
// 检查障碍物和无人车位置，若障碍物在无人车后，忽略，由后车模块处理
const auto& obstacle_sl = path_obstacle->PerceptionSLBoundary();
    if (obstacle_sl.end_s() <= adc_sl.start_s()) {
      // skip backside vehicles
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] behind ADC. SKIP";
      continue;
    }


// apollo/modules/planning/tasks/traffic_decider/front_vehicle.cc
// 障碍物已经在超车模块中被标记，迫使无人车超车，那么此时就不需要考虑该障碍物
    // check SIDE_PASS decision
    if (path_obstacle->LateralDecision().has_sidepass()) {
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] SIDE_PASS. SKIP";
      continue;
    }


// apollo/modules/planning/tasks/traffic_decider/front_vehicle.cc
//检查是否必须要停车条件，车道没有足够的宽度来允许无人车超车

double left_width = 0.0;
    double right_width = 0.0;
    reference_line.GetLaneWidth(obstacle_sl.start_s(), &left_width,
                                &right_width);

    double left_driving_width = left_width - obstacle_sl.end_l() -
                                config_.front_vehicle().nudge_l_buffer();  // 计算障碍物左侧空余距离
    double right_driving_width = right_width + obstacle_sl.start_l() -
                                 config_.front_vehicle().nudge_l_buffer();  // 计算障碍物右侧空余距离，+号是因为车道线FLU坐标系左侧l是负轴，右侧是正轴

    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "] left_driving_width[" << left_driving_width
           << "] right_driving_width[" << right_driving_width << "] adc_width["
           << adc_width << "]";

    // stop if not able to bypass or if obstacle crossed reference line
    if ((left_driving_width < adc_width && right_driving_width < adc_width) ||
        (obstacle_sl.start_l() <= 0.0 && obstacle_sl.end_l() >= 0.0)) {
      ADEBUG << "STOP: obstacle[" << obstacle_id << "]";

      // build stop decision
      double stop_distance =
          path_obstacle->MinRadiusStopDistance(vehicle_param);
      const double stop_s = obstacle_sl.start_s() - stop_distance;
      auto stop_point = reference_line.GetReferencePoint(stop_s);
      double stop_heading = reference_line.GetReferencePoint(stop_s).heading();

      ObjectDecisionType stop;
      auto stop_decision = stop.mutable_stop();
      if (obstacle_type == PerceptionObstacle::UNKNOWN_MOVABLE ||
          obstacle_type == PerceptionObstacle::BICYCLE ||
          obstacle_type == PerceptionObstacle::VEHICLE) {
        stop_decision->set_reason_code(
            StopReasonCode::STOP_REASON_HEAD_VEHICLE);
      } else {
        stop_decision->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);
      }
      stop_decision->set_distance_s(-stop_distance);
      stop_decision->set_stop_heading(stop_heading);
      stop_decision->mutable_stop_point()->set_x(stop_point.x());
      stop_decision->mutable_stop_point()->set_y(stop_point.y());
      stop_decision->mutable_stop_point()->set_z(0.0);

      path_decision->AddLongitudinalDecision("front_vehicle",
                                             path_obstacle->Id(), stop);









4.6 // 禁停区情况处理--KEEP_CLEAR

禁停区分为两类，第一类是传统的禁停区，第二类是交叉路口。那么对于禁停区做的处理和对于人行横道上障碍物构建虚拟墙很相似。具体做法是在参考线上构建一块禁停区，从纵向的start_s到end_s(这里的start_s和end_s是禁停区start_s和end_s在参考线上的投影点)。禁停区宽度是参考线的道路宽。

具体的处理情况为(禁停区和交叉路口处理一致)：


// apollo/modules/planning/tasks/traffic_decider/keep_clear.cc


bool KeepClear::BuildKeepClearObstacle(
    Frame* const frame, ReferenceLineInfo* const reference_line_info,
    PathOverlap* const keep_clear_overlap,
    const std::string& virtual_obstacle_id) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(keep_clear_overlap);

  // check // 1. 检查无人车是否已经驶入禁停区或者交叉路口，是则可直接忽略
  const double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  if (adc_front_edge_s - keep_clear_overlap->start_s >
      config_.keep_clear().min_pass_s_distance()) {
    ADEBUG << "adc inside keep_clear zone[" << keep_clear_overlap->object_id
           << "] s[" << keep_clear_overlap->start_s << ", "
           << keep_clear_overlap->end_s << "] adc_front_edge_s["
           << adc_front_edge_s << "]. skip this keep clear zone";
    return false;
  }

  // create virtual static obstacle  // 2. 创建新的禁停区障碍物，并且打上标签为不能停车
  auto* obstacle = frame->CreateStaticObstacle(
      reference_line_info, virtual_obstacle_id, keep_clear_overlap->start_s,
      keep_clear_overlap->end_s);
  if (!obstacle) {
    AERROR << "Failed to create obstacle [" << virtual_obstacle_id << "]";
    return false;
  }
  auto* path_obstacle = reference_line_info->AddObstacle(obstacle);
  if (!path_obstacle) {
    AERROR << "Failed to create path_obstacle: " << virtual_obstacle_id;
    return false;
  }
  path_obstacle->SetReferenceLineStBoundaryType(
      StBoundary::BoundaryType::KEEP_CLEAR);

  return true;
}


// apollo/modules/planning/common/frame.cc
这里额外补充一下创建禁停区障碍物流程，主要还是计算禁停区障碍物的标定框，即center和length，width

const Obstacle *Frame::CreateStaticObstacle(
    ReferenceLineInfo *const reference_line_info,
    const std::string &obstacle_id, const double obstacle_start_s,
    const double obstacle_end_s) {
  if (reference_line_info == nullptr) {
    AERROR << "reference_line_info nullptr";
    return nullptr;
  }

  const auto &reference_line = reference_line_info->reference_line();

  // start_xy
  //   // 计算禁停区障碍物start_xy，需要映射到ReferenceLine
  common::SLPoint sl_point;
  sl_point.set_s(obstacle_start_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_start_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_start_xy)) {
    AERROR << "Failed to get start_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }

  // end_xy
  //   // 计算禁停区障碍物end_xy，需要映射到ReferenceLine
  sl_point.set_s(obstacle_end_s);
  sl_point.set_l(0.0);
  common::math::Vec2d obstacle_end_xy;
  if (!reference_line.SLToXY(sl_point, &obstacle_end_xy)) {
    AERROR << "Failed to get end_xy from sl: " << sl_point.DebugString();
    return nullptr;
  }
//   // 计算禁停区障碍物左侧宽度和右侧宽度，与参考线一致
  double left_lane_width = 0.0;
  double right_lane_width = 0.0;
  if (!reference_line.GetLaneWidth(obstacle_start_s, &left_lane_width,
                                   &right_lane_width)) {
    AERROR << "Failed to get lane width at s[" << obstacle_start_s << "]";
    return nullptr;
  }
//   // 最终可以得到禁停区障碍物的标定框
  common::math::Box2d obstacle_box{
      common::math::LineSegment2d(obstacle_start_xy, obstacle_end_xy),
      left_lane_width + right_lane_width};
//  // CreateStaticVirtualObstacle函数是将禁停区障碍物封装成PathObstacle放入PathDecision中
  return CreateStaticVirtualObstacle(obstacle_id, obstacle_box);
}






4.7 // 寻找停车点状态--PULL_OVER

寻找停车点本质上是寻找停车的位置，如果当前已经有停车位置(也就是上个状态就是PULL_OVER)，那么就只需要更新状态信息即可；若不存在，那么就需要计算停车点的位置，然后构建停车区障碍物(同人行横道虚拟墙障碍物和禁停区障碍物)，然后创建障碍物的PULL_OVER标签

// apollo/modules/planning/tasks/traffic_decider/pull_over.cc

Status PullOver::ApplyRule(Frame* const frame,
                           ReferenceLineInfo* const reference_line_info) {
  frame_ = frame;
  reference_line_info_ = reference_line_info;

  if (!IsPullOver()) {
    return Status::OK();
  }
  // 检查上时刻是不是PULL_OVER，如果是PULL_OVER，那么说明已经有停车点stop_point
  if (CheckPullOverComplete()) {
    return Status::OK();
  }

  common::PointENU stop_point;
  if (GetPullOverStop(&stop_point) != 0) { // 获取停车位置失败，无人车将在距离终点的车道上进行停车
    BuildInLaneStop(stop_point);
    ADEBUG << "Could not find a safe pull over point. STOP in-lane";
  } else {
    BuildPullOverStop(stop_point);   // 获取停车位置成功，无人车将在距离终点的车道上靠边进行停车
  }

  return Status::OK();
}


// 1. 获取停车点--GetPullOverStop函数
int PullOver::GetPullOverStop(PointENU* stop_point) {
  auto& pull_over_status =
      GetPlanningStatus()->mutable_planning_state()->pull_over();

  bool found = false;
  bool retry = true;
  if (pull_over_status.has_start_point() && pull_over_status.has_stop_point()) {
    // reuse existing/previously-set stop point
    stop_point->set_x(pull_over_status.stop_point().x());
    stop_point->set_y(pull_over_status.stop_point().y());

    ValidateStopPointCode ret = IsValidStop(*stop_point);
    if (ret == OK) {
      found = true;
    } else {
      if (ret == PASS_DEST_POINT_TOO_FAR) {
        retry = false;
      } else if (failure_count_ < config_.pull_over().max_failure_count()) {
        retry = false;
      }
    }
  }

  if (!found && retry) {
    // finding pull_over_stop_point
    if (FindPullOverStop(stop_point) == 0) {
      found = true;
    }
  }

  // found valid pull_over_stop_point
  if (found) {
    failure_count_ = 0;
    stop_point_.set_x(stop_point->x());
    stop_point_.set_y(stop_point->y());
    return 0;
  }
  // 如果状态信息中已经存在了停车点位置并且有效，那么就可以直接拿来用
  // when fail, use previous invalid stop_point for smoothness
  failure_count_++;
  if (stop_point_.has_x() && stop_point_.has_y() &&
      failure_count_ < config_.pull_over().max_failure_count()) {
    stop_point->set_x(stop_point_.x());
    stop_point->set_y(stop_point_.y());
    return 0;
  }

  // fail to find valid pull_over_stop_point
  stop_point_.Clear();
  return -1;
}



如果不存在，那么就需要寻找停车位置--FindPullOverStop函数
停车位置需要在目的地点前PARKING_SPOT_LONGITUDINAL_BUFFER(默认1m)，距离路测buffer_to_boundary(默认0.5m)处停车。但是停车条件必须满足在路的最边上，也就意味着这条lane的右侧lane不能是机动车道(CITY_DRIVING)。Apollo采用的方法为采样检测，从车头到终点位置，每隔kDistanceUnit(默认5m)进行一次停车条件检查，满足则直接停车。




int PullOver::FindPullOverStop(PointENU* stop_point) {
  const auto& reference_line = reference_line_info_->reference_line();
  const double adc_front_edge_s = reference_line_info_->AdcSlBoundary().end_s();

  double check_length = 0.0;
  double total_check_length = 0.0;
  double check_s = adc_front_edge_s; // check_s为当前车辆车头的累计距离

  constexpr double kDistanceUnit = 5.0;
  while (check_s < reference_line.Length() &&
      total_check_length < config_.pull_over().max_check_distance()) {  // 在当前车道上，向前采样方式进行停车位置检索，前向检索距离不超过max_check_distance(默认60m)
    check_s += kDistanceUnit;
    total_check_length += kDistanceUnit;

    // find next_lane to check
    std::string prev_lane_id;
    std::vector<hdmap::LaneInfoConstPtr> lanes;
    reference_line.GetLaneFromS(check_s, &lanes);
    hdmap::LaneInfoConstPtr lane;
    for (auto temp_lane : lanes) {
      if (temp_lane->lane().id().id() == prev_lane_id) {
        continue;
      }
      lane = temp_lane;
      prev_lane_id = temp_lane->lane().id().id();
      break;
    }

    std::string lane_id = lane->lane().id().id();
    ADEBUG << "check_s[" << check_s << "] lane[" << lane_id << "]";

    // check turn type: NO_TURN/LEFT_TURN/RIGHT_TURN/U_TURN
    const auto& turn = lane->lane().turn();
    if (turn != hdmap::Lane::NO_TURN) {
      ADEBUG << "path lane[" << lane_id << "] turn[" << Lane_LaneTurn_Name(turn)
             << "] can't pull over";
      check_length = 0.0;
      continue;
    }

// 检查该点(check_s)位置右车道，如果右车道还是机动车道，那么改点不能停车，至少需要变道，继续前向搜索。
    // check rightmost driving lane:
    //   NONE/CITY_DRIVING/BIKING/SIDEWALK/PARKING
    bool rightmost_driving_lane = true;
    for (auto& neighbor_lane_id :
         lane->lane().right_neighbor_forward_lane_id()) {
      const auto neighbor_lane =
          HDMapUtil::BaseMapPtr()->GetLaneById(neighbor_lane_id);
      if (!neighbor_lane) {
        ADEBUG << "Failed to find lane[" << neighbor_lane_id.id() << "]";
        continue;
      }
      const auto& lane_type = neighbor_lane->lane().type();
      if (lane_type == hdmap::Lane::CITY_DRIVING) {
        ADEBUG << "lane[" << lane_id << "]'s right neighbor forward lane["
               << neighbor_lane_id.id() << "] type["
               << Lane_LaneType_Name(lane_type) << "] can't pull over";
        rightmost_driving_lane = false;
        break;
      }
    }
    if (!rightmost_driving_lane) {
      check_length = 0.0;
      continue;
    }

    // check if on overlaps
    const auto& vehicle_param =
        VehicleConfigHelper::GetConfig().vehicle_param();
    const double adc_length = vehicle_param.length();
    const double parking_spot_start_s = check_s - adc_length -
        PARKING_SPOT_LONGITUDINAL_BUFFER;
    const double parking_spot_end_s = check_s +
        PARKING_SPOT_LONGITUDINAL_BUFFER;
    if (OnOverlap(parking_spot_start_s, parking_spot_end_s)) {
      ADEBUG << "lane[" << lane_id << "] on overlap.  can't pull over";
      check_length = 0.0;
      continue;
    }


// 如果右侧车道不是机动车道，那么路测允许停车。停车位置需要在目的地点前PARKING_SPOT_LONGITUDINAL_BUFFER(默认1m)，距离路测buffer_to_boundary(默认0.5m)处停车。纵向与停车点的距离以车头为基准；侧方与停车点距离取{车头距离车道边线，车尾距离车道边线，车身中心距离车道边线}距离最小值为基准。
    // all the lane checks have passed
    check_length += kDistanceUnit;
    ADEBUG << "check_length: " << check_length << "; plan_distance:" <<
        config_.pull_over().plan_distance();
    if (check_length >= config_.pull_over().plan_distance()) {
      PointENU point;
      // check corresponding parking_spot
      if (FindPullOverStop(check_s, &point) != 0) {
        // parking_spot not valid/available
        check_length = 0.0;
        continue;
      }

      stop_point->set_x(point.x());
      stop_point->set_y(point.y());
      ADEBUG << "stop point: lane[" << lane->id().id()
          << "] (" << stop_point->x() << ", " << stop_point->y() << ")";

      return 0;
    }
  }

  return -1;
}

// 1. 获取停车点--GetPullOverStop函数
在1中如果找到了停车位置，那么就直接对停车位置构建一个PathObstacle，然后设置他的标签Stop即可。创建停车区障碍物的方式跟上述一样，这里也不重复讲解。该功能由函数BuildPullOverStop完成。

在1中如果找不到停车位置，那么就去寻找历史状态中的数据，找到了就根据2中停车，找不到强行在车道上停车。该功能由函数BuildInLaneStop完成

先去寻找历史数据的inlane_dest_point，也就是历史数据是否允许在车道上停车
如果没找到，那么去寻找停车位置，如果找到了就可以进行2中的停车
如果仍然没找到停车位置，去寻找类中使用过的inlane_adc_potiion_stop_point_，如果找到了可以进行2中的停车
如果依旧没找到那么只能强行在距离终点plan_distance处，在车道上强行停车，并更新inlane_adc_potiion_stop_point_，供下次使用。





4.8 // 参考线结束情况处理--REFERENCE_LINE_END
// 当参考线结束，一般就需要重新路由查询，所以无人车需要停车，这种情况下如果程序正常，一般是前方没有路了，需要重新查询一点到目的地新的路由，具体的代码也是跟人行横道上的不可忽略障碍物一样，在参考线终点前构建一个停止墙障碍物，并设置齐标签为停车Stop。

// apollo/modules/planning/tasks/traffic_decider/reference_line_end.cc


Status ReferenceLineEnd::ApplyRule(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  const auto& reference_line = reference_line_info->reference_line();
  // check
  // 检查参考线剩余的长度，足够则可忽略这个情况，min_reference_line_remain_length：50m
  double remain_s =
      reference_line.Length() - reference_line_info->AdcSlBoundary().end_s();
  if (remain_s >
      config_.reference_line_end().min_reference_line_remain_length()) {
    return Status::OK();
  }

  // create avirtual stop wall at the end of reference line to stop the adc
  // create avirtual stop wall at the end of reference line to stop the adc
  std::string virtual_obstacle_id =
      REF_LINE_END_VO_ID_PREFIX + reference_line_info->Lanes().Id();
  double obstacle_start_s =
      reference_line.Length() - 2 * FLAGS_virtual_stop_wall_length; // 在参考线终点前，创建停止墙障碍物
  auto* obstacle = frame->CreateStopObstacle(
      reference_line_info, virtual_obstacle_id, obstacle_start_s);
  if (!obstacle) {
    return Status(common::PLANNING_ERROR,
                  "Failed to create reference line end obstacle");
  }
  PathObstacle* stop_wall = reference_line_info->AddObstacle(obstacle);
  if (!stop_wall) {
    return Status(
        common::PLANNING_ERROR,
        "Failed to create path obstacle for reference line end obstacle");
  }

  // build stop decision   // build stop decision，设置障碍物停止标签
  const double stop_line_s =
      obstacle_start_s - config_.reference_line_end().stop_distance();
  auto stop_point = reference_line.GetReferencePoint(stop_line_s);

  ObjectDecisionType stop;
  auto stop_decision = stop.mutable_stop();
  stop_decision->set_reason_code(StopReasonCode::STOP_REASON_DESTINATION);
  stop_decision->set_distance_s(-config_.reference_line_end().stop_distance());
  stop_decision->set_stop_heading(stop_point.heading());
  stop_decision->mutable_stop_point()->set_x(stop_point.x());
  stop_decision->mutable_stop_point()->set_y(stop_point.y());
  stop_decision->mutable_stop_point()->set_z(0.0);

  auto* path_decision = reference_line_info->path_decision();
  path_decision->AddLongitudinalDecision(
      TrafficRuleConfig::RuleId_Name(config_.rule_id()), stop_wall->Id(), stop);

  return Status::OK();
}









4.9 //重新路由查询情况处理--REROUTING
根据具体路况进行处理，根据代码可以分为以下情况：

若当前参考线为直行，非转弯。那么暂时就不需要重新路由，等待新的路由
若当前车辆不在参考线上，不需要重新路由，等待新的路由
若当前车辆可以退出了，不需要重新路由，等待新的路由
若当前通道Passage终点不在参考线上，不需要重新路由，等待新的路由
若参考线的终点距离无人车过远，不需要重新路由，等待新的路由
若上时刻进行过路由查询，距离当前时间过短，不需要重新路由，等待新的路由
其他情况，手动发起路由查询需求
其实这个模块我还是有点不是特别敢肯定，只能做保留的解释。首先代码Frame::Rerouting做的工作仅仅重新路由，得到当前位置到目的地的一个路况，这个过程并没有产生新的参考线，因为参考线的产生依赖于ReferenceLineProvider线程。所以说对于第二点，车辆不在参考线上，即使重新路由了，但是没有生成矫正的新参考线，所以重新路由也是无用功，反之还不如等待ReferenceLineProvider去申请重新路由并生成对应的参考线。所以说2,3,4等情况的重点在于缺乏参考线，而不在于位置偏离了。


// apollo/modules/planning/tasks/traffic_decider/rerouting.cc

bool Rerouting::ChangeLaneFailRerouting() {
  const auto& segments = reference_line_info_->Lanes();
  // 1. If current reference line is drive forward, no rerouting.
  if (segments.NextAction() == routing::FORWARD) {
    // if not current lane, do not check for rerouting
    return true;
  }

  // 2. If vehicle is not on current reference line yet, no rerouting
  if (!segments.IsOnSegment()) {
    return true;
  }

  // 3. If current reference line can connect to next passage, no rerouting
  if (segments.CanExit()) {
    return true;
  }

  // 4. If the end of current passage region not appeared, no rerouting
  const auto& route_end_waypoint = segments.RouteEndWaypoint();
  if (!route_end_waypoint.lane) {
    return true;
  }
  auto point = route_end_waypoint.lane->GetSmoothPoint(route_end_waypoint.s);
  const auto& reference_line = reference_line_info_->reference_line();
  common::SLPoint sl_point;
  if (!reference_line.XYToSL({point.x(), point.y()}, &sl_point)) {
    AERROR << "Failed to project point: " << point.ShortDebugString();
    return false;
  }
  if (!reference_line.IsOnRoad(sl_point)) {
    return true;
  }
  // 5. If the end of current passage region is further than kPrepareRoutingTime
  // * speed, no rerouting
  double adc_s = reference_line_info_->AdcSlBoundary().end_s();
  const auto* vehicle_state = common::VehicleStateProvider::instance();
  double speed = vehicle_state->linear_velocity();
  const double prepare_rerouting_time =
      config_.rerouting().prepare_rerouting_time();
  const double prepare_distance = speed * prepare_rerouting_time;
  if (sl_point.s() > adc_s + prepare_distance) {
    ADEBUG << "No need rerouting now because still can drive for time: "
           << prepare_rerouting_time << " seconds";
    return true;
  }
  // 6. Check if we have done rerouting before
  auto* rerouting = GetPlanningStatus()->mutable_rerouting();
  if (rerouting == nullptr) {
    AERROR << "rerouting is nullptr.";
    return false;
  }
  double current_time = Clock::NowInSeconds();
  if (rerouting->has_last_rerouting_time() &&
      (current_time - rerouting->last_rerouting_time() <
       config_.rerouting().cooldown_time())) {
    ADEBUG << "Skip rerouting and wait for previous rerouting result";
    return true;
  }
  if (!frame_->Rerouting()) {
    AERROR << "Failed to send rerouting request";
    return false;
  }

  // store last rerouting time.
  rerouting->set_last_rerouting_time(current_time);
  return true;
}









4.10 // 信号灯情况处理--SIGNAL_LIGHT

信号灯处理相对来说比较简单，无非是有红灯就停车；有黄灯速度小，就停车；有绿灯，或者黄灯速度大就直接驶过。具体的处理步骤分为

// apollo/modules/planning/tasks/traffic_decider/signal_light.cc

// 1. 检查当前路况下是否有信号灯区域--由函数FindValidSignalLight完成
bool SignalLight::FindValidSignalLight(
    ReferenceLineInfo* const reference_line_info) {
  const std::vector<hdmap::PathOverlap>& signal_lights =
      reference_line_info->reference_line().map_path().signal_overlaps();
  if (signal_lights.size() <= 0) {
    ADEBUG << "No signal lights from reference line.";
    return false;
  }
  signal_lights_from_path_.clear();
  for (const hdmap::PathOverlap& signal_light : signal_lights) {
    if (signal_light.start_s + config_.signal_light().min_pass_s_distance() >
        reference_line_info->AdcSlBoundary().end_s()) {
      signal_lights_from_path_.push_back(signal_light);
    }
  }
  return signal_lights_from_path_.size() > 0;
}



// 2. 获取TrafficLight Perception发布的信号等信息--由函数ReadSignals完成
void SignalLight::ReadSignals() {
  detected_signals_.clear();
  if (AdapterManager::GetTrafficLightDetection()->Empty()) {
    return;
  }
  if (AdapterManager::GetTrafficLightDetection()->GetDelaySec() >
      config_.signal_light().signal_expire_time_sec()) {
    ADEBUG << "traffic signals msg is expired: "
           << AdapterManager::GetTrafficLightDetection()->GetDelaySec();
    return;
  }
  const TrafficLightDetection& detection =
      AdapterManager::GetTrafficLightDetection()->GetLatestObserved();
  for (int j = 0; j < detection.traffic_light_size(); j++) {
    const TrafficLight& signal = detection.traffic_light(j);
    detected_signals_[signal.id()] = &signal;
  }
}


// 3. 决策--由函数MakeDecisions完成

void SignalLight::MakeDecisions(Frame* const frame,
                                ReferenceLineInfo* const reference_line_info) {
  planning_internal::SignalLightDebug* signal_light_debug =
      reference_line_info->mutable_debug()
          ->mutable_planning_data()
          ->mutable_signal_light();
  signal_light_debug->set_adc_front_s(
      reference_line_info->AdcSlBoundary().end_s());
  signal_light_debug->set_adc_speed(
      common::VehicleStateProvider::instance()->linear_velocity());

  bool has_stop = false;
  for (auto& signal_light : signal_lights_from_path_) {
    const TrafficLight signal = GetSignal(signal_light.object_id);
    double stop_deceleration = util::GetADCStopDeceleration(
        reference_line_info, signal_light.start_s,
        config_.signal_light().min_pass_s_distance());

    planning_internal::SignalLightDebug::SignalDebug* signal_debug =
        signal_light_debug->add_signal();
    signal_debug->set_adc_stop_deacceleration(stop_deceleration);
    signal_debug->set_color(signal.color());
    signal_debug->set_light_id(signal_light.object_id);
    signal_debug->set_light_stop_s(signal_light.start_s);

    // 1. 如果信号灯是红灯，并且加速度不是很大
    // 2. 如果信号灯是未知，并且加速度不是很大
    // 3. 如果信号灯是黄灯，并且加速度不是很大
    // 以上三种情况，无人车停车，停车标签与前面一致

    if ((signal.color() == TrafficLight::RED &&
         stop_deceleration < config_.signal_light().max_stop_deceleration()) ||
        (signal.color() == TrafficLight::UNKNOWN &&
         stop_deceleration < config_.signal_light().max_stop_deceleration()) ||
        (signal.color() == TrafficLight::YELLOW &&
         stop_deceleration <
             config_.signal_light().max_stop_deacceleration_yellow_light())) {
      if (config_.signal_light().righ_turn_creep().enabled() &&
          reference_line_info->IsRightTurnPath()) {
        SetCreepForwardSignalDecision(reference_line_info, &signal_light);
      }
      if (BuildStopDecision(frame, reference_line_info, &signal_light)) {
        has_stop = true;
        signal_debug->set_is_stop_wall_created(true);
      }
    }
    //  // 设置交叉口区域，以及是否有权力通行，停车表明无法通行。
    if (has_stop) {
      reference_line_info->SetJunctionRightOfWay(signal_light.start_s,
                                                 false);  // not protected
    } else {
      reference_line_info->SetJunctionRightOfWay(signal_light.start_s, true);
      // is protected
    }
  }
}protected
    }













4.11 //  停车情况处理--STOP_SIGN
// apollo/modules/planning/tasks/traffic_decider/stop_sign.cc


停车情况相对来说比较复杂，根据代码将停车分为：寻找下一个最近的停车信号，决策处理。寻找下一个停车点比较简单，由函数FindNextStopSign完成，这里直接跳过。接下来分析决策部分，可以分为以下几步：

// 1. 获取等待车辆列表--由函数GetWatchVehicles完成
int StopSign::GetWatchVehicles(const StopSignInfo& stop_sign_info,
                               StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  watch_vehicles->clear();

  StopSignStatus stop_sign_status = GetPlanningStatus()->stop_sign();
   // 遍历所有的车道
  for (int i = 0; i < stop_sign_status.lane_watch_vehicles_size(); ++i) {
    auto lane_watch_vehicles = stop_sign_status.lane_watch_vehicles(i);
    std::string associated_lane_id = lane_watch_vehicles.lane_id();
    std::string s;
        // 获取每个车道的等候车辆
    for (int j = 0; j < lane_watch_vehicles.watch_vehicles_size(); ++j) {
      std::string vehicle = lane_watch_vehicles.watch_vehicles(j);
      s = s.empty() ? vehicle : s + "," + vehicle;
      (*watch_vehicles)[associated_lane_id].push_back(vehicle);
    }
    ADEBUG << "GetWatchVehicles watch_vehicles: lane_id[" << associated_lane_id
           << "] vehicle[" << s << "]";
  }

  return 0;
}

这个过程其实就是获取无人车前方的等待车辆，存储形式为：

typedef std::unordered_map<std::string, std::vector<std::string>> StopSignLaneVehicles;

map中第一个string是车道id，第二个vector<string>是这个车道上在无人车前方的等待车辆id。整体的查询是直接在PlanningStatus.stop_sign(停车状态)中获取，第一次为空，后续不为空。



// 2 检查与更新停车状态PlanningStatus.stop_sign--由函数ProcessStopStatus完成
停车过程可以分为5个阶段：正常行驶DRIVE--开始停车STOP--等待缓冲状态WAIT--缓慢前进CREEP--彻底停车DONE。

/*
如果停车状态是正常行驶DRIVE。
这种情况下，如果车辆速度很大，或者与停车区域距离过远，那么继续设置为行驶DRIVE；反之就进入停车状态STOP。状态检查由CheckADCkStop完成。

如果停车状态是开始停车STOP。
这种情况下，如果从开始停车到当前经历的等待时间没有超过阈值stop_duration(默认1s)，继续保持STOP状态。反之，如果前方等待车辆不为空，那么就进入下一阶段WAIT缓冲阶段；如果前方车辆为空，那么可以直接进入到缓慢前进CREEP状态或者停车完毕状态。

如果停车状态是等待缓冲WAIT
这种情况下，如果等待时间没有超过一个阈值wait_timeout(默认8s)或者前方存在等待车辆，继续保持等待状态。反之可以进入到缓慢前进或者停车完毕状态

如果停车状态是缓慢前进CREEP
这种情况下，只需要检查无人车车头和停车区域的距离，如果大于某个值那么说明可以继续缓慢前进，保持状态不变，反之就可以完全停车了。

更新前方等待车辆
a.当前状态是DRIVE，那么需要将障碍物都加入到前方等待车辆列表中，因为这些障碍物到时候都会排在无人车前方等待。
*/

int StopSign::ProcessStopStatus(ReferenceLineInfo* const reference_line_info,
                                const StopSignInfo& stop_sign_info,
                                StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(reference_line_info);

  // get stop status from PlanningStatus
  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  if (!stop_sign_status->has_status()) {
    stop_sign_status->set_status(StopSignStatus::UNKNOWN);
  }
  stop_status_ = stop_sign_status->status();

  // adjust status
  // 更新停车状态。如果无人车距离最近一个停车区域过远，那么状态为正常行驶
  double adc_front_edge_s = reference_line_info->AdcSlBoundary().end_s();
  double stop_line_start_s = next_stop_sign_overlap_.start_s;
  if (stop_line_start_s - adc_front_edge_s >
      config_.stop_sign().max_valid_stop_distance()) {
    ADEBUG << "adjust stop status. too far from stop line. distance["
           << stop_line_start_s - adc_front_edge_s << "]";
    stop_status_ = StopSignStatus::DRIVE;
  }

  // get stop start time from PlanningStatus
  double stop_start_time = Clock::NowInSeconds() + 1;
  if (stop_sign_status->has_stop_start_time()) {
    stop_start_time = stop_sign_status->stop_start_time();
  }
  double wait_time = Clock::NowInSeconds() - stop_start_time;
  ADEBUG << "stop_start_time: " << stop_start_time
         << "; wait_time: " << wait_time;

  // check & update stop status
  switch (stop_status_) {
    case StopSignStatus::UNKNOWN:
    // a. 当前状态是DRIVE，那么需要将障碍物都加入到前方等待车辆列表中，因为这些障碍物到时候都会排在无人车前方等待
    case StopSignStatus::DRIVE:
      stop_status_ = StopSignStatus::DRIVE;
      if (CheckADCkStop(reference_line_info)) {
        stop_start_time = Clock::NowInSeconds();
        stop_status_ = StopSignStatus::STOP;

        // update PlanningStatus: stop start time
        stop_sign_status->set_stop_start_time(stop_start_time);
        ADEBUG << "update stop_start_time: " << stop_start_time;
      }
      break;
    case StopSignStatus::STOP:
      if (wait_time >= config_.stop_sign().stop_duration()) {
        if (watch_vehicles != nullptr && !watch_vehicles->empty()) {
          stop_status_ = StopSignStatus::WAIT;
        } else {
          stop_status_ = CheckCreep(stop_sign_info) ?
              StopSignStatus::CREEP : StopSignStatus::STOP_DONE;
        }
      }
      break;
    case StopSignStatus::WAIT:
      if (wait_time > config_.stop_sign().wait_timeout() ||
          (watch_vehicles == nullptr || watch_vehicles->empty())) {
        stop_status_ = CheckCreep(stop_sign_info) ?
            StopSignStatus::CREEP : StopSignStatus::STOP_DONE;
      }
      break;
    case StopSignStatus::CREEP:
      if (CheckCreepDone(reference_line_info)) {
        stop_status_ = StopSignStatus::STOP_DONE;
      }
      break;
    case StopSignStatus::STOP_DONE:
      break;
    default:
      break;
  }

  // update PlanningStatus: stop status
  stop_sign_status->set_status(stop_status_);
  ADEBUG << "update stop_status: " << StopSignStatus_Status_Name(stop_status_);

  return 0;
}


// b.如果无人车当前状态是等待或者停车，删除部分排队等待车辆--RemoveWatchVehicle函数完成

这种情况下，如果障碍物已经驶过停车区域，那么对其删除；否则继续保留。


int StopSign::RemoveWatchVehicle(
    const PathObstacle& path_obstacle,
    const std::vector<std::string>& watch_vehicle_ids,
    StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  const PerceptionObstacle& perception_obstacle =
      path_obstacle.obstacle()->Perception();
  const std::string& obstacle_id = std::to_string(perception_obstacle.id());
  PerceptionObstacle::Type obstacle_type = perception_obstacle.type();
  std::string obstacle_type_name = PerceptionObstacle_Type_Name(obstacle_type);

  // check if being watched
  if (std::find(watch_vehicle_ids.begin(), watch_vehicle_ids.end(),
                obstacle_id) == watch_vehicle_ids.end()) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "] not being watched. skip";
    return 0;
  }

  // check type
  if (obstacle_type != PerceptionObstacle::UNKNOWN &&
      obstacle_type != PerceptionObstacle::UNKNOWN_MOVABLE &&
      obstacle_type != PerceptionObstacle::BICYCLE &&
      obstacle_type != PerceptionObstacle::VEHICLE) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]. skip";
    return 0;
  }

  auto point = common::util::MakePointENU(perception_obstacle.position().x(),
                                          perception_obstacle.position().y(),
                                          perception_obstacle.position().z());
  double obstacle_s = 0.0;
  double obstacle_l = 0.0;
  LaneInfoConstPtr obstacle_lane;
  if (HDMapUtil::BaseMap().GetNearestLaneWithHeading(
          point, 5.0, perception_obstacle.theta(), M_PI / 3.0, &obstacle_lane,
          &obstacle_s, &obstacle_l) != 0) {
    ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
           << "]: Failed to find nearest lane from map for position: "
           << point.DebugString()
           << "; heading:" << perception_obstacle.theta();
    return -1;
  }

  bool erase = false;

  bool is_path_cross = !path_obstacle.reference_line_st_boundary().IsEmpty();

  // check obstacle is on an associate lane guarded by stop sign
  const std::string& obstable_lane_id = obstacle_lane.get()->id().id();
  auto assoc_lane_it = std::find_if(
      associated_lanes_.begin(), associated_lanes_.end(),
      [&obstable_lane_id](
          std::pair<LaneInfoConstPtr, OverlapInfoConstPtr>& assc_lane) {
        return assc_lane.first.get()->id().id() == obstable_lane_id;
      });
  if (assoc_lane_it != associated_lanes_.end()) {
    // check pass stop line of the stop_sign
    auto over_lap_info = assoc_lane_it->second.get()->GetObjectOverlapInfo(
        obstacle_lane.get()->id());
    if (over_lap_info == nullptr) {
      AERROR << "can't find over_lap_info for id: " << obstable_lane_id;
      return -1;
    }

    double stop_line_end_s = over_lap_info->lane_overlap_info().end_s();
    double obstacle_end_s = obstacle_s + perception_obstacle.length() / 2;
    double distance_pass_stop_line = obstacle_end_s - stop_line_end_s;
    // 如果障碍物已经驶过停车区一定距离，可以将障碍物从等待车辆中删除。
    if (distance_pass_stop_line > config_.stop_sign().min_pass_s_distance() &&
        !is_path_cross) {
      erase = true;

      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] distance_pass_stop_line[" << distance_pass_stop_line
             << "] stop_line_end_s[" << stop_line_end_s << "] obstacle_end_s["
             << obstacle_end_s << "] is_path_cross[" << is_path_cross
             << "] passed stop sign, AND path not crosses. "
             << "erase from watch_vehicles";
    }
  } else {
    // passes associated lane (in junction)
    if (!is_path_cross) {
      erase = true;
      ADEBUG << "obstacle_id[" << obstacle_id << "] type[" << obstacle_type_name
             << "] obstable_lane_id[" << obstable_lane_id << "] is_path_cross["
             << is_path_cross
             << "] passed associated lane, AND path not crosses. "
             << "erase from watch_vehicles";
    }
  }

  // check if obstacle stops
  /*
  if (!erase) {
    auto speed = std::hypot(perception_obstacle.velocity().x(),
                            perception_obstacle.velocity().y());
    if (speed > config_.stop_sign().max_watch_vehicle_stop_speed()) {
      ADEBUG << "obstacle_id[" << obstacle_id
          << "] type[" << obstacle_type_name
          << "] velocity[" << speed
          << "] not stopped. erase from watch_vehicles";
      erase = true;
    }
  }
  */

  if (erase) {
    for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
         it != watch_vehicles->end(); ++it) {
      std::vector<std::string>& vehicles = it->second;
      vehicles.erase(std::remove(vehicles.begin(), vehicles.end(), obstacle_id),
                     vehicles.end());
    }
  }

  return 0;
}


对剩下来的障碍物重新组成一个新的等待队列--ClearWatchVehicle函数完成

int StopSign::ClearWatchVehicle(ReferenceLineInfo* const reference_line_info,
                                StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(reference_line_info);
  CHECK_NOTNULL(watch_vehicles);

  const auto& path_obstacles =
      reference_line_info->path_decision()->path_obstacles().Items();
  std::unordered_set<std::string> obstacle_ids;
  std::transform(
      path_obstacles.begin(), path_obstacles.end(),
      std::inserter(obstacle_ids, obstacle_ids.end()),
      [](const PathObstacle* path_obstacle) {
        return std::to_string(path_obstacle->obstacle()->Perception().id());
      });

  for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
       it != watch_vehicles->end();
       /*no increment*/) {
    std::vector<std::string>& vehicle_ids = it->second;
    // clean obstacles not in current perception
       // 如果新的队列中已经不存在该障碍物了，那么直接将障碍物从这条车道中删除
    for (auto obstacle_it = vehicle_ids.begin();
         obstacle_it != vehicle_ids.end();
         /*no increment*/) {
      if (obstacle_ids.count(*obstacle_it) == 0) {
        ADEBUG << "lane[" << it->first << "] obstacle[" << *obstacle_it
               << "] not exist any more. erase.";
        obstacle_it = vehicle_ids.erase(obstacle_it);
      } else {
        ++obstacle_it;
      }
    }
    if (vehicle_ids.empty()) {  // 如果这整条车道上都不存在等待车辆了，直接把这条车道删除
      watch_vehicles->erase(it++);
    } else {
      ++it;
    }
  }

  /* debug
  for (StopSignLaneVehicles::iterator it = watch_vehicles->begin();
       it != watch_vehicles->end(); it++) {
    std::string s;
    std::for_each(it->second.begin(),
                  it->second.end(),
                  [&](std::string &id) { s = s.empty() ? id : s + "," + id; });
    ADEBUG << "ClearWatchVehicle: lane_id[" << it->first << "] vehicle["
        << s << "]; size[" << it->second.size() << "]";
  }
  */

  return 0;
}


更新车辆状态PlanningStatus.stop_sign
这部分由函数UpdateWatchVehicles完成，主要是将3中得到的新的等待车辆队列更新至stop_sign。


int StopSign::UpdateWatchVehicles(StopSignLaneVehicles* watch_vehicles) {
  CHECK_NOTNULL(watch_vehicles);

  auto* stop_sign_status = GetPlanningStatus()->mutable_stop_sign();
  stop_sign_status->clear_lane_watch_vehicles();

  for (auto it = watch_vehicles->begin(); it != watch_vehicles->end(); ++it) {
    auto* lane_watch_vehicles = stop_sign_status->add_lane_watch_vehicles();
    lane_watch_vehicles->set_lane_id(it->first);
    std::string s;
    for (size_t i = 0; i < it->second.size(); ++i) {
      std::string vehicle = it->second[i];
      s = s.empty() ? vehicle : s + "," + vehicle;
      lane_watch_vehicles->add_watch_vehicles(vehicle);
    }
    ADEBUG << "UpdateWatchVehicles watch_vehicles: lane_id[" << it->first
           << "] vehicle[" << s << "]";
  }

  return 0;
}



最后总结一下，障碍物和路况对无人车的决策影响分为两类，一类是纵向影响LongitudinalDecision，一类是侧向影响LateralDecision。

纵向影响：

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int, athObstacle::ObjectTagCaseHash>
    PathObstacle::s_longitudinal_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},      // 忽略，优先级0
        {ObjectDecisionType::kOvertake, 100},  // 超车，优先级100
        {ObjectDecisionType::kFollow, 300},    // 跟随，优先级300
        {ObjectDecisionType::kYield, 400},     // 减速，优先级400
        {ObjectDecisionType::kStop, 500}};     // 停车，优先级500
侧向影响：

const std::unordered_map<ObjectDecisionType::ObjectTagCase, int, PathObstacle::ObjectTagCaseHash>
    PathObstacle::s_lateral_decision_safety_sorter_ = {
        {ObjectDecisionType::kIgnore, 0},      // 忽略，优先级0
        {ObjectDecisionType::kNudge, 100},     // 微调，优先级100
        {ObjectDecisionType::kSidepass, 200}}; // 绕行，优先级200
当有一个障碍物在11中路况下多次出发无人车决策时该怎么办？

纵向决策合并，lhs为第一次决策，rhs为第二次决策，如何合并两次决策

ObjectDecisionType PathObstacle::MergeLongitudinalDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_longitudinal_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {        // 优先选取优先级大的决策
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore()) {
      return rhs;
    } else if (lhs.has_stop()) {    // 如果优先级相同，都是停车，选取停车距离小的决策，防止安全事故
      return lhs.stop().distance_s() < rhs.stop().distance_s() ? lhs : rhs;
    } else if (lhs.has_yield()) {   // 如果优先级相同，都是减速，选取减速距离小的决策，防止安全事故
      return lhs.yield().distance_s() < rhs.yield().distance_s() ? lhs : rhs;
    } else if (lhs.has_follow()) {  // 如果优先级相同，都是跟随，选取跟随距离小的决策，防止安全事故
      return lhs.follow().distance_s() < rhs.follow().distance_s() ? lhs : rhs;
    } else if (lhs.has_overtake()) { // 如果优先级相同，都是超车，选取超车距离大的决策，防止安全事故
      return lhs.overtake().distance_s() > rhs.overtake().distance_s() ? lhs  : rhs;
    } else {
      DCHECK(false) << "Unknown decision";
    }
  }
  return lhs;  // stop compiler complaining
}
侧向合并，lhs为第一次决策，rhs为第二次决策，如何合并两次决策

ObjectDecisionType PathObstacle::MergeLateralDecision(
    const ObjectDecisionType& lhs, const ObjectDecisionType& rhs) {
  if (lhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return rhs;
  }
  if (rhs.object_tag_case() == ObjectDecisionType::OBJECT_TAG_NOT_SET) {
    return lhs;
  }
  const auto lhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, lhs.object_tag_case());
  const auto rhs_val =
      FindOrDie(s_lateral_decision_safety_sorter_, rhs.object_tag_case());
  if (lhs_val < rhs_val) {         // 优先选取优先级大的决策       
    return rhs;
  } else if (lhs_val > rhs_val) {
    return lhs;
  } else {
    if (lhs.has_ignore() || lhs.has_sidepass()) {
      return rhs;
    } else if (lhs.has_nudge()) {                        // 如果优先级相同，都是微调，选取侧向微调大的决策
      return std::fabs(lhs.nudge().distance_l()) >
                     std::fabs(rhs.nudge().distance_l())
                 ? lhs
                 : rhs;
    }
  }
  return lhs;
}