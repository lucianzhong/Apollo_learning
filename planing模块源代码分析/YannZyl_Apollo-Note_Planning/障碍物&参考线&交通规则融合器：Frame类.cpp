 
// apollo 3.0


 1. 
 在Frame类中，主要的工作还是对障碍物预测轨迹(由Predition模块得到的未来5s内障碍物运动轨迹)、无人车参考线(ReferenceLineProvider类提供)以及当前路况(停车标志、人行横道、减速带等)信息进行融合。

实际情况下，能影响无人车运动的不一定只有障碍物，同时还有各个路况，举个例子：

a. 障碍物影响

情况1：无人车车后的障碍物，对无人车没太大影响，可以忽略

情况2：无人车前面存在障碍物，无人车就需要停车或者超车

b. 路况影响

情况3：前方存在禁停区或者交叉路口(不考虑信号灯)，那么无人车在参考线上行驶，禁停区区域不能停车

情况4：前方存在人行横道，若有人，那么需要停车；若没人，那么无人车可以驶过

所以综上所述，其实这章节最重要的工作就是结合路况和障碍物轨迹，给每个障碍物(为了保持一致，路况也需要封装成障碍物形式)一个标签，这个标签表示该障碍物存在情况下对无人车的影响，例如有些障碍物可忽略，有些障碍物会促使无人车超车，有些障碍物促使无人车需要停车等。

障碍物信息的获取策略
无人车参考线ReferenceLineInof初始化(感知模块障碍物获取)
依据交通规则对障碍物设定标签(原始感知障碍物&&路况障碍物)




2.  障碍物信息的获取策略--滞后预测(Lagged Prediction)

主要的工作是获取Prediction模块发布的障碍物预测轨迹数据，并且进行后处理工作


Prediction模块发布的数据格式：
apollo/modules/prediction/proto/prediction_obstacle.proto


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


	可以使用prediction_obstacles.prediction_obstacle()形式获取所有障碍物的轨迹信息，对于每个障碍物prediction_obstacle，可以使用prediction_obstacle.trajectory()获取他所有可能运动方案/轨迹

	可以使用const auto& prediction = *(AdapterManager::GetPrediction());来获取Adapter中所有已发布的历史消息，最常见的肯定是取最近发布的PredictionObstacles(prediction.GetLatestObserved())，
	 但是Apollo中采用更为精确地障碍物预测获取方式--滞后预测(Lagged Prediction)，除了使用Prediction模块最近一次发布的信息，同时还是用历史信息中的障碍物轨迹预测数据



	 apollo/modules/planning/planning.cc

	Status Planning::InitFrame(const uint32_t sequence_num,
                           const TrajectoryPoint& planning_start_point,
                           const double start_time,
                           const VehicleState& vehicle_state) {
  frame_.reset(new Frame(sequence_num, planning_start_point, start_time,
                         vehicle_state, reference_line_provider_.get()));
  auto status = frame_->Init();
  if (!status.ok()) {
    AERROR << "failed to init frame:" << status.ToString();
    return status;
  }
  return Status::OK();
}



apollo/modules/planning/common/frame.cc

Status Frame::Init() {
  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
  vehicle_state_ = common::VehicleStateProvider::instance()->vehicle_state();
  const auto &point = common::util::MakePointENU(
      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
  if (std::isnan(point.x()) || std::isnan(point.y())) {
    AERROR << "init point is not set";
    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
  }
  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
         << FLAGS_align_prediction_time;

  // prediction
  if (FLAGS_enable_prediction && AdapterManager::GetPrediction() &&
      !AdapterManager::GetPrediction()->Empty()) {
    if (FLAGS_enable_lag_prediction && lag_predictor_) {
      lag_predictor_->GetLaggedPrediction(&prediction_); //// 滞后预测策略，获取障碍物轨迹信息
    } else {
      prediction_.CopyFrom(
          AdapterManager::GetPrediction()->GetLatestObserved());  // 不采用滞后预测策略，直接取最近一次Prediction模块发布的障碍物信息
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
    std::string err_str =
        "Found collision with obstacle: " + collision_obstacle->Id();
    apollo::common::monitor::MonitorLogBuffer buffer(&monitor_logger_);
    buffer.ERROR(err_str);
    return Status(ErrorCode::PLANNING_ERROR, err_str);
  }
  if (!CreateReferenceLineInfo()) {
    AERROR << "Failed to init reference line info";
    return Status(ErrorCode::PLANNING_ERROR,
                  "failed to init reference line info");
  }

  return Status::OK();
}




采用滞后预测策略获取障碍物轨迹信息的主要步骤可分为：
apollo/modules/planning/common/lag_prediction.cc

void LagPrediction::GetLaggedPrediction(PredictionObstacles* obstacles) const {
  obstacles->mutable_prediction_obstacle()->Clear();
  if (!AdapterManager::GetPrediction() ||
      AdapterManager::GetPrediction()->Empty()) {
    return;
  }
  const auto& prediction = *(AdapterManager::GetPrediction());
  if (!AdapterManager::GetLocalization() ||
      AdapterManager::GetLocalization()->Empty()) {  // no localization
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
    double distance =
        common::util::DistanceXY(perception.position(), adc_position);
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
      if (perception.confidence() < FLAGS_perception_confidence_threshold &&  perception.type() != PerceptionObstacle::VEHICLE) {			// 障碍物置信度必须大于0.5，获取必须是车辆VEHICLE类，否则不作处理
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
  obstacles->set_perception_error_code(
      latest_prediction->perception_error_code());
  obstacles->set_start_timestamp(latest_prediction->start_timestamp());
  obstacles->set_end_timestamp(latest_prediction->end_timestamp());
  bool apply_lag = std::distance(prediction.begin(), prediction.end()) >=
                   static_cast<int32_t>(min_appear_num_);
  for (const auto& iter : obstacle_lag_info) {
    if (apply_lag && iter.second.count < min_appear_num_) {		 // 历史信息中如果障碍物出现次数小于min_appear_num_/3次，次数太少，可忽略。
      continue;
    }
    if (apply_lag && iter.second.last_observed_seq > max_disappear_num_) {   // 历史信息中如果障碍物最近一次发布距离现在过远，可忽略。
      continue;
    }
    AddObstacleToPrediction(timestamp - iter.second.last_observed_time,
                            *(iter.second.obstacle_ptr), obstacles);
  }
}


最近一次发布的数据直接加入PredictionObstacles容器中
从上面的代码可以看到，滞后预测对于最近一次发布的数据处理比较简单，障碍物信息有效只需要满足两个条件：

障碍物置信度(Perception模块CNN分割获得)必须大于0.5，或者障碍物是车辆类
障碍物与车辆之间的距离小于30m


如何判断这些障碍物轨迹信息是否有效。两个步骤：

步骤1：记录历史发布数据中每个障碍物出现的次数(在最近依次发布中出现的障碍物忽略，因为不是最新的数据了)，必须满足两个条件：

障碍物置信度(Perception模块CNN分割获得)必须大于0.5，或者障碍物是车辆类
障碍物与车辆之间的距离小于30m
步骤2：对于步骤1中得到的障碍物信息，进行筛选，信息有效需要满足两个条件：

信息队列中历史数据大于3(min_appear_num_)，并且每个障碍物出现次数大于3(min_appear_num_)
信息队列中历史数据大于3(min_appear_num_)，并且障碍物信息上一次发布距离最近一次发布不大于5(max_disappear_num_)，需要保证数据的最近有效性。





3. 无人车与障碍物相对位置的设置--ReferenceLineInfo类初始化

	从**1. 障碍物信息的获取策略--滞后预测(Lagged Prediction)**中可以得到障碍物短期(未来5s)内的运动轨迹；从ReferenceLineProvider类中我们可以得到车辆的理想规划轨迹。
	下一步就是将障碍物的轨迹信息加入到这条规划好的参考线ReferenceLine中，确定在什么时间点，无人车能前进到什么位置，需要保证在这个时间点上，障碍物与无人车不相撞。
	这个工作依旧是在Frame::Init()中完成，主要是完成ReferenceLineInfo类的生成，这个类综合了障碍物预测轨迹与无人车规划轨迹的信息，同时也是最后路径规划的基础类


apollo/modules/planning/common/frame.cc

	Status Frame::Init() {
		  hdmap_ = hdmap::HDMapUtil::BaseMapPtr();
		  vehicle_state_ = common::VehicleStateProvider::instance()->vehicle_state();
		  const auto &point = common::util::MakePointENU(
		      vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());
		  if (std::isnan(point.x()) || std::isnan(point.y())) {
		    AERROR << "init point is not set";
		    return Status(ErrorCode::PLANNING_ERROR, "init point is not set");
		  }
		  ADEBUG << "Enabled align prediction time ? : " << std::boolalpha
		         << FLAGS_align_prediction_time;

		  // prediction
		         // Step A prediction，障碍物预测轨迹信息获取，采用滞后预测策略
		  if (FLAGS_enable_prediction && AdapterManager::GetPrediction() &&
		      !AdapterManager::GetPrediction()->Empty()) {
		    if (FLAGS_enable_lag_prediction && lag_predictor_) {
		      lag_predictor_->GetLaggedPrediction(&prediction_);
		    } else {
		      prediction_.CopyFrom(
		          AdapterManager::GetPrediction()->GetLatestObserved());
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


		如何完成ReferenceLineInfo类的初始化工作，其实比较简单，主要有两个过程：

		1.根据无人车规划路径ReferenceLine实例化ReferenceLineInfo类，数量与ReferenceLine一致
		2.根据障碍物轨迹初始化ReferenceLineInfo::path_decision_

	
	1. 实例化ReferenceLineInfo类:

	apollo/modules/planning/common/frame.cc

		bool Frame::CreateReferenceLineInfo() {

			 // Step A 从ReferenceLineProvider中获取无人车的短期内规划路径ReferenceLine，并进行收缩操作
		  std::list<ReferenceLine> reference_lines;
		  std::list<hdmap::RouteSegments> segments;
		  if (!reference_line_provider_->GetReferenceLines( &reference_lines, &segments )) {
		    AERROR << "Failed to create reference line";
		    return false;
		  }
		  DCHECK_EQ(reference_lines.size(), segments.size());



		  auto forword_limit =
		      ReferenceLineProvider::LookForwardDistance(vehicle_state_);

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

		  // Step B RerfenceLineInfo初始化
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
		  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(),
		                                     &perception_sl)) {
		    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
		    return path_obstacle;
		  }
		  path_obstacle->SetPerceptionSlBoundary(perception_sl);


		   // 计算障碍物是否对无人车行驶有影响：无光障碍物满足以下条件：
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




		apollo/modules/planning/common/path_obstacle.cc

		void PathObstacle::BuildReferenceLineStBoundary(
    const ReferenceLine& reference_line, const double adc_start_s) {
  const auto& adc_param =
      VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  const double adc_width = adc_param.width();
  if (obstacle_->IsStatic() ||
      obstacle_->Trajectory().trajectory_point().empty()) {
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


		apollo/modules/planning/common/path_obstacle.cc



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
   //Step 1. 首先还是对障碍物轨迹点两两选择，每两个点可以构建上图中的object_moving_box以及object_boundary。


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


总结一下**无人车参考线ReferenceLineInof初始化(加入障碍物轨迹信息)**这步的功能，给定了无人车的规划轨迹ReferenceLine和障碍物的预测轨迹PredictionObstacles，
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


	apollo/modules/planning/planning.cc
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



apollo/modules/planning/tasks/traffic_decider/traffic_decider.cc


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




4.1 后车情况处理--BACKSIDE_VEHICLE

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

apollo/modules/planning/tasks/traffic_decider/change_lane.cc

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



创建障碍物超车标签

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