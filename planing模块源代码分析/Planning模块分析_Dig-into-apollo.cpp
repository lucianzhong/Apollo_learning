
https://github.com/daohu527/Dig-into-Apollo/tree/master/planning


1. 
	Planning模块简介
	规划(planning)模块的作用是根据感知预测的结果，当前的车辆信息和路况规划出一条车辆能够行驶的轨迹，这个轨迹会交给控制(control)模块，控制模块通过油门，刹车和方向盘使得车辆按照规划的轨迹运行。
	规划模块的轨迹是短期轨迹，即车辆短期内行驶的轨迹，长期的轨迹是routing模块规划出的导航轨迹，即起点到目的地的轨迹，规划模块会先生成导航轨迹，然后根据导航轨迹和路况的情况，沿着短期轨迹行驶，直到目的地。
	这点也很好理解，我们开车之前先打开导航，然后根据导航行驶，如果前面有车就会减速或者变道，超车，避让行人等，这就是短期轨迹，结合上述的方式直到行驶到目的地

	可以看到规划(planning)模块的上游是Localization, Prediction, Routing模块，而下游是Control模块。
	Routing模块先规划出一条导航线路，然后Planning模块根据这条线路做局部优化，如果Planning模块发现短期规划的线路行不通（比如前面修路，或者错过了路口），会触发Routing模块重新规划线路，因此这两个模块的数据流是双向的。
	Planning模块的输入在"planning_component.h"中，接口如下:

	apollo/modules/planning/planning_component.h
	bool Proc( const std::shared_ptr<prediction::PredictionObstacles>& prediction_obstacles, const std::shared_ptr<canbus::Chassis>& chassis, const std::shared_ptr<localization::LocalizationEstimate>& localization_estimate) override;

	输入参数为:
		预测的障碍物信息(prediction_obstacles)
		车辆底盘(chassis)信息(车辆的速度，加速度，航向角等信息)
		车辆当前位置(localization_estimate)
		实际上还有高精度地图信息，不在参数中传入，而是在函数中直接读取的。

	Planning模块的输出结果在"PlanningComponent::Proc()"中，为规划好的线路，发布到Control模块订阅的Topic中。输出结果为：规划好的路径
	modules/planning/planning_component.cc
	planning_writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));

	Planning整个流程:
	1. 模块的入口是PlanningComponent，在Cyber中注册模块，订阅和发布消息，并且注册对应的Planning类。
	2. Planning的过程之前是定时器触发，即每隔一段固定的时间执行一次，现已经改为事件触发，即只要收集完成对应TOPIC的消息，就会触发执行，这样的好处是提高的实时性。
	3. Planning类主要实现了2个功能，一个是启动ReferenceLineProvider来提供参考线，后面生成的轨迹都是在参考线的基础上做优化，ReferenceLineProvider启动了一个单独的线程，每隔50ms执行一次，和Planning主流程并行执行。Planning类另外的一个功能是执行Planning主流程。
	4.Planning主流程先是选择对应的Planner，我们这里主要分析PublicRoadPlanner，在配置文件中定义了Planner支持的场景(Scenario)，把规划分为具体的几个场景来执行，每个场景又分为几个阶段(Stage)，每个阶段会执行多个任务(Task)，任务执行完成后，对应的场景就完成了。
		不同场景间的切换是由一个状态机(ScenarioDispatch)来控制的。规划控制器根据ReferenceLineProvider提供的参考线，在不同的场景下做切换，生成一条车辆可以行驶的轨迹，并且不断重复上述过程直到到达目的地。


2. // Planning模块入口
   // 模块注册
	Planning模块的入口为"planning_component.h"和"planning_component.cc"两个文件
	  // 订阅和发布消息
	  std::shared_ptr<cyber::Reader<perception::TrafficLightDetection>> traffic_light_reader_;
	  std::shared_ptr<cyber::Reader<routing::RoutingResponse>> routing_reader_;
	  std::shared_ptr<cyber::Reader<planning::PadMessage>> pad_message_reader_;
	  std::shared_ptr<cyber::Reader<relative_map::MapMsg>> relative_map_reader_;

	  std::shared_ptr<cyber::Writer<ADCTrajectory>> planning_writer_;
	  std::shared_ptr<cyber::Writer<routing::RoutingRequest>> rerouting_writer_;
	  // 在Cyber中注册模块
	  CYBER_REGISTER_COMPONENT(PlanningComponent)


	//模块初始化
	除了注册模块，订阅和发布消息之外，planning模块实现了2个主要函数"init"和"proc"。
	Init中实现了模块的初始化：

	bool PlanningComponent::Init() {

	  if (FLAGS_open_space_planner_switchable) { //上面实现了3种Planning的注册，planning模块根据配置选择不同的Planning实现方式
	    planning_base_ = std::make_unique<OpenSpacePlanning>();
	  } else {
	    if (FLAGS_use_navigation_mode) {
	      planning_base_ = std::make_unique<NaviPlanning>();
	    } else {
	      planning_base_ = std::make_unique<OnLanePlanning>();
	    }
	  }
	  CHECK(apollo::cyber::common::GetProtoFromFile(FLAGS_planning_config_file, &config_)) << "failed to load planning config file " << FLAGS_planning_config_file;
	  planning_base_->Init(config_);

	  if (FLAGS_use_sim_time) {
	    Clock::SetMode(Clock::MOCK);
	  }

	  // Init接下来实现了具体的消息发布和消息订阅

	  // 读取routing模块的消息
	  routing_reader_ = node_->CreateReader<RoutingResponse>(FLAGS_routing_response_topic, [this](const std::shared_ptr<RoutingResponse>& routing) {
	        AINFO << "Received routing data: run routing callback." << routing->header().DebugString();
	        std::lock_guard<std::mutex> lock(mutex_);
	        routing_.CopyFrom(*routing);
	      });

	  // 读取红绿灯
	  traffic_light_reader_ = node_->CreateReader<TrafficLightDetection>( FLAGS_traffic_light_detection_topic, [this](const std::shared_ptr<TrafficLightDetection>& traffic_light) {
	        ADEBUG << "Received traffic light data: run traffic light callback.";
	        std::lock_guard<std::mutex> lock(mutex_);
	        traffic_light_.CopyFrom(*traffic_light);
	      });

	// 是否使用导航模式
	  if (FLAGS_use_navigation_mode) {
	    pad_message_reader_ = node_->CreateReader<PadMessage>( FLAGS_planning_pad_topic, [this](const std::shared_ptr<PadMessage>& pad_message) {
	          ADEBUG << "Received pad data: run pad callback.";
	          std::lock_guard<std::mutex> lock(mutex_);
	          pad_message_.CopyFrom(*pad_message);
	        });

	    // 读取相对地图
	    relative_map_reader_ = node_->CreateReader<MapMsg>(FLAGS_relative_map_topic, [this](const std::shared_ptr<MapMsg>& map_message) {
	          ADEBUG << "Received relative map data: run relative map callback.";
	          std::lock_guard<std::mutex> lock(mutex_);
	          relative_map_.CopyFrom(*map_message);
	        });
	  }
	  // 发布规划好的线路
	  planning_writer_ =  node_->CreateWriter<ADCTrajectory>(FLAGS_planning_trajectory_topic);
	  // 发布重新规划请求
	  rerouting_writer_ = node_->CreateWriter<RoutingRequest>(FLAGS_routing_request_topic);
	  return true;
	}




	上面实现了3种Planning的注册，planning模块根据配置选择不同的Planning实现方式，"FLAGS_open_space_planner_switchable"和"FLAGS_use_navigation_mode"在Planning模块的conf目录中。因为上述2个配置默认都为false，Planning默认情况下的实现是"OnLanePlanning"。
		OpenSpacePlanning - 主要的应用场景是自主泊车和狭窄路段的掉头。参考
		NaviPlanning -
		OnLanePlanning - 主要的应用场景是开放道路的自动驾驶。




	// 模块运行
	Proc的主要是检查数据，并且执行注册好的Planning，生成路线并且发布

	bool PlanningComponent::Proc( const std::shared_ptr<prediction::PredictionObstacles>&  prediction_obstacles, const std::shared_ptr<canbus::Chassis>& chassis,  const std::shared_ptr<localization::LocalizationEstimate>& localization_estimate) {
	  CHECK(prediction_obstacles != nullptr);

	  if (FLAGS_use_sim_time) {
	    Clock::SetNowInSeconds(localization_estimate->header().timestamp_sec());
	  }
	  // check and process possible rerouting request    // 1. 检查是否需要重新规划线路
	  CheckRerouting();

	  // process fused input data    // 2. 数据放入local_view_中，并且检查输入数据。
	  local_view_.prediction_obstacles = prediction_obstacles;
	  local_view_.chassis = chassis;
	  local_view_.localization_estimate = localization_estimate;
	  {
	    std::lock_guard<std::mutex> lock(mutex_);
	    if (!local_view_.routing ||  hdmap::PncMap::IsNewRouting(*local_view_.routing, routing_)) {
	      local_view_.routing = std::make_shared<routing::RoutingResponse>(routing_);
	    }
	  }
	  {
	    std::lock_guard<std::mutex> lock(mutex_);
	    local_view_.traffic_light = std::make_shared<TrafficLightDetection>(traffic_light_);
	    local_view_.relative_map = std::make_shared<MapMsg>(relative_map_);
	  }

	  if (!CheckInput()) {
	    AERROR << "Input check failed";
	    return false;
	  }

	  ADCTrajectory adc_trajectory_pb;
	  planning_base_->RunOnce(local_view_, &adc_trajectory_pb);    // 3. 执行注册好的Planning，生成线路。
	  auto start_time = adc_trajectory_pb.header().timestamp_sec();
	  common::util::FillHeader(node_->Name(), &adc_trajectory_pb);

	  // modify trajectory relative time due to the timestamp change in header
	  const double dt = start_time - adc_trajectory_pb.header().timestamp_sec();
	  for (auto& p : *adc_trajectory_pb.mutable_trajectory_point()) {
	    p.set_relative_time(p.relative_time() + dt);
	  }
	  planning_writer_->Write(std::make_shared<ADCTrajectory>(adc_trajectory_pb));    // 4. 发布消息
	  return true;
	}


	整个"PlanningComponent"的分析就完成了，可以看到"PlanningComponent"是Planning模块的入口，在Apollo3.5引入了Cyber之后，实现了Planning模块在Cyber中的注册，订阅和发布topic消息。同时实现了3种不同的Planning，根据配置选择其中的一种并且运行。
	由于默认的Planning是开放道路的OnLanePlanning，我们接下来主要分析这个Planning。


3. OnLanePlanning
	每次Planning会根据以下2个信息作为输入来执行：
		Planning上下文信息
		Frame结构体(车辆信息，位置信息等所有规划需要用到的信息，在/planning/common/frame.h中)


	modules/planning/common/frame.h


	// 初始化
	OnLanePlanning的初始化逻辑在Init中，主要实现分配具体的Planner，启动参考线提供器(reference_line_provider_)，代码分析如下：
	modules/planning/on_lane_planning.cc

	Status OnLanePlanning::Init(const PlanningConfig& config) {
	  config_ = config;
	  if (!CheckPlanningConfig(config_)) {
	    return Status(ErrorCode::PLANNING_ERROR, "planning config error: " + config_.DebugString());
	  }

	  PlanningBase::Init(config_);

	  planner_dispatcher_->Init();

	  CHECK(apollo::cyber::common::GetProtoFromFile( FLAGS_traffic_rule_config_filename, &traffic_rule_configs_)) << "Failed to load traffic rule config file " << FLAGS_traffic_rule_config_filename;

	  // clear planning status
	  PlanningContext::Instance()->mutable_planning_status()->Clear();

	  // load map
	  hdmap_ = HDMapUtil::BaseMapPtr();
	  CHECK(hdmap_) << "Failed to load map";

	  // instantiate reference line provider
	  // 启动参考线提供器，会另启动一个线程，执行一个定时任务，每隔50ms提供一次参考线。
	  reference_line_provider_ = std::make_unique<ReferenceLineProvider>(hdmap_);
	  reference_line_provider_->Start();

	  // dispatch planner
	  // 启动参考线提供器，会另启动一个线程，执行一个定时任务，每隔50ms提供一次参考线。
	  planner_ = planner_dispatcher_->DispatchPlanner();
	  if (!planner_) {
	    return Status( ErrorCode::PLANNING_ERROR,  "planning is not initialized with config : " + config_.DebugString());
	  }
	  start_time_ = Clock::NowInSeconds();
	  return planner_->Init(config_);
	}


	// 可以看到"DispatchPlanner"在"OnLanePlanning"实例化的时候就指定了
	/modules/planning/on_lane_planning.h

	class OnLanePlanning : public PlanningBase {
		 public:
		  OnLanePlanning() {
		    planner_dispatcher_ = std::make_unique<OnLanePlannerDispatcher>();
		  }
		}


	// 在看"OnLanePlannerDispatcher"具体的实现
	modules/planning/planner/on_lane_planner_dispatcher.cc

	namespace apollo {
	namespace planning {
	// OnLanePlannerDispatcher具体实现
	std::unique_ptr<Planner> OnLanePlannerDispatcher::DispatchPlanner() {
	  PlanningConfig planning_config;
	  bool res_load_config = apollo::cyber::common::GetProtoFromFile(FLAGS_planning_config_file, &planning_config);
	  if (!res_load_config) {
	    return nullptr;
	  }

	  return planner_factory_.CreateObject(
	      planning_config.standard_planning_config().planner_type(0));  // PUBLIC_ROAD规划器
	}

	}  // namespace planning
	}  // namespace apollo




	//事件触发
	OnLanePlanning的主要逻辑在"RunOnce()"中，在Apollo 3.5之前是定时器触发，3.5改为事件触发，即收到对应的消息之后，就触发执行，这样做的好处是增加了实时性 
	modules/planning/on_lane_planning.cc

void OnLanePlanning::RunOnce(const LocalView& local_view, ADCTrajectory* const ptr_trajectory_pb) {
	  local_view_ = local_view;
	  const double start_timestamp = Clock::NowInSeconds();
	  const double start_system_timestamp = std::chrono::duration<double>( std::chrono::system_clock::now().time_since_epoch()).count();

	  // localization
	  ADEBUG << "Get localization:" << local_view_.localization_estimate->DebugString();
	  // chassis
	  ADEBUG << "Get chassis:" << local_view_.chassis->DebugString();

	  Status status = VehicleStateProvider::Instance()->Update( *local_view_.localization_estimate, *local_view_.chassis);

	  VehicleState vehicle_state = VehicleStateProvider::Instance()->vehicle_state();
	  const double vehicle_state_timestamp = vehicle_state.timestamp();
	  DCHECK_GE(start_timestamp, vehicle_state_timestamp);

	  if (!status.ok() || !util::IsVehicleStateValid(vehicle_state)) {
	    std::string msg( "Update VehicleStateProvider failed " "or the vehicle state is out dated.");
	    AERROR << msg;
	    ptr_trajectory_pb->mutable_decision()->mutable_main_decision()->mutable_not_ready()->set_reason(msg);
	    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
	    // TODO(all): integrate reverse gear
	    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
	    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
	    GenerateStopTrajectory(ptr_trajectory_pb);
	    return;
	  }

	  if (start_timestamp - vehicle_state_timestamp < FLAGS_message_latency_threshold) {
	    vehicle_state = AlignTimeStamp(vehicle_state, start_timestamp);
	  }

	  if (util::IsDifferentRouting(last_routing_, *local_view_.routing)) {
	    last_routing_ = *local_view_.routing;
	    PlanningContext::Instance()->mutable_planning_status()->Clear();
	    reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);
	  }

	  // Update reference line provider and reset pull over if necessary
	  reference_line_provider_->UpdateVehicleState(vehicle_state);

	  // planning is triggered by prediction data, but we can still use an estimated
	  // cycle time for stitching
	  const double planning_cycle_time =  1.0 / static_cast<double>(FLAGS_planning_loop_rate);

	  std::string replan_reason;
	  std::vector<TrajectoryPoint> stitching_trajectory =
	      TrajectoryStitcher::ComputeStitchingTrajectory(
	          vehicle_state, start_timestamp, planning_cycle_time,
	          FLAGS_trajectory_stitching_preserved_length, true,
	          last_publishable_trajectory_.get(), &replan_reason);

	  EgoInfo::Instance()->Update(stitching_trajectory.back(), vehicle_state);
	  const uint32_t frame_num = static_cast<uint32_t>(seq_num_++);

	    // 初始化Frame
	  status = InitFrame(frame_num, stitching_trajectory.back(), vehicle_state);

	  if (status.ok()) {
	    EgoInfo::Instance()->CalculateFrontObstacleClearDistance(
	        frame_->obstacles());
	  }

	  if (FLAGS_enable_record_debug) {
	    frame_->RecordInputDebug(ptr_trajectory_pb->mutable_debug());
	  }
	  ptr_trajectory_pb->mutable_latency_stats()->set_init_frame_time_ms(
	      Clock::NowInSeconds() - start_timestamp);
	  if (!status.ok()) {
	    AERROR << status.ToString();
	    if (FLAGS_publish_estop) {
	      // "estop" signal check in function "Control::ProduceControlCommand()"
	      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
	      // we should add more information to ensure the estop being triggered.
	      ADCTrajectory estop_trajectory;
	      EStop* estop = estop_trajectory.mutable_estop();
	      estop->set_is_estop(true);
	      estop->set_reason(status.error_message());
	      status.Save(estop_trajectory.mutable_header()->mutable_status());
	      ptr_trajectory_pb->CopyFrom(estop_trajectory);
	    } else {
	      ptr_trajectory_pb->mutable_decision()
	          ->mutable_main_decision()
	          ->mutable_not_ready()
	          ->set_reason(status.ToString());
	      status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
	      GenerateStopTrajectory(ptr_trajectory_pb);
	    }
	    // TODO(all): integrate reverse gear
	    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
	    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
	    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
	    const uint32_t n = frame_->SequenceNum();
	    FrameHistory::Instance()->Add(n, std::move(frame_));
	    return;
	  }

	  for (auto& ref_line_info : *frame_->mutable_reference_line_info()) {
	    TrafficDecider traffic_decider;
	    traffic_decider.Init(traffic_rule_configs_);
	    auto traffic_status = traffic_decider.Execute(frame_.get(), &ref_line_info);
	    if (!traffic_status.ok() || !ref_line_info.IsDrivable()) {
	      ref_line_info.SetDrivable(false);
	      AWARN << "Reference line " << ref_line_info.Lanes().Id()
	            << " traffic decider failed";
	    }
	  }

	  // 执行计划
	  status = Plan(start_timestamp, stitching_trajectory, ptr_trajectory_pb);

	  for (const auto& p : ptr_trajectory_pb->trajectory_point()) {
	    ADEBUG << p.DebugString();
	  }

	  const auto end_system_timestamp = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
	  const auto time_diff_ms = (end_system_timestamp - start_system_timestamp) * 1000;
	  ADEBUG << "total planning time spend: " << time_diff_ms << " ms.";

	  ptr_trajectory_pb->mutable_latency_stats()->set_total_time_ms(time_diff_ms);
	  ADEBUG << "Planning latency: " << ptr_trajectory_pb->latency_stats().DebugString();

	  if (!status.ok()) {
	    status.Save(ptr_trajectory_pb->mutable_header()->mutable_status());
	    AERROR << "Planning failed:" << status.ToString();
	    if (FLAGS_publish_estop) {
	      AERROR << "Planning failed and set estop";
	      // "estop" signal check in function "Control::ProduceControlCommand()"
	      // estop_ = estop_ || local_view_.trajectory.estop().is_estop();
	      // we should add more information to ensure the estop being triggered.
	      EStop* estop = ptr_trajectory_pb->mutable_estop();
	      estop->set_is_estop(true);
	      estop->set_reason(status.error_message());
	    }
	  }

	  ptr_trajectory_pb->set_is_replan(stitching_trajectory.size() == 1);
	  if (ptr_trajectory_pb->is_replan()) {
	    ptr_trajectory_pb->set_replan_reason(replan_reason);
	  }

	  if (frame_->open_space_info().is_on_open_space_trajectory()) {
	    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
	    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();
	    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
	  } else {
	    auto* ref_line_task = ptr_trajectory_pb->mutable_latency_stats()->add_task_stats();
	    ref_line_task->set_time_ms(reference_line_provider_->LastTimeDelay() * 1000.0);
	    ref_line_task->set_name("ReferenceLineProvider");
	    // TODO(all): integrate reverse gear
	    ptr_trajectory_pb->set_gear(canbus::Chassis::GEAR_DRIVE);
	    FillPlanningPb(start_timestamp, ptr_trajectory_pb);
	    ADEBUG << "Planning pb:" << ptr_trajectory_pb->header().DebugString();

	    frame_->set_current_frame_planned_trajectory(*ptr_trajectory_pb);
	    if (FLAGS_enable_planning_smoother) {
	      planning_smoother_.Smooth(FrameHistory::Instance(), frame_.get(), ptr_trajectory_pb);
	    }
	  }

	  const uint32_t n = frame_->SequenceNum();
	  FrameHistory::Instance()->Add(n, std::move(frame_));
	}





4.  // Planner
	我们先看下Planner目录结构，一共实现了5种Planner：
	.
	├── BUILD
	├── navi_planner_dispatcher.cc
	├── navi_planner_dispatcher.h
	├── navi_planner_dispatcher_test.cc
	├── on_lane_planner_dispatcher.cc
	├── on_lane_planner_dispatcher.h
	├── on_lane_planner_dispatcher_test.cc
	├── planner_dispatcher.cc
	├── planner_dispatcher.h
	├── planner.h
	├── lattice           // lattice planner
	├── navi              // navi planner
	├── open_space        // open space planner
	├── public_road       // public road planner
	└── rtk               // rtk planner


	可以看到Planner目录分别实现了Planner发布器和具体的Planner，关于发布器我们后面会根据流程图来介绍，这里先介绍一下5种不同的Planner:
		rtk - 根据录制的轨迹来规划行车路线
		public_road - 开放道路的轨迹规划器
		lattice - 基于网格算法的轨迹规划器
		navi - 基于实时相对地图的规划器
		open_space - 自主泊车规划器

	// Planner注册场景
		PlanningComponent在cyber中注册
		选择Planning
		根据不同的Dispatcher，分发Planner


	下面我们主要介绍"PublicRoadPlanner"，主要的实现还是在Init和Plan中。init中主要是注册规划器支持的场景(scenario)

	modules/planning/planner/public_road/public_road_planner.cc


	Status PublicRoadPlanner::Init(const PlanningConfig& config) {
	  config_ = config;   // 读取public Road配置
	  scenario_manager_.Init();
	  return Status::OK();
	}

	我们看下"PublicRoadPlanner"支持的场景有哪些？
	// 还是在"/conf/planning_config.pb.txt"中

	standard_planning_config {
	  planner_type: PUBLIC_ROAD
	  planner_public_road_config {
	  	  // 支持的场景
	     scenario_type: LANE_FOLLOW  // 车道线保持
	     scenario_type: BARE_INTERSECTION_UNPROTECTED   // 超车
	     scenario_type: PULL_OVER   // 停止
	     scenario_type: STOP_SIGN_UNPROTECTED  
	     scenario_type: TRAFFIC_LIGHT_PROTECTED  // 红绿灯
	     scenario_type: TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN  // 红绿灯左转
	     scenario_type: TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN  // 红绿灯右转
	     scenario_type: VALET_PARKING  // 代客泊车
	  }
	}


	// 运行场景
	Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point, Frame* frame,  ADCTrajectory* ptr_computed_trajectory) {
	  DCHECK_NOTNULL(frame);
	  // 更新场景，决策当前应该执行什么场景
	  scenario_manager_.Update(planning_start_point, *frame);
	   // 获取当前场景
	  scenario_ = scenario_manager_.mutable_scenario();
	  // 执行当前场景的任务
	  auto result = scenario_->Process(planning_start_point, frame);

	  if (FLAGS_enable_record_debug) {
	    auto scenario_debug = ptr_computed_trajectory->mutable_debug() ->mutable_planning_data() ->mutable_scenario();
	    scenario_debug->set_scenario_type(scenario_->scenario_type());
	    scenario_debug->set_stage_type(scenario_->GetStage());
	    scenario_debug->set_msg(scenario_->GetMsg());
	  }

	   // 当前场景完成
	  if (result == scenario::Scenario::STATUS_DONE) {
	    // only updates scenario manager when previous scenario's status is
	    // STATUS_DONE
	    scenario_manager_.Update(planning_start_point, *frame);
	  } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
	  	// 当前场景失败
	    return Status(common::PLANNING_ERROR, "scenario returned unknown");
	  }
	  return Status::OK();
	}

	可以看到"Planner"模块把具体的规划转化成一系列的场景，每次执行规划之前先判断更新当前的场景，然后针对具体的场景去执行。



5. // Scenario

	我们同样先看下"Scenario"的目录结构：

	.
	├── bare_intersection
	├── BUILD
	├── lane_follow            // 车道线保持
	├── narrow_street_u_turn   // 狭窄掉头
	├── scenario.cc
	├── scenario.h
	├── scenario_manager.cc
	├── scenario_manager.h
	├── side_pass             // 超车
	├── stage.cc
	├── stage.h
	├── stop_sign             // 停止
	├── traffic_light         // 红绿灯
	├── util
	└── valet_parking         // 代客泊车



	// 场景转换
	场景转换的实现在"scenario_manager.cc"中，其中实现了场景注册，创建场景和更新场景的功能。

	bool ScenarioManager::Init(
      const std::set<ScenarioConfig::ScenarioType>& supported_scenarios) {
	  RegisterScenarios();	 // 注册场景
	  default_scenario_type_ = ScenarioConfig::LANE_FOLLOW;
	  supported_scenarios_ = supported_scenarios;
	  current_scenario_ = CreateScenario(default_scenario_type_);   // 创建场景，默认为lane_follow
	  return true;
	}

	// 更新场景
	void ScenarioManager::Update(const common::TrajectoryPoint& ego_point, const Frame& frame) {
  	CHECK(!frame.reference_line_info().empty());
	  Observe(frame); // 保留当前帧
	  ScenarioDispatch(ego_point, frame); // 场景分发
	}

	// 通过一个有限状态机，决定当前的场景
	// 可以看到，每次切换场景必须是从默认场景(LANE_FOLLOW)开始，即每次场景切换之后都会回到默认场景。
	void ScenarioManager::ScenarioDispatch(const common::TrajectoryPoint& ego_point,  const Frame& frame) {

	}


	//场景运行
	/modules/planning/scenarios/scenario.cc
	场景的执行在"scenario.cc"和对应的场景目录中，实际上每个场景又分为一个或者多个阶段(stage)，每个阶段又由不同的任务(task)组成。执行一个场景，就是顺序执行不同阶段的不同任务。 

	下面我们来看一个具体的例子，Scenario对应的stage和task在"planning/conf/scenario"中:
			// Scenario对应的Stage
		scenario_type: SIDE_PASS
		stage_type: SIDE_PASS_APPROACH_OBSTACLE
		stage_type: SIDE_PASS_GENERATE_PATH
		stage_type: SIDE_PASS_STOP_ON_WAITPOINT
		stage_type: SIDE_PASS_DETECT_SAFETY
		stage_type: SIDE_PASS_PASS_OBSTACLE
		stage_type: SIDE_PASS_BACKUP

		// Stage对应的Task
		stage_type: SIDE_PASS_APPROACH_OBSTACLE
		enabled: true
		task_type: DP_POLY_PATH_OPTIMIZER
		task_type: PATH_DECIDER
		task_type: SPEED_BOUNDS_PRIORI_DECIDER
		task_type: DP_ST_SPEED_OPTIMIZER
		task_type: SPEED_DECIDER
		task_type: SPEED_BOUNDS_FINAL_DECIDER
		task_type: QP_SPLINE_ST_SPEED_OPTIMIZER

		// 以此类推


		由于Scenario都是顺序执行，只需要判断这一阶段是否结束，然后转到下一个阶段就可以了。具体的实现在：
		modules/planning/scenarios/scenario.cc

		Scenario::ScenarioStatus Scenario::Process(const common::TrajectoryPoint& planning_init_point, Frame* frame) {
		  if (current_stage_ == nullptr) {
		    AWARN << "Current stage is a null pointer.";
		    return STATUS_UNKNOWN;
		  }
		  // 如果当前阶段完成，则退出
		  if (current_stage_->stage_type() == ScenarioConfig::NO_STAGE) {
		    scenario_status_ = STATUS_DONE;
		    return scenario_status_;
		  }
		   // 进入下一阶段执行或者错误处理
		  auto ret = current_stage_->Process(planning_init_point, frame);
		  switch (ret) {
		    case Stage::ERROR: {
		      AERROR << "Stage '" << current_stage_->Name() << "' returns error";
		      scenario_status_ = STATUS_UNKNOWN;
		      break;
		    }
		    case Stage::RUNNING: {
		      scenario_status_ = STATUS_PROCESSING;
		      break;
		    }
		    case Stage::FINISHED: {
		      auto next_stage = current_stage_->NextStage();
		      if (next_stage != current_stage_->stage_type()) {
		        AINFO << "switch stage from " << current_stage_->Name() << " to "
		              << ScenarioConfig::StageType_Name(next_stage);
		        if (next_stage == ScenarioConfig::NO_STAGE) {
		          scenario_status_ = STATUS_DONE;
		          return scenario_status_;
		        }
		        if (stage_config_map_.find(next_stage) == stage_config_map_.end()) {
		          AERROR << "Failed to find config for stage: " << next_stage;
		          scenario_status_ = STATUS_UNKNOWN;
		          return scenario_status_;
		        }
		        current_stage_ = CreateStage(*stage_config_map_[next_stage]);
		        if (current_stage_ == nullptr) {
		          AWARN << "Current stage is a null pointer.";
		          return STATUS_UNKNOWN;
		        }
		      }
		      if (current_stage_ != nullptr &&
		          current_stage_->stage_type() != ScenarioConfig::NO_STAGE) {
		        scenario_status_ = STATUS_PROCESSING;
		      } else {
		        scenario_status_ = STATUS_DONE;
		      }
		      break;
		    }
		    default: {
		      AWARN << "Unexpected Stage return value: " << ret;
		      scenario_status_ = STATUS_UNKNOWN;
		    }
		  }
		  return scenario_status_;
		}


	我们接着看一下Stage中"Process"的执行：
	modules/planning/scenarios/lane_follow/lane_follow_stage.cc

	Stage::StageStatus LaneFollowStage::Process(const TrajectoryPoint& planning_start_point, Frame* frame) {
	  bool has_drivable_reference_line = false;

	  ADEBUG << "Number of reference lines:\t" << frame->mutable_reference_line_info()->size();

	  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
	    if (has_drivable_reference_line) {
	      reference_line_info.SetDrivable(false);
	      break;
	    }
	    // 根据参考线规划 // LANE_FOLLOW中的PlanOnReferenceLine
	    auto cur_status =  PlanOnReferenceLine(planning_start_point, frame, &reference_line_info);

	    if (cur_status.ok()) {
	      if (reference_line_info.IsChangeLanePath()) {
	        ADEBUG << "reference line is lane change ref.";
	        if (reference_line_info.Cost() < kStraightForwardLineCost &&  LaneChangeDecider::IsClearToChangeLane(&reference_line_info)) {
	          has_drivable_reference_line = true;
	          reference_line_info.SetDrivable(true);
	          ADEBUG << "\tclear for lane change";
	        } else {
	          reference_line_info.SetDrivable(false);
	          ADEBUG << "\tlane change failed";
	        }
	      } else {
	        ADEBUG << "reference line is NOT lane change ref.";
	        has_drivable_reference_line = true;
	      }
	    } else {
	      reference_line_info.SetDrivable(false);
	    }
	  }
	  return has_drivable_reference_line ? StageStatus::RUNNING : StageStatus::ERROR;
	}


// LANE_FOLLOW中的PlanOnReferenceLine
Status LaneFollowStage::PlanOnReferenceLine(
    const TrajectoryPoint& planning_start_point, Frame* frame,
    ReferenceLineInfo* reference_line_info) {
  if (!reference_line_info->IsChangeLanePath()) {
    reference_line_info->AddCost(kStraightForwardLineCost);
  }
  ADEBUG << "planning start point:" << planning_start_point.DebugString();

  auto ret = Status::OK();
    // 顺序执行stage中的任务
   for (auto* optimizer : task_list_) {
    const double start_timestamp = Clock::NowInSeconds();
    // 任务
    ret = optimizer->Execute(frame, reference_line_info);
    if (!ret.ok()) {
      AERROR << "Failed to run tasks[" << optimizer->Name()
             << "], Error message: " << ret.error_message();
      break;
    }
    const double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "after optimizer " << optimizer->Name() << ":"
           << reference_line_info->PathSpeedDebugString();
    ADEBUG << optimizer->Name() << " time spend: " << time_diff_ms << " ms.";

    RecordDebugInfo(reference_line_info, optimizer->Name(), time_diff_ms);
  }

  RecordObstacleDebugInfo(reference_line_info);

  // check path and speed results for path or speed fallback
  reference_line_info->set_trajectory_type(ADCTrajectory::NORMAL);
  if (!ret.ok()) {
    PlanFallbackTrajectory(planning_start_point, frame, reference_line_info);
  }

  DiscretizedTrajectory trajectory;
  if (!reference_line_info->CombinePathAndSpeedProfile(
          planning_start_point.relative_time(),
          planning_start_point.path_point().s(), &trajectory)) {
    std::string msg("Fail to aggregate planning trajectory.");
    AERROR << msg;
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  // determine if there is a destination on reference line.
  double dest_stop_s = -1.0;
  // 增加障碍物的代价
  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->LongitudinalDecision().has_stop() &&
        obstacle->LongitudinalDecision().stop().reason_code() ==
            STOP_REASON_DESTINATION) {
      SLPoint dest_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                  reference_line_info->reference_line());
      dest_stop_s = dest_sl.s();
    }
  }

  for (const auto* obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }
    if (obstacle->LongitudinalDecision().has_stop()) {
      bool add_stop_obstacle_cost = false;
      if (dest_stop_s < 0.0) {
        add_stop_obstacle_cost = true;
      } else {
        SLPoint stop_sl = GetStopSL(obstacle->LongitudinalDecision().stop(),
                                    reference_line_info->reference_line());
        if (stop_sl.s() < dest_stop_s) {
          add_stop_obstacle_cost = true;
        }
      }
      if (add_stop_obstacle_cost) {
        constexpr double kRefrenceLineStaticObsCost = 1e3;
        reference_line_info->AddCost(kRefrenceLineStaticObsCost);
      }
    }
  }

  if (FLAGS_enable_trajectory_check) {
    if (ConstraintChecker::ValidTrajectory(trajectory) !=
        ConstraintChecker::Result::VALID) {
      std::string msg("Current planning trajectory is not valid.");
      AERROR << msg;
      return Status(ErrorCode::PLANNING_ERROR, msg);
    }
  }
// 返回参考线
  reference_line_info->SetTrajectory(trajectory);
  reference_line_info->SetDrivable(true);
  return Status::OK();
}

上面是用"LaneFollowStage"中的"PlanOnReferenceLine"来举例子，不同场景中的"PlanOnReferenceLine"实现可能也不一样，这样设计的好处是，当发现一个场景有问题，需要修改不会影响到其他的场景。同时也可以针对不同场景做优化，比通用的规划更加适合单独的场景。每种场景都有一个专门的目录来进行优化。


6. // task
	我们先看Task的目录结构：

	.
	├── BUILD
	├── deciders       // 决策器
	├── optimizers     // 优化器
	├── rss
	├── smoothers     // 平滑器
	├── task.cc
	├── task_factory.cc
	├── task_factory.h
	└── task.h


	可以看到每个Task都可以对应到一个决策器或者优化器（平滑器不作为Task，单独作为一个类）
	每个Task都实现了"Execute"方法，而每个决策器和优化器都继承至Task类
	Task类的生成用到了设计模式的工厂模式，通过"TaskFactory"类生产不同的Task类