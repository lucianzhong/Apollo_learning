Reference:
https://paul.pub/apollo-planning/


1.
 Apollo系统中的Planning模块实际上是整合了决策和规划两个功能，该模块是自动驾驶系统中最核心的模块之一（另外三个核心模块是：定位，感知和控制）

	这其中主要的组件包括：
		Apollo FSM：一个有限状态机，与高清地图确定车辆状态给定其位置和路线。
		Planning Dispatcher：根据车辆的状态和其他相关信息，调用合适的Planner。
		Planner：获取所需的上下文数据和其他信息，确定相应的车辆意图，执行该意图所需的规划任务并生成规划轨迹。它还将更新未来作业的上下文。
		Deciders和Optimizers：一组实现决策任务和各种优化的无状态库。优化器特别优化车辆的轨迹和速度。决策者是基于规则的分类决策者，他们建议何时换车道、何时停车、何时爬行（慢速行进）或爬行何时完成。
		黄色框：这些框被包含在未来的场景和/或开发人员中，以便基于现实世界的驱动用例贡献他们自己的场景。

	PncMap：全称是Planning and Control Map。这个部分的实现并不在Planning内部，而是位于/modules/map/pnc_map/目录下。但是由于该实现与Planning模块紧密相关，因此这里放在一起讨论。该模块的主要作用是：根据Routing提供的数据，生成Planning模块需要的路径信息。
	Frame：Frame中包含了Planning一次计算循环中需要的所有数据。例如：地图，车辆状态，参考线，障碍物信息等等。ReferenceLine是车辆行驶的参考线，TrafficDecider与交通规则相关，这两个都是Planning中比较重要的子模块，因此会在下文中专门讲解。
	EM Planner：下文中我们会看到，Apollo系统中内置了好几个Planner，但目前默认使用的是EM Planner，这也是专门为开放道路设计的。该模块的实现可以说是整个Planning模块的灵魂所在。因此其算法值得专门用另外一篇文章来讲解。
		读者也可以阅读其官方论文来了解：Baidu Apollo EM Motion Planner。


2. 基础数据结构:

	这些数据结构集中定义在两个地方：
	proto目录：该目录下都是通过Protocol Buffers格式定义的结构。这些结构会在编译时生成C++需要的文件。这些结构没有业务逻辑，就是专门用来存储数据的。（实际上不只是Planning，几乎每个大的模块都会有自己的proto文件夹。）
	common目录：这里是C++定义的数据结构。很显然，通过C++定义数据结构的好处是这些类的实现中可以包含一定的处理逻辑。


	apollo/modules/planning/proto/
	├── auto_tuning_model_input.proto
	├── auto_tuning_raw_feature.proto
	├── decider_config.proto
	├── decision.proto
	├── dp_poly_path_config.proto
	├── dp_st_speed_config.proto
	├── lattice_sampling_config.proto
	├── lattice_structure.proto
	├── navi_obstacle_decider_config.proto
	├── navi_path_decider_config.proto
	├── navi_speed_decider_config.proto
	├── pad_msg.proto
	├── planner_open_space_config.proto
	├── planning.proto
	├── planning_config.proto
	├── planning_internal.proto
	├── planning_stats.proto
	├── planning_status.proto
	├── poly_st_speed_config.proto
	├── poly_vt_speed_config.proto
	├── proceed_with_caution_speed_config.proto
	├── qp_piecewise_jerk_path_config.proto
	├── qp_problem.proto
	├── qp_spline_path_config.proto
	├── qp_st_speed_config.proto
	├── reference_line_smoother_config.proto
	├── side_pass_path_decider_config.proto
	├── sl_boundary.proto
	├── spiral_curve_config.proto
	├── st_boundary_config.proto
	├── traffic_rule_config.proto
	└── waypoint_sampler_config.proto

	自动生成C++需要的数据结构
	可以方便的从文本文件导入和导出。下文将看到，Planning模块中有很多配置文件就是和这里的proto结构相对应的



	//common目录
	apollo/modules/planning/common/
	├── change_lane_decider.h
	├── decision_data.h
	├── distance_estimator.h
	├── ego_info.h
	├── frame.h
	├── frame_manager.h
	├── indexed_list.h
	├── indexed_queue.h
	├── lag_prediction.h
	├── local_view.h
	├── obstacle.h
	├── obstacle_blocking_analyzer.h
	├── path
	│   ├── discretized_path.h
	│   ├── frenet_frame_path.h
	│   └── path_data.h
	├── path_decision.h
	├── planning_context.h
	├── planning_gflags.h
	├── reference_line_info.h
	├── speed
	│   ├── speed_data.h
	│   ├── st_boundary.h
	│   └── st_point.h
	├── speed_limit.h
	├── speed_profile_generator.h
	├── threshold.h
	├── trajectory
	│   ├── discretized_trajectory.h
	│   ├── publishable_trajectory.h
	│   └── trajectory_stitcher.h
	└── trajectory_info.h


		名称								说明
	EgoInfo类				包含了自车信息，例如：当前位置点，车辆状态，外围Box等。
	Frame类					包含了一次Planning计算循环中的所有信息。
	FrameManager类			Frame的管理器，每个Frame会有一个整数型id。
	LocalView类				Planning计算需要的输入，下文将看到其定义。
	Obstacle类				描述一个特定的障碍物。障碍物会有一个唯一的id来区分。
	PlanningContext类		Planning全局相关的信息，例如：是否正在变道。这是一个单例。
	ReferenceLineInfo类		车辆行驶的参考线，下文会专门讲解。
	path文件夹				描述车辆路线信息。包含：PathData，DiscretizedPath，FrenetFramePath三个类。
	speed文件夹				描述车辆速度信息。包含SpeedData，STPoint，StBoundary三个类。
	trajectory文件夹			描述车辆轨迹信息。包含DiscretizedTrajectory，PublishableTrajectory，TrajectoryStitcher三个类。
	planning_gflags.h		定义了模块需要的许多常量，例如各个配置文件的路径。


3. 模块配置
	Planning模块中有很多处的逻辑是通过配置文件控制的。通过将这部分内容从代码中剥离，可以方便的直接对配置文件进行调整，而不用编译源代码。这对于系统调试和测试来说，是非常方便的。
	Apollo系统中，很多模块都是类似的设计。因此每个模块都会将配置文件集中放在一起，也就是每个模块下的conf目录。

	Planning模块的配置文件如下所示：

	apollo/modules/planning/conf/
	├── adapter.conf
	├── cosTheta_smoother_config.pb.txt
	├── navi_traffic_rule_config.pb.txt
	├── planner_open_space_config.pb.txt
	├── planning.conf
	├── planning_config.pb.txt
	├── planning_config_navi.pb.txt
	├── planning_navi.conf
	├── qp_spline_smoother_config.pb.txt
	├── scenario
	│   ├── lane_follow_config.pb.txt
	│   ├── side_pass_config.pb.txt
	│   ├── stop_sign_unprotected_config.pb.txt
	│   ├── traffic_light_protected_config.pb.txt
	│   └── traffic_light_unprotected_right_turn_config.pb.txt
	├── spiral_smoother_config.pb.txt
	└── traffic_rule_config.pb.txt

	这里的绝大部分文件都是.pb.txt后缀的。因为这些文件是和上面提到的proto结构相对应的。因此可以直接被proto文件生成的数据结构读取。
	读者暂时不用太在意这些文件的内容。随着对于Planning模块实现的熟悉，再回过来看这些配置文件，就很容易理解每个配置文件的作用了。下文中，对一些关键内容我们会专门提及。





4. 

	Cyber RT以组件的方式来管理各个模块，组件的实现会基于该框架提供的基类：apollo::cyber::Component。

	Planning模块自然也不例外。其实现类是下面这个
	modules/planning/planning_component.h

	class PlanningComponent final:public cyber::Component<prediction::PredictionObstacles, canbus::Chassis, localization::LocalizationEstimate> { }


在PlanningComponent的实现中，会根据具体的配置选择Planning的入口。Planning的入口通过PlanningBase类来描述的。
PlanningBase只是一个抽象类，该类有三个子类：
	OpenSpacePlanning
	NaviPlanning
	StdPlanning

bool PlanningComponent::Init() {
  if (FLAGS_open_space_planner_switchable) {
    planning_base_ = std::make_unique<OpenSpacePlanning>();
  } else {
    if (FLAGS_use_navigation_mode) {
      planning_base_ = std::make_unique<NaviPlanning>();
    } else {
      planning_base_ = std::make_unique<StdPlanning>();
    }
  }
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_planning_config_file, &config_)) << "failed to load planning config file " << FLAGS_planning_config_file;
  planning_base_->Init(config_);

	目前，FLAGS_open_space_planner_switchable和FLAGS_use_navigation_mode的配置都是false，因此最终的Planning入口类是：StdPlanning。


	所以接下来，我们只要关注StdPlanning的实现即可。在这个类中，下面这个方法是及其重要的：

	apollo/modules/planning/on_lane_planning.cc
	void OnLanePlanning::RunOnce( const LocalView& local_view, ADCTrajectory* const ptr_trajectory_pb) { }
	方法的注释已经说明得很清楚了：这是Planning模块的主体逻辑，会被timer以固定的间隔调用。每次调用就是一个规划周期



5.

	Planner的配置文件路径是在planning_gflags.cc中指定的,/modules/planning/common/planning_gflags.cc 
	// planning config file
	DEFINE_string(planning_config_file, "/apollo/modules/planning/conf/planning_config.pb.txt", "planning config file");


	modules/planning/conf/planning_config.pb.txt

	//设置了两个Planner，最终选择哪一个由下面这个函数
	standard_planning_config {
	  planner_type: planner_public_road_config//default
	  planner_type: OPEN_SPACE
	  planner_public_road_config {
	     scenario_type: LANE_FOLLOW
	     scenario_type: SIDE_PASS
	     scenario_type: STOP_SIGN_UNPROTECTED
	  }
	}

	//设置了两个Planner，最终选择哪一个由下面这个函数
	/modules/planning/planner/navi_planner_dispatcher.cc

	std::unique_ptr<Planner> NaviPlannerDispatcher::DispatchPlanner() {
	  PlanningConfig planning_config;
	  if (!apollo::cyber::common::GetProtoFromFile(FLAGS_planning_config_file, &planning_config)) {
	    return nullptr;
	  }
	  auto planner_type = PlannerType::NAVI;
	  if (planning_config.has_navigation_planning_config()) {
	    planner_type = planning_config.navigation_planning_config().planner_type(0);
	  }
	  return planner_factory_.CreateObject(planner_type);
	}





6. PublicRoadPlanner
	PublicRoadPlanner是目前默认的Planner，它实现了EM（Expectation Maximization）算法。
	Planner的算法实现依赖于两个输入：
		车辆自身状态：通过 TrajectoryPoint 描述。该结构中包含了车辆的位置，速度，加速度，方向等信息。
		当前环境信息：通过Frame描述。前面我们已经提到，Frame中包含了一次Planning计算循环中的所有信息。

	/modules/planning/planner/public_road/public_road_planner.cc

	Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point, Frame* frame, ADCTrajectory* ptr_computed_trajectory)


	/modules/planning/common/frame.h
	explicit Frame(uint32_t sequence_num, const LocalView &local_view, const common::TrajectoryPoint &planning_start_point, const common::VehicleState &vehicle_state, ReferenceLineProvider *reference_line_provider);


	/modules/planning/common/local_view.h
	LocalView类:Planning计算需要的输入，下文将看到其定义
	struct LocalView {
	  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;			//障碍物的预测信息
	  std::shared_ptr<canbus::Chassis> chassis;											//车辆底盘信息
	  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;		//大致定位信息
	  std::shared_ptr<perception::TrafficLightDetection> traffic_light;					//交通灯信息
	  std::shared_ptr<routing::RoutingResponse> routing;								//导航路由信息
	  bool is_new_routing = false;
	  std::shared_ptr<relative_map::MapMsg> relative_map;								//相对地图信息
	};


	对于每个Planner来说，其主要的逻辑都实现在Plan方法中:
	/modules/planning/planner/public_road/public_road_planner.cc
	Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point, Frame* frame, ADCTrajectory* ptr_computed_trajectory) {
	  DCHECK_NOTNULL(frame);
	  scenario_manager_.Update(planning_start_point, *frame);				//确定当前Scenario：因为Frame中包含了当前状态的所有信息，所以通过它就可以确定目前是处于哪一个场景下
	  scenario_ = scenario_manager_.mutable_scenario();
	  auto result = scenario_->Process(planning_start_point, frame);		//获取当前Scenario

	  if (FLAGS_enable_record_debug) {
	    auto scenario_debug = ptr_computed_trajectory->mutable_debug()->mutable_planning_data()->mutable_scenario();
	    scenario_debug->set_scenario_type(scenario_->scenario_type());
	    scenario_debug->set_stage_type(scenario_->GetStage());
	    scenario_debug->set_msg(scenario_->GetMsg());
	  }

	  if (result == scenario::Scenario::STATUS_DONE) {		//通过Scenario进行具体的处理
	    // only updates scenario manager when previous scenario's status is STATUS_DONE
	    scenario_manager_.Update(planning_start_point, *frame);		//如果处理成功，则再次通过ScenarioManager更新
	  } else if (result == scenario::Scenario::STATUS_UNKNOWN) {
	    return Status(common::PLANNING_ERROR, "scenario returned unknown");
	  }
	  return Status::OK();
	}



7. Scenario 场景分类
	Apollo3.5聚焦在三个主要的驾驶场景:
	1.车道保持
		车道保持场景是默认的驾驶场景，它不仅仅包含单车道巡航。同时也包含了：换道行驶,遵循基本的交通约定,基本转弯

	2. Side Pass
		在这种情况下，如果在自动驾驶车辆（ADC）的车道上有静态车辆或静态障碍物，并且车辆不能在不接触障碍物的情况下安全地通过车道，则执行以下策略：检查邻近车道是否接近通行,如果无车辆，进行绕行，绕过当前车道进入邻道,一旦障碍物安全通过，回到原车道上

    3.停止标识
		停止标识有两种分离的驾驶场景：
		1、未保护：在这种情况下，汽车预计会通过具有双向停车位的十字路口。因此，我们的ADC必须爬过并测量十字路口的交通密度，然后才能继续走上它的道路
		2、受保护：在此场景中，汽车预期通过具有四向停车位的十字路口导航。我们的ADC将必须对在它之前停下来的汽车进行测量，并在移动之前了解它在队列中的位置



8. // 场景实现

	//ScenarioManager：场景管理器类。负责注册，选择和创建Scenario
	/modules/planning/scenarios/scenario_manager.cc 
	//Scenario：描述一个特定的场景（例如：Side Pass）。该类中包含了CreateStage方法用来创建Stage。一个Scenario可能有多个Stage对象。在Scenario中会根据配置顺序依次调用Stage::Process方法。该方法的返回值决定了从一个Stage切换到另外一个Stage
	/modules/planning/scenarios/scenario.cc
	//Stage：如上面所说，一个Scenario可能有多个Stage对象。场景功能实现的主体逻辑通常是在Stage::Process方法中
	/modules/planning/scenarios/stage.cc


	// 场景配置
	所有场景都是通过配置文件来进行配置的。很显然，首先需要在proto文件夹中定义其结构,/modules/planning/conf

	// scenario configs
	message ScenarioConfig {
	  enum ScenarioType {
	    LANE_FOLLOW = 0;  // default scenario
	    CHANGE_LANE = 1;
	    SIDE_PASS = 2;  // go around an object when it blocks the road
	    APPROACH = 3;   // approach to an intersection
	    STOP_SIGN_PROTECTED = 4;
	    STOP_SIGN_UNPROTECTED = 5;
	    TRAFFIC_LIGHT_LEFT_TURN_PROTECTED = 6;
	    TRAFFIC_LIGHT_LEFT_TURN_UNPROTECTED = 7;
	    TRAFFIC_LIGHT_RIGHT_TURN_PROTECTED = 8;
	    TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED = 9;
	    TRAFFIC_LIGHT_GO_THROUGH = 10;
	  }

	  // StageType is a superset of stages from all scenarios.
	  // It is created to keep different scenarios have uniform config interface
	  enum StageType {
	    NO_STAGE = 0;

	    LANE_FOLLOW_DEFAULT_STAGE = 1;

	    STOP_SIGN_UNPROTECTED_PRE_STOP = 100;
	    STOP_SIGN_UNPROTECTED_STOP = 101;
	    STOP_SIGN_UNPROTECTED_CREEP = 102 ;
	    STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE = 103;

	    SIDE_PASS_APPROACH_OBSTACLE = 200;
	    SIDE_PASS_GENERATE_PATH= 201;
	    SIDE_PASS_STOP_ON_WAITPOINT = 202;
	    SIDE_PASS_DETECT_SAFETY = 203;
	    SIDE_PASS_PASS_OBSTACLE = 204;
	    SIDE_PASS_BACKUP = 205;

	    TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED_STOP = 300;
	    TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED_CREEP = 301 ;
	    TRAFFIC_LIGHT_RIGHT_TURN_UNPROTECTED_INTERSECTION_CRUISE = 302;
	  };

	  message StageConfig {
	    optional StageType stage_type = 1;
	    optional bool enabled = 2 [default = true];
	    // an ordered list of tasks that are used at runtime. Its order determines the runtime order of the tasks.
	    repeated TaskConfig.TaskType task_type = 3;
	    // an unordered task configurations
	    repeated TaskConfig task_config = 4;
	  }

	  optional ScenarioType scenario_type = 1;
	  oneof scenario_config {
	    ScenarioLaneFollowConfig lane_follow_config = 2;
	    ScenarioSidePassConfig side_pass_config = 3;
	    ScenarioStopSignUnprotectedConfig stop_sign_unprotected_config = 4;
	    ScenarioTrafficLightRightTurnUnprotectedConfig traffic_light_right_turn_unprotected_config = 5;
	  }
	  // a list of stages that are used at runtime.  The first one is default stage.
	  repeated StageType stage_type = 6;		//定义了ScenarioConfig结构，一个ScenarioConfig中可以包含多个StageConfig
	  // an unordered list of stage configs.
	  repeated StageConfig stage_config = 7;
	}

	// 场景注册
	/modules/planning/scenarios/scenario_manager.cc 
	前面我们已经提到，ScenarioManager负责场景的注册。实际上，注册的方式就是读取配置文件

	void ScenarioManager::RegisterScenarios() {
	  // lane_follow
	  CHECK(Scenario::LoadConfig( FLAGS_scenario_lane_follow_config_file, &config_map_[ScenarioConfig::LANE_FOLLOW]));
	  // bare_intersection
	  CHECK(Scenario::LoadConfig( FLAGS_scenario_bare_intersection_unprotected_config_file, &config_map_[ScenarioConfig::BARE_INTERSECTION_UNPROTECTED]));
	  // park_and_go
	  CHECK(Scenario::LoadConfig( FLAGS_scenario_park_and_go_config_file, &config_map_[ScenarioConfig::PARK_AND_GO]));
	  // pull_over
	  CHECK(Scenario::LoadConfig( FLAGS_scenario_pull_over_config_file, &config_map_[ScenarioConfig::PULL_OVER]));
	  // stop_sign
	  CHECK(Scenario::LoadConfig( FLAGS_scenario_stop_sign_unprotected_config_file, &config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]));
	  // traffic_light
	  CHECK(Scenario::LoadConfig( FLAGS_scenario_traffic_light_protected_config_file, &config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]));
	  CHECK(Scenario::LoadConfig( FLAGS_scenario_traffic_light_unprotected_left_turn_config_file, &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]));
	  CHECK(Scenario::LoadConfig( FLAGS_scenario_traffic_light_unprotected_right_turn_config_file, &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]));
	  // valet parking
	  CHECK(Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file, &config_map_[ScenarioConfig::VALET_PARKING]));
	}

	// 场景确定
	/modules/planning/scenarios/scenario_manager.cc 

	void ScenarioManager::Update(const common::TrajectoryPoint& ego_point, const Frame& frame) { //确定场景的依据是Frame数据
	  CHECK(!frame.reference_line_info().empty());
	  Observe(frame);
	  ScenarioDispatch(ego_point, frame);
	}


9. Frenet坐标系:
	最主要的原因是因为大部分的道路都不是笔直的，而是具有一定弯曲度的弧线
	相比于笛卡尔坐标系，Frenet坐标系明显地简化了问题。因为在公路行驶中，我们总是能够简单的找到道路的参考线（即道路的中心线），那么基于参考线的位置的表示就可以简单的使用纵向距离（即沿着道路方向的距离）和横向距离（即偏离参考线的距离）来描述

	



10.  ReferenceLine
	参考线是整个决策规划算法的基础。从前面的内容我们也看到了，在Planning模块的每个计算循环中，会先生成ReferencePath，然后在这个基础上进行后面的处理。例如：把障碍物投影到参考线上

	// ReferenceLineProvider:ReferenceLine由ReferenceLineProvider专门负责生成


	// 创建ReferenceLine: ReferenceLine是在StdPlanning::InitFrame函数中生成的
	modules/planning/on_lane_planning.cc

	Status OnLanePlanning::InitFrame( const uint32_t sequence_num, const TrajectoryPoint& planning_start_point, const VehicleState& vehicle_state) {
	  frame_.reset( new Frame( sequence_num, local_view_, planning_start_point, vehicle_state, reference_line_provider_.get()));
	    if (frame_ == nullptr) {
		    return Status(ErrorCode::PLANNING_ERROR, "Fail to init frame: nullptr.");
		  }
		  std::list<ReferenceLine> reference_lines;
		  std::list<hdmap::RouteSegments> segments;
		  if (!reference_line_provider_->GetReferenceLines(&reference_lines, &segments)) {
		    std::string msg = "Failed to create reference line";
		    return Status(ErrorCode::PLANNING_ERROR, msg);
		  }
		  DCHECK_EQ(reference_lines.size(), segments.size());



	//ReferenceLineInfo
	在ReferenceLine之外，在common目录下还有一个结构：ReferenceLineInfo，这个结构才是各个模块实际用到数据结构，它其中包含了ReferenceLine，但还有其他更详细的数据。
	从ReferenceLine到ReferenceLineInfo是在Frame::CreateReferenceLineInfo中完成的。

	modules/planning/common/frame.cc

	bool Frame::CreateReferenceLineInfo( const std::list<ReferenceLine> &reference_lines, const std::list<hdmap::RouteSegments> &segments) {
	  reference_line_info_.clear();
	  auto ref_line_iter = reference_lines.begin();
	  auto segments_iter = segments.begin();
	  while (ref_line_iter != reference_lines.end()) {
	    if (segments_iter->StopForDestination()) {
	      is_near_destination_ = true;
	    }
	    reference_line_info_.emplace_back( vehicle_state_, planning_start_point_, *ref_line_iter, *segments_iter);
	    ++ref_line_iter;
	    ++segments_iter;
	  }

	  ReferenceLineInfo不仅仅包含了参考线信息，还包含了车辆状态，路径信息，速度信息，决策信息以及轨迹信息等。Planning模块的算法很多都是基于ReferenceLineInfo结构完成的。
	  apollo/modules/planning/scenarios/stage.cc

	  bool Stage::ExecuteTaskOnReferenceLine( const common::TrajectoryPoint& planning_start_point, Frame* frame) {
		  for (auto& reference_line_info : *frame->mutable_reference_line_info()) {
		    if (!reference_line_info.IsDrivable()) {
		      AERROR << "The generated path is not drivable";
		      return false;
		    }
		    auto ret = common::Status::OK();
		    for (auto* task : task_list_) {
		      ret = task->Execute(frame, &reference_line_info);
		      if (!ret.ok()) {
		        AERROR << "Failed to run tasks[" << task->Name() << "], Error message: " << ret.error_message();
		        break;
		      }
		    }
		    if (reference_line_info.speed_data().empty()) {
		      *reference_line_info.mutable_speed_data() = SpeedProfileGenerator::GenerateFallbackSpeedProfile();
		      reference_line_info.AddCost(kSpeedOptimizationFallbackCost);
		      reference_line_info.set_trajectory_type(ADCTrajectory::SPEED_FALLBACK);
		    } else {
		      reference_line_info.set_trajectory_type(ADCTrajectory::NORMAL);
		    }
		    DiscretizedTrajectory trajectory;
		    if (!reference_line_info.CombinePathAndSpeedProfile(planning_start_point.relative_time(), planning_start_point.path_point().s(), &trajectory)) {
		      AERROR << "Fail to aggregate planning trajectory.";
		      return false;
		    }
		    reference_line_info.SetTrajectory(trajectory);
		    reference_line_info.SetDrivable(true);
		    return true;
		  }
		  return true;
	}


11.
	// Smoother
	为了保证车辆轨迹的平顺，参考线必须是经过平滑的，目前Apollo中包含了这么几个Smoother用来做参考线的平滑：
	在实现中，Smoother用到了下面两个开源库：Ipopt Project,Eigen


	// TrafficRule
	行驶在城市道路上的自动驾驶车辆必定受到各种交通规则的限制。在正常情况下，车辆不应当违反交通规则。另外，交通规则通常是多种条例，不同城市和国家地区的交通规则可能是不一样的。如果处理好这些交通规则就是模块实现需要考虑的了。
	交通条例的生效并非是一成不变的，因此自然就需要有一个配置文件来进行配置。交通规则的配置文件是：modules/planning/conf/traffic_rule_config.pb.txt


	//TrafficDecider
	TrafficDecider是交通规则处理的入口，它负责读取上面这个配置文件，并执行交通规则的检查。在上文中我们已经看到，交通规则的执行是在StdPlanning::RunOnce中完成的

	modules/planning/traffic_rules/traffic_decider.cc

	Status TrafficDecider::Execute( Frame *frame, ReferenceLineInfo *reference_line_info) {
	  CHECK_NOTNULL(frame);
	  CHECK_NOTNULL(reference_line_info);

	  for (const auto &rule_config : rule_configs_.config()) {
	    if (!rule_config.enabled()) {  //遍历配置文件中的每一条交通规则，判断是否enable。
	      ADEBUG << "Rule " << rule_config.rule_id() << " not enabled";
	      continue;
	    }
	    auto rule = s_rule_factory.CreateObject(rule_config.rule_id(), rule_config); //创建具体的交通规则对象
	    if (!rule) {
	      AERROR << "Could not find rule " << rule_config.DebugString();
	      continue;
	    }
	    rule->ApplyRule(frame, reference_line_info); //执行该条交通规则逻辑
	    ADEBUG << "Applied rule " << TrafficRuleConfig::RuleId_Name(rule_config.rule_id());
	  }

	  BuildPlanningTarget(reference_line_info);   //在ReferenceLineInfo上合并处理所有交通规则最后的结果
	  return Status::OK();
	}


12.
	//Task
	一直到目前最新的Apollo 3.5版本为止，Planning模块最核心的算法就是其EM Planner（实现类是PublicRoadPlanner），而EM Planner最核心的就是其决策器和优化器。
	但由于篇幅所限，这部分内容本文不再继续深入。预计后面会再通过一篇文章来讲解。这里我们仅仅粗略的了解一下其实现结构。
	Planning中这部分逻辑实现位于tasks目录下，无论是决策器还是优化器都是从apollo::planning::Task继承的。

	modules/planning/tasks/deciders/decider.cc
	Decider::Decider(const TaskConfig& config) : Task(config) {}

	apollo::common::Status Decider::Execute( Frame* frame, ReferenceLineInfo* reference_line_info) {
	  Task::Execute(frame, reference_line_info);
	  return Process(frame, reference_line_info);
	}

	apollo::common::Status Decider::Execute(Frame* frame) {
	  Task::Execute(frame);
	  return Process(frame);
	}

	// Task配置:上文中我们已经提到，场景和Task配置是在一起的
	/modules/planning/conf/scenario

	一个Scenario可能有多个Stage，每个Stage可以指定相应的Task，下面是一个配置示例：
	/modules/planning/conf/scenario/lane_follow_config.pb.txt

	scenario_type: LANE_FOLLOW
	stage_type: LANE_FOLLOW_DEFAULT_STAGE
	stage_config: {
	  stage_type: LANE_FOLLOW_DEFAULT_STAGE
	  enabled: true
	  task_type: DECIDER_RULE_BASED_STOP
	  task_type: PATH_BOUND_DECIDER
	  task_type: PIECEWISE_JERK_PATH_OPTIMIZER
	  task_type: PATH_DECIDER
	  task_type: SPEED_BOUNDS_PRIORI_DECIDER
	  task_type: DP_ST_SPEED_OPTIMIZER
	  task_type: SPEED_DECIDER
	  task_type: SPEED_BOUNDS_FINAL_DECIDER
	  task_type: QP_SPLINE_ST_SPEED_OPTIMIZER
	  task_type: DECIDER_RSS

	  task_config: {
	    task_type: DECIDER_RULE_BASED_STOP
	  }
	  task_config: {
	    task_type: PATH_BOUND_DECIDER
	  }
	  task_config: {
	    task_type: PIECEWISE_JERK_PATH_OPTIMIZER
	  }
	  task_config: {
	    task_type: PATH_DECIDER
	  }
	  task_config: {
	    task_type: SPEED_BOUNDS_PRIORI_DECIDER
	  }
	  task_config: {
	    task_type: SPEED_BOUNDS_FINAL_DECIDER
	  }
	  task_config: {
	    task_type: DP_ST_SPEED_OPTIMIZER
	  }
	  task_config: {
	    task_type: SPEED_DECIDER
	  }
	  task_config: {
	    task_type: QP_SPLINE_ST_SPEED_OPTIMIZER
	  }
	  task_config: {
	    task_type: DECIDER_RULE_BASED_STOP//这里的task_type与Task实现类是一一对应的
	  }
	}


	// Task读取
	在构造Stage对象的时候，会读取这里的配置文件，然后创建相应的Task：

	Stage::Stage( const ScenarioConfig::StageConfig& config) : config_(config) {
	  name_ = ScenarioConfig::StageType_Name(config_.stage_type());
	  next_stage_ = config_.stage_type();
	  std::unordered_map<TaskConfig::TaskType, const TaskConfig*, std::hash<int>> config_map;
	  for (const auto& task_config : config_.task_config()) {
	    config_map[task_config.task_type()] = &task_config;
	  }
	  for (int i = 0; i < config_.task_type_size(); ++i) {
	    auto task_type = config_.task_type(i);
	    CHECK(config_map.find(task_type) != config_map.end()) << "Task: " << TaskConfig::TaskType_Name(task_type) << " used but not configured";
	    auto iter = tasks_.find(task_type);
	    if (iter == tasks_.end()) {
	      auto ptr = TaskFactory::CreateTask(*config_map[task_type]);
	      task_list_.push_back(ptr.get());
	      tasks_[task_type] = std::move(ptr);
	    } else {
	      task_list_.push_back(iter->second.get());
	    }
	  }
	}


	// Task执行
	Task的执行是在Stage::Process中，通过ExecuteTaskOnReferenceLine完成的