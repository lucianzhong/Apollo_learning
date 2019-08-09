 1.
 规划（Planning）模块位于命名空间：apollo::planning，其作用在于构建无人车从起点到终的局部行驶路径，
 具体而言，就是给定导航地图、导航路径、当前定位点、车辆状态（包括：位置、速度、加速度、底盘）、 周边目标的感知及预测信息（如交通标志和障碍物等），规划模块计算出可供控制模块（Controller）执行的一条安全且舒适的行驶路径。
 注意，规划模块输出的路径是局部路径而非全局路径。
 举个简单示例加以说明，假如无人车需从长沙智能驾驶研究院行驶至长沙高铁南站，首先需借助Routing（路由寻径）模块输出全局导航路径，接下来才是规划模块基于全局导航路径进行一小段、一小段具体行驶路径的规划


2.
 Planning功能模块的启动命令为：
	/apollo/bazel-bin/cyber/mainboard -p compute_sched -d /apollo/modules/planning/dag/planning.dag
	
	-p compute_sched表明使用配置文件/apollo/cyber/conf/compute_sched.conf进行任务调度，该参数可忽略
	-d /apollo/modules/planning/dag/planning.dag表明动态加载的是Planning模块
	Planning模块的主入口为：/apollo/cyber/mainboard/mainboard.cc


3. /apollo/cyber/mainboard/mainboard.cc

	int main(int argc, char** argv) {
	  google::SetUsageMessage("we use this program to load dag and run user apps.");

	  // parse the argument
	  ModuleArgument module_args;
	  module_args.ParseArgument(argc, argv);   //main函数十分简单，首先是解析参数

	  // initialize cyber
	  apollo::cyber::Init(argv[0]);			// 初始化cyber环境

	  // start module
	  ModuleController controller(module_args);  //接下来创建一个ModuleController类对象controller
	  if (!controller.Init()) {  				//调用controller.Init()启动相关功能模块,ModuleController::Init()函数十分简单，内部调用了ModuleController::LoadAll()函数
	    controller.Clear();
	    AERROR << "module start error.";
	    return -1;
	  }

	  apollo::cyber::WaitForShutdown();		// 进入Cyber RT的消息循环，直到等待cyber::WaitForShutdown()返回
	  controller.Clear();
	  AINFO << "exit mainboard.";

	  return 0;
	}



4. cyber/mainboard/module_controller.cc

	bool ModuleController::LoadAll() {
	  const std::string work_root = common::WorkRoot();
	  const std::string current_path = common::GetCurrentPath();
	  const std::string dag_root_path = common::GetAbsolutePath(work_root, "dag");

	  for (auto& dag_conf : args_.GetDAGConfList()) {
	    std::string module_path = "";
	    if (dag_conf == common::GetFileName(dag_conf)) {
	      // case dag conf argument var is a filename
	      module_path = common::GetAbsolutePath(dag_root_path, dag_conf);
	    } else if (dag_conf[0] == '/') {
	      // case dag conf argument var is an absolute path
	      module_path = dag_conf;
	    } else {
	      // case dag conf argument var is a relative path
	      module_path = common::GetAbsolutePath(current_path, dag_conf);
	      if (!common::PathExists(module_path)) {
	        module_path = common::GetAbsolutePath(work_root, dag_conf);
	      }
	    }
	    AINFO << "Start initialize dag: " << module_path;
	    if (!LoadModule(module_path)) {
	      AERROR << "Failed to load module: " << module_path;
	      return false;
	    }
	  }
	  return true;
	}

	//上述函数处理一个dag_conf配置文件循环，读取配置文件中的所有dag_conf，并逐一调用bool ModuleController::LoadModule(const std::string& path)函数加载功能模块。


// apollo::planning::PlanningComponent类对象的创建过程


5.Cyber RT使用工厂设计模式创建apollo::planning::PlanningComponent类对象


6. apollo::planning::PlanningComponent类的注册过程




7. apollo::planning::PlanningComponent类对象的动态创建过程










//具体规划算法分析

8.   Apollo 3.5将规划分为两种模式： OnLanePlanning（车道规划，可用于城区及高速公路各种复杂道路）
								NaviPlanning（导航规划，主要用于高速公路）
     
     根据Apollo团队的最新开发思路，今后只会保留一个规划算法：PublicRoadPlanner

	包含四种具体规划算法：
	   1. PublicRoadPlanner（即以前的EMPlanner，是Apollo 3.5的主用规划算法）
	   2. LatticePlanner（Apollo 3.5的第二重要规划算法，成熟度不足，里面的一些优秀算法思想将被融合到PublicRoadPlanner中，今后该算法将不再维护）
	   3. NaviPlanner（百度美研与长沙智能驾驶研究院合作开发，主要用于高速公路场景）
	   4. RTKPlanner（循迹算法，一般不用。如需循迹，可使用Python脚本程序modules/tools/record_play/rtk_player.py）


9. 场景(Scenario):
    
    每个场景又包含若干个阶段(Stage)，在每个阶段均使用若干个任务(Task)生成局部行驶轨迹.
    基于场景(Scenario)、阶段(Stage)和任务(Task)的理念进行规划，优点是能合理有效地应对每种场景，易于扩充，并且基于配置文件动态增减场景、阶段及使用的任务，灵活性强；缺点是可能会遗漏一些特殊场景，但可通过不断扩充新的场景加以解决

	PublicRoadPlanner算法从Routing模块输出的高精地图Lane序列获得全局导航路径，基于场景(Scenario)的理念进行局部行驶轨迹规划。
	具体而言，将公共道路行驶划分为

			 1.BareIntersectionUnprotectedScenario(裸露交叉路口无保护场景，即没有红绿灯及交通标志的交叉路口场景，感谢Apollo美研团队Yifei Jiang老师的答疑)
			 2.LaneFollowScenario(跟车场景)
			 3.NarrowStreetUTurnScenario(狭窄街道调头场景，暂未实现)
			 4.SidePassScenario(侧向通行场景，即前方有停止车辆，借道绕行后再回原车道)
			 5.StopSignUnprotectedScenario(停止标志无保护场景)
			 6.TrafficLightProtectedScenario(红绿灯保护场景)
			 7.TrafficLightUnprotectedLeftTurnScenario(红绿灯无保护左转弯场景)
			 8.TrafficLightUnprotectedRightTurnScenario(红绿灯无保护右转弯场景)
			 9.PullOverScenario（靠边停车场景）
			 10.ValetParkingScenario(泊车场景)等多个场景

10.阶段(Stage)

	BareIntersectionUnprotectedScenario场景包含:
												BareIntersectionUnprotectedStageApproach、
												BareIntersectionUnprotectedStageIntersectionCruise两个阶段；


	LaneFollowScenario场景包含
								LaneFollowStage一个阶段；


	SidePassScenario场景包含

							StageApproachObstacle、
							StageDetectSafety、
							StageGeneratePath、
							StageStopOnWaitPoint、
							StagePassObstacle、
							StageBackup六个阶段；


	StopSignUnprotectedScenario场景包含
										StopSignUnprotectedStagePreStop、
										StopSignUnprotectedStageStop、
										StopSignUnprotectedStageCreep、
										StopSignUnprotectedStageIntersectionCruise四个阶段；



	TrafficLightProtectedScenario场景包含

										TrafficLightProtectedStageApproach、
										TrafficLightProtectedStageIntersectionCruise两个阶段；


	TrafficLightUnprotectedLeftTurnScenario场景包含
													TrafficLightUnprotectedLeftTurnStageStop、
													TrafficLightUnprotectedLeftTurnStageCreep、
													TrafficLightUnprotectedLeftTurnStageIntersectionCruise三个阶段；


	TrafficLightUnprotectedRightTurnScenario场景包含
														TrafficLightUnprotectedRightTurnStageStop、
														TrafficLightUnprotectedRightTurnStageCreep、
														TrafficLightUnprotectedRightTurnStageIntersectionCruise三个阶段；



	PullOverScenario场景包含PullOverStageApproach一个阶段（尚未开发完毕）；

	ValetParkingScenario场景包含StageApproachingParkingSpot、
								StageParking两个阶段。


11.任务(Task)

   任务分为决策（Decider）与优化（Optimizer ）两类，
   其中决策类任务包含PathLaneBorrowDecider,SpeedLimitDecider等（所有决策类任务均包含于modules/planning/tasks/deciders目录），

   优化类任务包含DpPolyPathOptimizer、DpStSpeedOptimizer等（所有优化类任务均包含于modules/planning/tasks/optimizers目录）。

   任意一个场景中的任意一个阶段均会利用上述两类任务的若干种。
   例如，BareIntersectionUnprotectedScenario场景中的BareIntersectionUnprotectedStageApproach阶段使用了PathDecider、SpeedBoundsDecider、DpStSpeedOptimizer、
        SpeedDecider、SpeedBoundsDecider等决策任务及DpPolyPathOptimizer、DpPolyPathOptimizer等优化任务（见配置文件modules/planning/conf/scenario/bare_intersection_unprotected_config.pb.txt）。










12. apollo/modules/planning/scenarios/scenario_manager.cc

	ScenarioManager::ScenarioDispatch使用Strategy设计模式来分派具体的场景

	void ScenarioManager::ScenarioDispatch(const common::TrajectoryPoint& ego_point, const Frame& frame) {

																											}



13.	与PublicRoadPlanner规划算法相关的有两处: 1. PublicRoadPlanner::Init
										 2. PublicRoadPlanner::Plan


PublicRoadPlanner::Init:  首先读取配置文件/apollo/modules/planning/conf/planning_config.pb.txt，获取所有支持的场景supported_scenarios，

						  然后调用scenario_manager_.Init(supported_scenarios);对这些场景进行初始化: 具体而言就是先调用ScenarioManager::RegisterScenarios函数将配置文件中的所有场景添加到场景管理器对象scenario::ScenarioManager scenario_manager_中，
						  																	   再调用ScenarioManager::CreateScenario函数，生成当前路况对应的场景对象std::unique_ptr<Scenario> current_scenario_。




PublicRoadPlanner::Plan函数内:首先调用函数 ScenarioManager:Update 根据实时路况更新当前场景对象std::unique_ptr<Scenario> current_scenario_，

                             接着调用scenario_->Process(planning_start_point, frame)语句实施具体的场景算法。如果Scenario::Process函数的返回值是scenario::Scenario::STATUS_DONE，表明当前场景状态已完成，则再次调用函数ScenarioManager::Update更新当前场景，否则继续处理当前场景并返回。


14. 看场景更新函数 ScenarioManager::Update 的代码

	apollo/modules/planning/scenarios/scenario_manager.cc

	void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             const Frame& frame) {
	  CHECK(!frame.reference_line_info().empty());
	  Observe(frame);
	  ScenarioDispatch(ego_point, frame);
	}
    
     ScenarioManager::Observe:
     用于更新first_encountered_overlap_map_（车辆沿着参考线行驶首次遇到的道路连接的键值对，key表示道路连接类型，例如：PNC_JUNCTION（用于规划控制模块的交叉路口，是一个由多条道路停止线包围而成的多边形区域，
         感谢Apollo美研团队Yifei Jiang老师的答疑）、SIGNAL（红绿灯） 、STOP_SIGN（停止标志）、YIELD_SIGN（让行标志），value表示对应的地图元素数据）
     


     ScenarioManager::ScenarioDispatch使用Strategy设计模式来分派具体的场景




15. Apollo系统中的Planning模块实际上是整合了决策和规划两个功能，该模块是自动驾驶系统中最核心的模块之一（另外三个核心模块是：定位，感知和控制）










































16. 基础数据结构:

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


17. 模块配置
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





18. /modules/planning/on_lane_planning.cc

	StdPlanning::RunOnce























19. Planner概述

	Planner的配置文件路径是在planning_gflags.cc中指定的,/modules/planning/common/planning_gflags.cc


		// planning config file
	DEFINE_string(planning_config_file,
	              "/apollo/modules/planning/conf/planning_config.pb.txt", "planning config file");


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





20. PublicRoadPlanner
	
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



21. Scenario 场景分类

	Apollo3.5聚焦在三个主要的驾驶场景:

	1.车道保持
		车道保持场景是默认的驾驶场景，它不仅仅包含单车道巡航。同时也包含了：
																	换道行驶
																	遵循基本的交通约定
																	基本转弯

	2. Side Pass
		在这种情况下，如果在自动驾驶车辆（ADC）的车道上有静态车辆或静态障碍物，并且车辆不能在不接触障碍物的情况下安全地通过车道，则执行以下策略：

																															检查邻近车道是否接近通行
																															如果无车辆，进行绕行，绕过当前车道进入邻道
																															一旦障碍物安全通过，回到原车道上

    3.停止标识
		停止标识有两种分离的驾驶场景：

		1、未保护：在这种情况下，汽车预计会通过具有双向停车位的十字路口。因此，我们的ADC必须爬过并测量十字路口的交通密度，然后才能继续走上它的道路
		2、受保护：在此场景中，汽车预期通过具有四向停车位的十字路口导航。我们的ADC将必须对在它之前停下来的汽车进行测量，并在移动之前了解它在队列中的位置



22. // 场景实现

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
	  CHECK(Scenario::LoadConfig(FLAGS_scenario_lane_follow_config_file,
	                             &config_map_[ScenarioConfig::LANE_FOLLOW]));

	  // bare_intersection
	  CHECK(Scenario::LoadConfig(
	      FLAGS_scenario_bare_intersection_unprotected_config_file,
	      &config_map_[ScenarioConfig::BARE_INTERSECTION_UNPROTECTED]));

	  // park_and_go
	  CHECK(Scenario::LoadConfig(FLAGS_scenario_park_and_go_config_file,
	                             &config_map_[ScenarioConfig::PARK_AND_GO]));

	  // pull_over
	  CHECK(Scenario::LoadConfig(FLAGS_scenario_pull_over_config_file,
	                             &config_map_[ScenarioConfig::PULL_OVER]));

	  // stop_sign
	  CHECK(Scenario::LoadConfig(
	      FLAGS_scenario_stop_sign_unprotected_config_file,
	      &config_map_[ScenarioConfig::STOP_SIGN_UNPROTECTED]));

	  // traffic_light
	  CHECK(Scenario::LoadConfig(
	      FLAGS_scenario_traffic_light_protected_config_file,
	      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_PROTECTED]));
	  CHECK(Scenario::LoadConfig(
	      FLAGS_scenario_traffic_light_unprotected_left_turn_config_file,
	      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN]));
	  CHECK(Scenario::LoadConfig(
	      FLAGS_scenario_traffic_light_unprotected_right_turn_config_file,
	      &config_map_[ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN]));

	  // valet parking
	  CHECK(Scenario::LoadConfig(FLAGS_scenario_valet_parking_config_file,
	                             &config_map_[ScenarioConfig::VALET_PARKING]));
	}





	// 场景确定
	/modules/planning/scenarios/scenario_manager.cc 

	void ScenarioManager::Update(const common::TrajectoryPoint& ego_point, const Frame& frame) { //确定场景的依据是Frame数据
	  CHECK(!frame.reference_line_info().empty());

	  Observe(frame);

	  ScenarioDispatch(ego_point, frame);
	}




23. Frenet坐标系:
	最主要的原因是因为大部分的道路都不是笔直的，而是具有一定弯曲度的弧线
	相比于笛卡尔坐标系，Frenet坐标系明显地简化了问题。因为在公路行驶中，我们总是能够简单的找到道路的参考线（即道路的中心线），那么基于参考线的位置的表示就可以简单的使用纵向距离（即沿着道路方向的距离）和横向距离（即偏离参考线的距离）来描述

	



24.  ReferenceLine
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




25. 决策规划模块负责生成车辆的行驶轨迹。要做到这一点，决策规划模块需要从宏观到局部经过三个层次来进行决策:

	1.第一个层次是Routing的搜索结果。Routing模块的输入是若干个按顺序需要达到的途径点（也可能只有一个起点和终点）。
	  Routing模块根据地图的拓扑结构搜索出可达的完整路线来，这个路线的长度可能是几公里甚至几百公里。因此这个是最为宏观的数据。另外，Routing的搜索结果是相对固定的。在没有障碍物的情况下，车辆会一直沿着原先获取到的Routing路线行驶。
	  只有当车辆驶出了原先规划的路线之外（例如：为了避障），才会重新发送请求给Routing模块，以重新计算路线。

	2. 第二个层次就是参考线。决策规划模块会实时的根据车辆的具体位置来计算参考线。参考线的计算会以Routing的路线为基础。但同时，参考线会考虑车辆周边的动态信息，例如：障碍物，交通规则等。参考线是包含车辆所在位置周边一定的范围，通常是几百米的长度。
	   Routing结果，它是较为局部的数据。

	3.层次是轨迹。轨迹是决策规划模块的最终输出结果。它的依据是参考线。在同一时刻，参考线可能会有多条，例如：在变道的时候，自车所在车道和目标车道都会有一条参考线。而轨迹，是在所有可能的结果中，综合决策和优化的结果，最终的唯一结果。
	  因此它是更为具体和局部的数据。轨迹不仅仅包含了车辆的路线，还包含了车辆行驶这条路线时的详细状态，例如：车辆的方向，速度，加速度等等。 


	  在Planning模块一文中我们已经提到：参考线是整个决策规划算法的基础。在Planning模块的每个计算循环中，都会先生成参考线，然后在这个基础上进行后面的处理，例如：交通规则逻辑，障碍物投影，路径优化，速度决策等等。可以说，参考线贯穿了整个Planning模块的实现。



26. pnc_map
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






27. ReferenceLine结构

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


28. std::vector<ReferencePoint>是一系列的点，点包含了位置的信息。因此这些点就是生成车辆行驶轨迹的基础数据:

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



29. 创建ReferenceLine

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



30.  ReferenceLineSmoother
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



























2. 交通标识牌 planning:

     Two inputs to planning: Prediction (预测信息:如交通标志和障碍物等) perception_obstacles.proto定义了表示车辆周围的障碍物的数据，车辆周围障碍物的数据由感知模块提供。traffic_light_detection定义了信号灯状态的数据
     					     Relative Map



    阿波罗高精地图:交通信号元素：红绿灯、道路标牌(道路指示牌)


     TrafficDecider与交通规则相关



     struct LocalView {
	  std::shared_ptr<prediction::PredictionObstacles> prediction_obstacles;
	  std::shared_ptr<canbus::Chassis> chassis;
	  std::shared_ptr<localization::LocalizationEstimate> localization_estimate;
	  std::shared_ptr<perception::TrafficLightDetection> traffic_light;	//交通灯信息
	  std::shared_ptr<routing::RoutingResponse> routing;
	  bool is_new_routing = false;
	  std::shared_ptr<relative_map::MapMsg> relative_map;
	};