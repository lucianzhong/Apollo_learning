 Reference:
 https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/planning_arch.md

本文档将从代码层面讲解Apollo Planning规划模块的工作，规划模块是一个相对比较复杂的模块，他可以接受来自定位Localization、高精地图HD Map、交通灯模块Traffic Light，路由模块Routing以及预测模块Prediction的信息，
并且综合上述信息，给出无人驾驶汽车在当前环境下短期内的路径规划。注意这个规划仅仅是短期的。

Apollo针对路径规划的思想是，利用Routing模块产生一条当前位置到终点的路径，这条路径是很理想的，不会考虑实际的路况与环境信息。
Planning模块就是利用Routing产生的理想轨迹，结合当前路况下信号灯(Traffic Light模块得到)，障碍物短期内的运动轨迹(Prediction模块得到的LaneSequence及其概率)，来修正短期内无人驾驶汽车的路径(也就是决策)，通过Planning得到的一个修正的路径以后行驶到新的位置，
可以根据这个新的位置再次去查询Routing模块，得到新的路由轨迹，以此往复，直到到达终点。

这一节将从代码入手，详细讲解Planning模块的各个细节，小结与Prediction模块类似，首先介绍Planning模块中的各个组件，分析每个组件的功能和算法，最后整体分析Planning模块的工作流程。

Planning模块组件分为以下部分：
1. 车辆状态提供器: VehicleStateProvider
这是最简单的组件，他负责将定位Localization与底盘Chassis信息进行融合，得到当前车辆的状态

2. 规划与控制地图: Planning and Control Map, pnc map
pnc map其实和高精地图hd map没有关系，后者是专门为规划与控制模块设计的库函数，在hd map层次之上，负责一些地图相关信息的处理。例如查询车辆可能的形式路径(list)

3. 指引线提供器: Reference Line Provider
指引线提供器其实就是路径的生成，对于一系列的RouteSegments进行平滑与拼接，最终得到无人车行驶的指引线，也就是行驶路径。








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




	可借助GDB调试命令对上述执行流程进行更为深入的理解，例如TrafficLightProtectedScenario场景中TrafficLightProtectedStageApproach阶段的PathLaneBorrowDecider任务的调用堆栈如下，从下往上看，对于任意一个任务的调用流程一目了然：

#0  apollo::planning::PathLaneBorrowDecider::Process (this=0x7f8c28294460, frame=0x7f8c38029f70, 
    reference_line_info=0x7f8c3802b140) at modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.cc:39
#1  0x00007f8c0468b7c8 in apollo::planning::Decider::Execute (this=0x7f8c28294460, frame=0x7f8c38029f70, 
    reference_line_info=0x7f8c3802b140) at modules/planning/tasks/deciders/decider.cc:31
#2  0x00007f8c065c4a01 in apollo::planning::scenario::Stage::ExecuteTaskOnReferenceLine (this=0x7f8c28293eb0, 
    planning_start_point=..., frame=0x7f8c38029f70) at modules/planning/scenarios/stage.cc:96
#3  0x00007f8c06e721da in apollo::planning::scenario::traffic_light::TrafficLightProtectedStageApproach::Process (
    this=0x7f8c28293eb0, planning_init_point=..., frame=0x7f8c38029f70) at 
    modules/planning/scenarios/traffic_light/protected/stage_approach.cc:48
#4  0x00007f8c067f1732 in apollo::planning::scenario::Scenario::Process (
    this=0x7f8c2801bf20, planning_init_point=..., frame=0x7f8c38029f70) 
    at modules/planning/scenarios/scenario.cc:76
#5  0x00007f8c186e153a in apollo::planning::PublicRoadPlanner::Plan (
    this=0x23093de0, planning_start_point=..., frame=0x7f8c38029f70, 
    ptr_computed_trajectory=0x7f8b9a5fbed0) at modules/planning/planner/public_road/public_road_planner.cc:51
#6  0x00007f8c19ee5937 in apollo::planning::OnLanePlanning::Plan (
    this=0x237f3b0, current_time_stamp=1557133995.3679764, stitching_trajectory=std::vector of length 1, 
    capacity 1 = {...}, ptr_trajectory_pb=0x7f8b9a5fbed0)  at modules/planning/on_lane_planning.cc:436
#7  0x00007f8c19ee40fa in apollo::planning::OnLanePlanning::RunOnce (
    this=0x237f3b0, local_view=..., ptr_trajectory_pb=0x7f8b9a5fbed0) at modules/planning/on_lane_planning.cc:304
#8  0x00007f8c1ab0d494 in apollo::planning::PlanningComponent::Proc (
    this=0x1d0f310, prediction_obstacles=std::shared_ptr (count 4, weak 0) 0x7f8b840164f8, 
    chassis=std::shared_ptr (count 4, weak 0) 0x7f8b84018a08, 
    localization_estimate=std::shared_ptr (count 4, weak 0) 0x7f8b8400d3b8) at modules/planning/planning_component.cc:134
#9  0x00007f8c1abb46c4 in apollo::cyber::Component<apollo::prediction::PredictionObstacles, 
    apollo::canbus::Chassis, apollo::localization::LocalizationEstimate, apollo::cyber::NullType>::Process (this=0x1d0f310, 
    msg0=std::shared_ptr (count 4, weak 0) 0x7f8b840164f8, msg1=std::shared_ptr (count 4, weak 0) 0x7f8b84018a08, 
    msg2=std::shared_ptr (count 4, weak 0) 0x7f8b8400d3b8) at ./cyber/component/component.h:291
#10 0x00007f8c1aba2698 in apollo::cyber::Component<apollo::prediction::PredictionObstacles, 
    apollo::canbus::Chassis, apollo::localization::LocalizationEstimate, apollo::cyber::NullType>::Initialize(
    apollo::cyber::proto::ComponentConfig const&)::{lambda(std::shared_ptr<apollo::prediction::PredictionObstacles> const&,     
    std::shared_ptr<apollo::canbus::Chassis> const&, std::shared_ptr<apollo::localization::LocalizationEstimate> const&)#2}::operator()
    (std::shared_ptr<apollo::prediction::PredictionObstacles> const&, std::shared_ptr<apollo::canbus::Chassis> const&, 
    std::shared_ptr<apollo::localization::LocalizationEstimate> const&) const (__closure=0x2059a430, 
    msg0=std::shared_ptr (count 4, weak 0) 0x7f8b840164f8, msg1=std::shared_ptr (count 4, weak 0) 0x7f8b84018a08,     
    msg2=std::shared_ptr (count 4, weak 0) 0x7f8b8400d3b8) at ./cyber/component/component.h:378
#11 0x00007f8c1abb4ad2 in apollo::cyber::croutine::RoutineFactory apollo::cyber::croutine::CreateRoutineFactory
    <apollo::prediction::PredictionObstacles, apollo::canbus::Chassis, apollo::localization::LocalizationEstimate, 
    apollo::cyber::Component<apollo::prediction::PredictionObstacles, apollo::canbus::Chassis, 
    apollo::localization::LocalizationEstimate, apollo::cyber::NullType>::Initialize(
    apollo::cyber::proto::ComponentConfig const&)::{lambda(std::shared_ptr<apollo::prediction::PredictionObstacles> const&, 
    std::shared_ptr<apollo::canbus::Chassis> const&, std::shared_ptr<apollo::localization::LocalizationEstimate> const&)#2}&>
    (apollo::cyber::Component<apollo::prediction::PredictionObstacles, apollo::canbus::Chassis, 
    apollo::localization::LocalizationEstimate, apollo::cyber::NullType>::Initialize(apollo::cyber::proto::ComponentConfig const&)::
    {lambda(std::shared_ptr<apollo::prediction::PredictionObstacles> const&, std::shared_ptr<apollo::canbus::Chassis> const&, 
    std::shared_ptr<apollo::localization::LocalizationEstimate> const&)#2}&, 
    std::shared_ptr<apollo::cyber::data::DataVisitor<apollo::prediction::PredictionObstacles, 
    apollo::canbus::Chassis, apollo::localization::LocalizationEstimate, apollo::cyber::NullType> > const&)::
    {lambda()#1}::operator()() const::{lambda()#1}::operator()() const (__closure=0x2059a420) at ./cyber/croutine/routine_factory.h:108
#12 0x00007f8c1ac0466a in std::_Function_handler<void (), apollo::cyber::croutine::RoutineFactory 
apollo::cyber::croutine::CreateRoutineFactory<apollo::prediction::PredictionObstacles, apollo::canbus::Chassis, apollo::localization::LocalizationEstimate, 
apollo::cyber::Component<apollo::prediction::PredictionObstacles, apollo::canbus::Chassis, apollo::localization::LocalizationEstimate, 
apollo::cyber::NullType>::Initialize(apollo::cyber::proto::ComponentConfig const&)::{lambda(std::shared_ptr<apollo::prediction::PredictionObstacles> const&, 
std::shared_ptr<apollo::canbus::Chassis> const&, std::shared_ptr<apollo::localization::LocalizationEstimate> const&)#2}&>
(apollo::cyber::Component<apollo::prediction::PredictionObstacles, apollo::canbus::Chassis, apollo::localization::LocalizationEstimate, 
apollo::cyber::NullType>::Initialize(apollo::cyber::proto::ComponentConfig const&)::{lambda(std::shared_ptr<apollo::prediction::PredictionObstacles> const&, 
std::shared_ptr<apollo::canbus::Chassis> const&, std::shared_ptr<apollo::localization::LocalizationEstimate> const&)#2}&, 
std::shared_ptr<apollo::cyber::data::DataVisitor<apollo::prediction::PredictionObstacles, apollo::canbus::Chassis, apollo::localization::LocalizationEstimate, 
apollo::cyber::NullType> > const&)::{lambda()#1}::operator()() const::{lambda()#1}>::_M_invoke(std::_Any_data const&) (__functor=...) at 
/usr/include/c++/4.8/functional:2071
#13 0x00007f8c5f5b86e8 in std::function<void ()>::operator()() const (this=0x205f1160) at /usr/include/c++/4.8/functional:2471
#14 0x00007f8c57560cbc in apollo::cyber::croutine::CRoutine::Run (this=0x205f1148) at ./cyber/croutine/croutine.h:143
#15 0x00007f8c5755ff55 in apollo::cyber::croutine::(anonymous namespace)::CRoutineEntry (arg=0x205f1148) a
 ———————————————— 
版权声明：本文为CSDN博主「知行合一2018」的原创文章，遵循CC 4.0 by-sa版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/davidhopper/article/details/89360385



2. 在Docker内部使用GDB调试
gdb -q bazel-bin/modules/map/relative_map/navigation_lane_test
1
进入GDB调试界面后，使用l命令查看源代码，使用b 138在源代码第138行（可根据需要修改为自己所需的代码位置 ）设置断点，使用r命令运行navigation_lane_test程序，进入断点暂停后，使用p navigation_lane_查看当前变量值（可根据需要修改为其他变量名），使用n单步调试一条语句，使用s单步调试进入函数内部，使用c继续执行后续程序。如果哪个部分测试通不过，调试信息会立刻告诉你具体原因，可使用bt查看当前调用堆栈。
 ———————————————— 
版权声明：本文为CSDN博主「知行合一2018」的原创文章，遵循CC 4.0 by-sa版权协议，转载请附上原文出处链接及本声明。
原文链接：https://blog.csdn.net/davidhopper/article/details/82589722