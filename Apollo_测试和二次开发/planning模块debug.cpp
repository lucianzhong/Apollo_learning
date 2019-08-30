1. 

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
#15 0x00007f8c5755ff55 in apollo::cyber::croutine::(anonymous namespace)::CRoutineEntry (arg=0x205f1148) at cyber/croutine/croutine.cc:43

