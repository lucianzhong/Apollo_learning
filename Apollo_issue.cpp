
1. 
/*
	After a destination is given, routing module produces a sequence of end to end zigzag linear segments. Obviously, such lines can't be used for path planning. So a smoothing algorithm on the routing segment is done online to get a differentiable curve, which is called reference line, for the Planning module to work on.

	Don't quite understand what is your definition of passages. If you mean the smoothed reference line, then yes. As the current implementation on computing reference line is online, to save computation effort. only a definite horizon of reference line is computed each cycle, so there would be overlapped part. To deal with that, we use stitching to combine two together. Check reference_line_provider.h for more detail.



	 There's no "passage" in map. I guess you are referring to "passage" in routing.
	You can find the definition here:
	https://github.com/ApolloAuto/apollo/blob/master/modules/routing/proto/routing.proto
	Basically, lanes segment < passage < road

	A "passage" is a path generated based on a set of nodes/points. The algorithm can be found in
	https://github.com/ApolloAuto/apollo/blob/master/modules/routing/core/result_generator.cc

	There might be lane-changes inside one passage.


*/

/// https://github.com/ApolloAuto/apollo/issues/3843
Lane change is a planning option given by routing.
When there is a change lane region in routing, we will create two reference lines, one on current lane, and another on the target lane. The planning algorithm will try to make one planning trajectory on each reference line, and it will eventually select one based on traffic rules and cost functions.



The presence of obstacle does not affect how to create passage and reference line,
but the obstacles do affect the cost of driving on each reference line, then affect planning change lane.


lane change triggered by route from pnc map (This is true), or presence of obstacle on the lane (this feature not supported yet). Obstacle does not trigger change lane now, but it will affect change lane decision.

"reference line is changed" is not accurate. I would say "reference line is selected". Reference line is a static information derived from route and map. Multiple reference lines in a frame indicate that change lane is an option in the current frame. Planning selects reference line based on traffic condition and obstacles, and a lot of other logic.



// https://github.com/ApolloAuto/apollo/issues/4208
The current code flow is:
raw_reference_line -> anchor_points -> smoothed reference line.
Anchor points were designed as the middle layer, such that different smoothing algorithms can have a uniform constraint input (which is anchor point), without being bothered by how these anchor points are generated.

You may already notice that there is some logic in how to extract and shift these anchor points, and anchor points could come from a part of a previous reference line. This logic is orthogonal with the smoothing algorithm. It also allows us to generate anchor points from more resources than raw reference line, e.g., in places with out explicit lane info.



// https://github.com/ApolloAuto/apollo/issues/9434
	I do not understand the ROI mentioned in the open spacel planner, what is its function?
	Roi refers to "Region of Interest", describing the drivable area and target position














// https://github.com/ApolloAuto/apollo/issues/8439
Describe the bug
We are using Apollo 3.5 version and running into some issues with the planning module. After
finishing a routing request. The console on the dreamview shows "Routing Success! ". However, the "planning.INFO" tells me the routing is not ready. The consequence is that although there is a red line shows up on the map (reference_line), no planning messages are being published.

@yangydavid
if you launch planning module manually from terminal, you shall turn off planning module from DreamView, and then launch planning manually from terminal.
And, you need launch planning mode first, and then send routing request. So that planning module can receive routing response. Otherwise, when the planning module starts, the routing response was already gone. Although you can see routing red line in DreamView, the planning module actually never got it in time.

 Let us know if that is the case.



// https://github.com/ApolloAuto/apollo/issues/8354

I am using Apollo 3.0.
I would like to ask the question about the usage of "PathData" in EM planner.
In the first step of EM planner-- inside "DpPolyPathOptimizer(...)", there is a function "bool DPRoadGraph::FindPathTunnel(...)" to generate the "tunnel" of "PathData path_data". The resolution of the "frenet_frame" in the tunnel is set as "path_resolution / 2".

My question is: if I use my own planner to replace "DpPolyPathOptimizer(...)" and generate this "tunnel" with a sequence of "frenet_frame"s and the distance between two neighbouring "frenet_frame"s is a not a constant value (for example, the distance between two neighbouring "frenet_frame"s fluctuates around 0.4 to 0.6), will this new PathData generated by myself with non-constant "path_resolution" be still suitable for following steps in EM planner (e.g. PathDecider, DpStSpeedOptimizer(), SpeedDecider(), QpSplineStSpeedOptimizer(), PolyStSpeedOptimizer())?
@hezudao23 path planning is independent from speed planning. So the path generated by your customized or different resolution shall still work with the speed planning algorithms followed.
BTW, in Apollo 5.0, we are not using dp_poly_path_optimizer any more.



// https://github.com/ApolloAuto/apollo/issues/8406

// according to the position of the start plan point and the reference line,
// the path trajectory intercepted from the reference line is shifted on the
// y-axis to adc.
double dest_ref_line_y = path_points[0].y();

Path trajectory intercepted from the reference line should be lateral dist which is in the Frenet
coordinate, not the path_points[0].y() which is in the Cartesian coordinate.

@xmyqsh Path trajectory generated by the relative map is in FLU coordinate which origin is at the vehicle position. And FLU coordinate is not Frenet coordinate but Cartesian coordinate.
But at the start point of Path trajectory, the FLU coordinate is coincidence with the Frenet coordinate. So, the lateral dist is the path_points[0].y() exactly.




//https://github.com/ApolloAuto/apollo/issues/7314

lattice planner中对于静止障碍物的处理似乎有一些粗糙，我看到代码中表示，如果静止障碍物在当前车道上，就直接将其映射到ST图中，因此，如果车道上最右侧有一个小箱子（正常行驶不会撞到），我们应该不予考虑，但lattice中，我们似乎无法通过它。
你说的这个问题确实是在ST上无法越过的, 但是这种情况下就变道了, 这样投影就没问题. Lattice是基于高速公路的, 你说的那种情况发生的概率很小

void PathTimeGraph::SetStaticObstacle( const Obstacle* obstacle, const std::vector<PathPoint>& discretized_ref_points) {
  const Polygon2d& polygon = obstacle->PerceptionPolygon();
  std::string obstacle_id = obstacle->Id();
  SLBoundary sl_boundary =  ComputeObstacleBoundary(polygon.GetAllVertices(), discretized_ref_points);

  double left_width = FLAGS_default_reference_line_width * 0.5;
  double right_width = FLAGS_default_reference_line_width * 0.5;

  ptr_reference_line_info_->reference_line().GetLaneWidth(sl_boundary.start_s(), &left_width, &right_width);
  if (sl_boundary.start_s() > path_range_.second || sl_boundary.end_s() < path_range_.first || sl_boundary.start_l() > left_width || sl_boundary.end_l() < -right_width) {
    ADEBUG << "Obstacle [" << obstacle_id << "] is out of range.";
    return;
  }

  path_time_obstacle_map_[obstacle_id].set_id(obstacle_id);
  path_time_obstacle_map_[obstacle_id].set_bottom_left_point(SetPathTimePoint(obstacle_id, sl_boundary.start_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].set_bottom_right_point(SetPathTimePoint(obstacle_id, sl_boundary.start_s(), FLAGS_trajectory_time_length));
  path_time_obstacle_map_[obstacle_id].set_upper_left_point(SetPathTimePoint(obstacle_id, sl_boundary.end_s(), 0.0));
  path_time_obstacle_map_[obstacle_id].set_upper_right_point(SetPathTimePoint(obstacle_id, sl_boundary.end_s(), FLAGS_trajectory_time_length));
  static_obs_sl_boundaries_.push_back(std::move(sl_boundary));
  ADEBUG << "ST-Graph mapping static obstacle: " << obstacle_id << ", start_s : " << sl_boundary.start_s() << ", end_s : " << sl_boundary.end_s()<< ", start_l : " << sl_boundary.start_l()<< ", end_l : " << sl_boundary.end_l();
}




// https://github.com/ApolloAuto/apollo/issues/6038

planner是如何避免跟轨迹冲出道路边缘如防护栏等的？
在制作生成高精地图时，你说的护栏、道路边缘等非道路的就已经标记为不可行驶区域被排除了。另外在车辆行驶时，perception 也会实时检测这些障碍物。传到 planning时，这些都已经被标记为不可行驶区域， planning只会在可行驶区域撒点规划，自然不需要处理这些不可行驶静态障碍物的逻辑。




// https://github.com/ApolloAuto/apollo/issues/5897
apollo 3.0相对地图导航模式支持多车道超车，我们使用relative map+ lattice+navigation模式，
在对code分析中，lattice_planner.cc是在横向-0.5~0.5，纵向10m、20m、40m、80m范围根据指引线进行路径规划，也就是lattice_planner.cc是在本车道(当前指引线)进行规划吗？
lattice_planner.cc路径：
/home/alan/apollo/modules/planning/planner/lattice/lattice_planner.cc
如果前方有障碍物需要变换车道，这个时候需要临近车道的指引线给lattice_planner.cc吗？这个指引线是通过reference_line_provider.cc提供的吗？
reference_line_provider.cc地址：
/home/alan/apollo/modules/planning/reference_line/reference_line_provider.cc
即如果前方有障碍物需要换车道时是通过变换lattice_planner.cc的指引线进行的吗？


lattice指引线只有最多有两个,一个是ego lane, 还有一个是 neighbour lane , 选择哪个是通过所有的trajectory评分完毕后看分数最低的来决定的, 如果有障碍则连评分的资格都没有. lattice里面并没有去刻意判断是超车,还是跟车等等. 一切都是看评分. neighbour lane优先还是ego lane优先是通过ChangeLaneDecider类来完成基本判断的,仅仅是调整一下优先级而已


	// /modules/planning/common/frame.cc. 在初始化frame类时即Frame::Init()会调用bool Frame::CreateReferenceLineInfo()，在bool ReferenceLineProvider::GetReferenceLines中从相对地图处获取reference line和segment，在获取完信息后存放在ReferenceLineInfo类指针中。同时在bool Frame::CreateReferenceLineInfo()中通过如下判断本车道优先还是临近车道优先，
if (FLAGS_enable_change_lane_decider &&
!change_lane_decider_.Apply(&reference_line_info_)) {
AERROR << "Failed to apply change lane decider";
return false;
}
但是有个问题是这儿有一个FLAGS_enable_change_lane_decider 宏，如果盖红设置为false，不进入该逻辑，那么车道优先级是车当前车道为最优优先级吗？


2. 规划模块从相对地图模块接收到reference line，然后将这些reference line存放在reference line info这个类中，用python navigatior xxxx.smothed 脚本播放轨迹，播放几个轨迹，便有几个reference line，也便有几个reference line info。然后对每一个reference line info求取车的横向和纵向规划轨迹束，在求取纵向轨迹的过程中会考虑三种情况即超车、跟车、遇到障碍物及交通信号等停车，根据每一种情况分别进行轨迹生成，生成完横向纵向轨迹然后通过评价函数选取每一对横向纵向轨迹束进行打分，
获取轨迹对评价值：
double trajectory_pair_cost =
trajectory_evaluator.top_trajectory_pair_cost();
获取轨迹：
auto trajectory_pair = trajectory_evaluator.next_top_trajectory_pair();
在对每一个reference line info进行生成轨迹束的过程中最终只会有一个最优的轨迹数据存放到reference line info中。
每一个规划周期选出最优的纵向、横向轨迹对，这些信息存放在ReferenceLineInfo 类指针中， 然后通过 FindDriveReferenceLineInfo选出代价最小的轨迹发布出去，
选出代价最小轨迹：
const auto* best_ref_info = frame_->FindDriveReferenceLineInfo();
综上：即python navigatior xxxx.smothed播放几条参考线，生成几车道的相对地图，那么规划模块就会根据每一个参考线生成一最优条轨迹(此处的轨迹不是生成横向纵向轨迹束，而是对横向纵向轨迹束合成之后的轨迹)，然后从这几条参考线的最优轨迹中选取代价小的一条进行发布出去。



// https://github.com/ApolloAuto/apollo/issues/5915


The EM can find a path (dp_poly_path) that why you see the thin "green" line, but the trajectory (thicker green line) for that path will have a collision with the obstacle, that why EM planner fails even when it found a path, in this case, path from dp_poly_path is not a good path.

In order to make EM search for a good path, you have to tune the parameters of EM (change the sample resolution, increase sample in S/L direction...), but it may increase the processing time.

by changing the parameter Step_length_min and step_length_max to 11, can the vehicle successfully pass the obstacle. according to my experiments 11 is the limit value in this case. Thank you all.




// https://github.com/ApolloAuto/apollo/issues/5739
Hi Guys,

This is with respect to issue #4008.

I have created a custom created map with 2 lanes in same direction to test the lane change and overtake behavior. Below are the details.

a) Length - 320 m
b) Lane IDs 98_1_-1 and 98_1_-2
c) right neighbor and left neighbor has been updated as well.

The autonomous vehicle still do not do the lane change behavior.Can anybody please explain me what is missing in here ?

What planner you use.
if you use the EM, tune the parameter in planning_config.pb
also in planning_gflags (look for lane change parameter)

Good luck








// https://github.com/ApolloAuto/apollo/issues/9218
// Why path_out_lane_cost is not used?
 the value of "path_out_lane_cost" is huge and it was intended to "force" adc to keep in-lane. But that is not always what we really want, e.g when adc side-pass obstacles or change lanes...
and "path_l_cost" can help "keep" adc closer to reference_line, or in lane.
BTW, in Apollo 5.0, we are not using dp_poly_path_optimizer any more.

ComparableCost TrajectoryCost::CalculatePathCost()  {}




// https://github.com/ApolloAuto/apollo/issues/8439
// Routing not ready
Describe the bug
We are using Apollo 3.5 version and running into some issues with the planning module. After
finishing a routing request. The console on the dreamview shows "Routing Success! ". However, the "planning.INFO" tells me the routing is not ready. The consequence is that although there is a red line shows up on the map (reference_line), no planning messages are being published.

if you launch planning module manually from terminal, you shall turn off planning module from DreamView, and then launch planning manually from terminal.
And, you need launch planning mode first, and then send routing request. So that planning module can receive routing response. Otherwise, when the planning module starts, the routing response was already gone. Although you can see routing red line in DreamView, the planning module actually never got it in time.




// https://github.com/ApolloAuto/apollo/issues/7979
// How to limit curvature of reference line 

I understand that the reference line is generated from HDmap data. Is there any way to set a limit for curvature of the reference line without modifying the HDmap? Another way, how to implement the curvature constraints for the reference line?

Thank you for your reply, I changed the reference line smoother from QpSplineReferenceLineSmoother to SpiralReferenceLineSmoother. (change smoother_config_filename in the planning_gflags.cc)

The lines you mentioned is for the weight of kappa, what I am looking for is the boundary of kappa.
Then I look into https://github.com/ApolloAuto/apollo/blob/master/modules/planning/reference_line/spiral_problem_interface.cc
and I manually change the value of kappa_lower and kappa_upper for the optimizer.

However, these changes do not make any difference in the reference line.



// https://github.com/ApolloAuto/apollo/issues/6710
// Current Lane of the Ego vehicle 

Could you please let me know how we can fetch the current lane on which Ego vehicle is currently present. I am using the below code snippet, however it is returning more than 2 lanes. How can I get single lane information.

  const int status = HDMapUtil::BaseMap().GetLanes(point, 0.005, &lanes);
  static double speed_limit = 0;
  int lane_counter = 0;
  for (auto& lane: lanes){
    const hdmap::Lane& single_lane = lane->lane();
    speed_limit = single_lane.speed_limit();
    lane_counter = lane_counter + 1;
    // ADEBUG << "Speed Limit : " << speed_limit;
  }

If you want to get the lane(s) by point, the map may return your multiple lanes since some lanes are overlapped (especially around junctions and turns). This is as expected.
The lane by point is only unique if you talk about the lanes along the routing. There are some methods inside ReferenceLine class to get it.
https://github.com/ApolloAuto/apollo/blob/master/modules/planning/reference_line/reference_line.h



// https://github.com/ApolloAuto/apollo/issues/8283
// How to get obstacle velocity from reference line
Current Apollo API provides a way to get speed of obstacle "obstacle->speed()", my understanding is such speed is always positive which means obstacle always move in the same direction as ego vehicle.
However, In two-way traffic, obstacles may have opposite direction toward ego vehicle.
Is there any API to get the velocity of Obstacle ? positive means same direction and negative means opposite direction with ego vehicle ?


The speed is the absolute value of the velocity. There is also a velocity_heading parameter which shows the velocity direction from 0 to 2*PI.



// https://github.com/ApolloAuto/apollo/issues/7154
// modules/planning/common/reference_line_info.h

  const SLBoundary& AdcSlBoundary() const;  //  means the planning start point 
  const SLBoundary& VehicleSlBoundary() const; //  mean the real time location 




//
  The main cyber command should be cyber_recorder.

$ cyber_recorder
usage: cyber_recorder > []
The cyber_recorder commands are:
info Show information of an exist record.
play Play an exist record.
record Record some topic.
split Split an exist record.
recover Recover an exist record.

$ cyber_recorder record -h
usage: cyber_recorder record [options]
-o, --output output record file
-a, --all all channels
-c, --channel channel name
-h, --help show help message

$ cyber_recorder split -h
usage: cyber_recorder split [options]
-f, --file input record file
-o, --output output record file
-a, --all all channels
-c, --channel channel name
-b, --begin <2018-07-01 00:00:00> begin at assigned time
-e, --end <2018-07-01 01:00:00> end at assigned time

There is a script for recording data on self-driving vehicles, which could be a reference if that is relevant in your context.






// https://github.com/ApolloAuto/apollo/issues/6914
// Why Apollo stitches reference line? Just for saving computation resource? 

For now, we do not support offline reference line smoothing because of the complexity of city lane network. In current implementation, stitching is for stabilizing reference line and saving computation effort






// https://github.com/ApolloAuto/apollo/issues/6809
// Can traffic regulation loading make it possible to add new traffic rules without modifying the code? 
 I am  not sure what you exactly mean. If you want to add a new traffic rule and make it run, you have to create a class/implementation of that rule (please refer to any traffic rule class at https://github.com/ApolloAuto/apollo/tree/master/modules/planning/traffic_rules, as examples), and also create configuration if needed (please refer to https://github.com/ApolloAuto/apollo/blob/master/modules/planning/proto/traffic_rule_config.proto)

And then register your traffic rule as above, to make it effective and run.

void TrafficDecider::RegisterRules() {}




// https://github.com/ApolloAuto/apollo/issues/6615
//  smooth the recorded trajectory points

I have a question regarding to the reference line smoothing tool under the directory of /apollo/modules/tools/navigator/smooth.sh.


// Any demo or tutorial to test entire apollo 3.5 stack? #7019
I can think of two ways to test one or more modules.

With dreamview and other modules running in your local dev env.
Make sure you have the input data for the module you are interested in, e.g. they can be played from a record file. You will need to turn on the module you are interested in, and possibly all the downstream modules including sim_control, if you would like to see how your changes affect the way those downstream modules run, and sim_control helps to close the loop by feeding back virtual localization and chassis info. If you are able to observe the output of your module directly on dreamview, then you could just turn on your module.

With Apollo Simulation Platform on the cloud.
Submit your code changes onto a public repository forked from Apollo, and login to the simulation platform https://azure.apollo.auto, which provides about 200 scenarios to test prediction and planning modules currently.




// Why Apollo stitches reference line? Just for saving computation resource? #6914
For now, we do not support offline reference line smoothing because of the complexity of city lane network. In current implementation, stitching is for stabilizing reference line and saving computation effort



// Can traffic regulation loading make it possible to add new traffic rules without modifying the code?

If you want to add a new traffic rule and make it run, you have to create a class/implementation of that rule (please refer to any traffic rule class at https://github.com/ApolloAuto/apollo/tree/master/modules/planning/traffic_rules, as examples), and also create configuration if needed (please refer to https://github.com/ApolloAuto/apollo/blob/master/modules/planning/proto/traffic_rule_config.proto)
And then register your traffic rule as above, to make it effective and run.
planning/traffic_rules/traffic_decider.cc





// decision.proto - MainChangeLane
Change lane is supported through the combination effect from reference line provider, frame, reference line info, and a class called "ChangeLaneDecider".
Please first verify that the routing has a lane change region, otherwise the lane change will not take effect.





// navigation模式下的仿真怎么更新车身位置？ #6067

standard 模式中的 reference line 是 HDMap 中来的, HDMap 是事先采好的，作为本地文件存在硬盘上。
navigation 模式中的 reference line 用了 relative map. relative map 可以是感知实时生成的，仿真时由 rosbag 包中发送的。所以需要播放bag包

standard模式的仿真不需要实时位置，standard模式仿真依赖HDmap，而hdmap是百度已经做好的，所以你只需要选择起始点和终止点，并打开planning模块，车就可以跑起来
navigation模式就是2.5版本中为了方便开发者实车调试添加的，依赖于相对地图，而相对地图又是车实际跑的过程中产生的，如果你要仿真navigation模式就只能播放rosbag包来去提供实时的位置信息，



https://github.com/ApolloAuto/apollo/blob/master/modules/dreamview/backend/sim_control/sim_control.cc 

navigation模式:SimControl::OnReceiveNavigationInfo, localization 就是 relative map 实时算出的path的第一个点.
standard 模式:SimControl::PerfectControlModel,localization 就是取的 planning 的trajectory上相对时间对应应的点.





// would it be possible that the use any map to utilize the 'CartesianFrenetConverter' function? #6022
You can use road center line to build reference line. When you have reference line, you can map any point to frenet 's' and 'd'




// 关于相对地图+lattice 多车道超车问题 #5897
Navigation mode supports multiple lanes and lane change in Apollo 3.0. You could use following command to send out multiple navigation lines:

dev_docker:/apollo/modules/tools/navigator$python navigator.py navigation_line_1 navigation_line_2

in this example, navigation_line_1 navigation_line_2 could be navigation line on two adjacent lanes. And navigation_line_1 has higher priority than 2.



// Do not find code to set velocity and accelerate for speedpoint retrieved in DPSTGraph #3535

Why there are no code to set velocity and accelerate for speedpoint retrieved in DPSTGraph
The speed value is not set after st graph, because we only need the value of s and t in the following steps




// dp_st_graph, what is the difference of max_acceleration from vehicle_param and dp_st_speed_config? #3682
The first one is the physical limit of a vehicle, while the second one is what we used in dp st graph as an acceleration limit. The two may or may not be identical, where the second one is usually smaller, as used in our config files.





//
planning/tasks/deciders/path_decider/path_decider.cc
bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  if (!MakeStaticObstacleDecision(path_data, path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}




 for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto &obstacle = *path_obstacle->obstacle();
    bool is_bycycle_or_pedestrain =
        (obstacle.Perception().type() ==
             perception::PerceptionObstacle::BICYCLE ||
         obstacle.Perception().type() ==
             perception::PerceptionObstacle::PEDESTRIAN);


Currently, we make decisions(Nudge/Stop) for static obstacles in PathDecide.
The decisions(Yield/Overtake and etc) for moving obstacles are made by STGraph.


I read the code, and found that moving obstacles decision is made in speedDecider solver. Just as you said it's made by STGraph. However, why don't make decision for static and moving obstacles in PathDecider solver at the same time? In PathDecider, we have already got moving obstacle prediction trajectory from Prediction Module ros msg broadcast, even in SL graph (not ST graph), we could make decision(Yield/Overtake and etc) for moving obstacle.


we plan path first, and then speed based on path.
Currently we don't have SL graph. We have SL info, but that's not enough to make decision for moving obstacles without dimension of T(time).

once the path has been planed, we make decision for moving obstacles based on STGraph, and a "OVERTAKE/YIELD/STOP" decision (which would be achieved via speed) would be made if their paths may cross ego vehicle path.
In your second graph, ADC may slowdown(follow) or yield to that moving obstacle depending on its path/speed/heading.



// Planning: lane change with SLT graph and obstacle filter
SL graph is based on a single lane frenet frame rather than two lanes.
Moving obstacles only in ST graph.
actually always project moving obstacles into the reference line.



// is EM plannner using the qp Spline Path Optimizer?

in the function of EMPlanner::RegisterTasks(), the qp Spline Path Optimizer is not be registered, is that the qp Spline Path Optimizer is unnecessary？

since QpSplineStSpeedOptimizer is used, why are PolyStSpeedOptimizer used again?

Thank you for reply.


We found that DpPolyPath has result is good enough on road. In addition, the QP-OASES solver may not always return a valid solution, therefore, we skipped the QpSplinePath step. Road driving tests have proved that DpPolyPath is really good.

PolyStSpeedOptimizer is used to make speed decisions for obstacles, and QpSplineStSpeedOptimizer can further optimize the driving speed based on the obstacle decisions

I can not understand how the two tasks work completely. How do the speed decisions made by PolyStSpeedOptimizer exert influence on the QpSplineStSpeedOptimizer? or the two part is independent?

The dp_st_speed will use dynamic programming to find the min cost speed profile, which is used to determine whether we will overtake or yield a vehicle (represented as a st-region on the st graph). Then, those decisions will be treated as hard limits (inequality constraints) in qp_st_speed.
You can ignore poly_st_speed for now, it is still under development. qp_st_speed is the second speed optimizer after using dp_st_speed. You can find the task sequence in modules/planning/conf/planning_config.pb.txt



// Planning failed with one static obstacle ahead the vehicle under SimControl mode #5915

The EM can find a path (dp_poly_path) that why you see the thin "green" line, but the trajectory (thicker green line) for that path will have a collision with the obstacle, that why EM planner fails even when it found a path, in this case, path from dp_poly_path is not a good path.

In order to make EM search for a good path, you have to tune the parameters of EM (change the sample resolution, increase sample in S/L direction...), but it may increase the processing time.

by changing the parameter Step_length_min and step_length_max to 11, can the vehicle successfully pass the obstacle. according to my experiments 11 is the limit value in this case. Thank you all.



// 利用相对地图和lattice planner进行规划 #5864
code 版本： apollo v3.0
相关模块： relative map、planning module
问题：
1、之前看到过V3.0版本支持在相对地图下的lattice planning，请问应该怎么修改代码，https://github.com/ApolloAuto/apollo/blob/r3.0.0/modules/planning/proto/planning_config.proto 中只将planner_type改为NAVI，https://github.com/ApolloAuto/apollo/blob/r3.0.0/modules/planning/conf/planning_config_navi.pb.txt中的planner_type改为Lattice就可以了么？
2、代码中并没有看到navi planner调用em planner或者lattice planner中的代码，怎么看到lattice planner使用的是相对地图呢？
谢谢



我的理解
相对地图+lattice在planning的调用关系是：
/home/alan/apollo/modules/planning/navi_planning.cc
调用planner里面的规划算法，这里面的规划算法有Lattice、Em、NAVI
planner规划算法的位置：
/home/alan/apollo/modules/planning/planner
所以你只要在前面的文档里面设置了LATTICE，同时最好用dreamview里面navigation 模式下的界面进行操作，因为这个界面调用的脚本里面有设置--use_navigation_mode这个模式的参数，设置完
就会运行相对地图+Lattice模式，可以在navi_planning.cc和/home/alan/apollo/modules/planning/planner/lattice/lattice_planner.cc加log查看。



// Trajectory stitching, why? #5033
Can you please explain what is the purpose of stitching trajectories in planning?
Is stitching between lastly published trajectory and something else?

You dont want the car change trajectory every time planner run -> car will change direction frequently
So you keep a portion of previous plan trajectory and stitch it to new trajectory.

Trajectory is much longer than the length of the ego car.






// How to calculate stbounday() ? #5751

I have been going through the Apollo code and having difficulty in understanding how the stboundary() is calculated. I got the point that stboundary it is the "s" values with respect time. However I got the below doubts when going through the st_boundary.cc program.

t value - does this start when the routing starts ?

s values - Could you please let me know from which point the values are being calculated / measured ? Is it taken with respect to Reference line starting ?

If stboundary is empty (IsEmpty() is true), does that mean the obstacle is not moving ?

What does the below functions return in stboundary class

        double min_s() const;
        double min_t() const;
        double max_s() const;
        double max_t() const;
Do we see Stgraph on PNC monitor ?
Is QpSplineStSpeedOptimizer is same as ST graph ?

Also similar condition is being checked in backside_vehicle.cc. Could you please explain the below piece of code.

    // Ignore the car comes from back of ADC
    if (path_obstacle->reference_line_st_boundary().min_s() < -adc_length_s) {
      path_decision->AddLongitudinalDecision("backside_vehicle/st-min-s < adc",
                                             path_obstacle->Id(), ignore);
      path_decision->AddLateralDecision("backside_vehicle/st-min-s < adc",
                                        path_obstacle->Id(), ignore);
      continue;



(1) t value is for time. we plan 0-6s
(2) s: start with the current position of the ego vehicle
(3) it means the obstacle has  path would not cross with ego vehicle.
(4) these returns the st boundary of an obstacle. min_t() and min_s() are the (x,y) of the left lower point, max_t() and max_s() are the (x,y) of the right upper point.
(5) yes. StZGraph is on PNC monitor.
(6) this check means that, the obstacle has started behind the back of ego vehicle, and we can ignore it because it is behind ego vehicle.




// Build Stboundary , what is the difference between PathObstacle with StBoundaryMapper? #4785
Hey I m not quite understand what is the difference between BuildTrajectoryStBoundary in path_obstacle.cc and CreateStBoundary in st_boundary_mapper? In both, they set stboundary to a given path_obstacle. So what is the differenece?
They are similar. Build Stboundary in path obstacle uses reference line to pre-build approximate st boundaries.



// How planning module decide emergency stop or not? #4964
I have a check in modules/planning/planning.cc
And I see it will publish planning topic with PublishPlanningPb(&estop, start_timestamp); in some emergency cases.
But I do not know how planning make decision for estop reason or not.Can anyone tell me how to reproduce that case?



The following is estop reseason code:
ESTOP_REASON_INTERNAL_ERR:1,ESTOP_REASON_COLLISION:2,ESTOP_REASON_ST_FIND_PATH:3,ESTOP_REASON_ST_MAKE_DECISION:4,ESTOP_REASON_SENSOR_ERROR:5

In reference_line_info.cc, according to error code, we set estop decision
// check stop decision
int error_code = MakeMainStopDecision(decision_result);
if (error_code < 0) {
MakeEStopDecision(decision_result);
}
MakeMainMissionCompleteDecision(decision_result);
SetObjectDecisions(decision_result->mutable_object_decision());
}





// How two passages are connected? #5253
I think the passage is a single reference line that a vehicle can travel without changing lane.
It looks like routing is a sequence of such passages.
The question is that how two different passages are connected to represent the routing curve?
I think they must be overlapped then correct?


After a destination is given, routing module produces a sequence of end to end zigzag linear segments. Obviously, such lines can't be used for path planning. So a smoothing algorithm on the routing segment is done online to get a differentiable curve, which is called reference line, for the Planning module to work on.

Don't quite understand what is your definition of passages. If you mean the smoothed reference line, then yes. As the current implementation on computing reference line is online, to save computation effort. only a definite horizon of reference line is computed each cycle, so there would be overlapped part. To deal with that, we use stitching to combine two together. Check reference_line_provider.h for more detail.



// PnC monitor elements #5418
The red line: routing line
The slimmest blue line: reference line
The medium width blue belt: dp path line
The widest belt: planning  final trajectory



// What's the average computation time to solve a qp_spline_referenceline_smoother #5528

Hi,

To smooth a reference line, normally how many knots you have and what is  the average computation time with qpOASES?

Thanks a lot

The number of knots depends on the max length of each spline.
The average computation time depends on your hardware (number of cores, cpu speed, memory, etc.)


Thanks for the reply.

For instance, a 100m curve with 100 splines, running on your AVs, it takes dozens of milliseconds or seconds?

I have tried cvxopt in python as well as qpOASES in c++. Cvxopt takes 50 ms while qpOASES takes 5s to solve the same question.

For a qp problem used in our qp_st_speed_optimizer, the average running time is ~10ms

I ran a simple test in python, not in real-time, 100 cubic splines with 800 variables took 50ms including the matrix assigning.
cvxopt converts automatically a QP to a SOCP, which might be faster in this case
Yes, it could be faster. CXVOPT has an c interface api (https://cvxopt.org/userguide/c-api.html). If you are interested in improving the optimization performance, please feel free to add a cvxopt-based solver into Apollo.

On a Renesas H3 board running qnx, the time to process each configured task in a loop is roughly as the following (apollo r2.0.0):

TrafficDecider: 1 ms
PathDecider: negligble
SpeedDecider: negligble
DpPolyPathOptimizer: 4~6 ms
QpSplineStSpeedOptimizer (use qpOases underlying): 4~6 ms




// Minimal output to be published by the planning module for the control to work #4947
Control will need a list of (x, y t) points to follow.


// EM dp_road_graph #4943
Hi,
This is what I understand about EM.
EM planner generates bundle trajectory and then calculate the cost of each trajectory, and then select the best trajectory by calculating the trajectory cost. After that EM generate the speed profile for that trajectory.
My question is: in trajectory_cost.cc I saw you use some SpeedData to calculate the cost with Dynamic Obstacle.
Base on my understanding, at this step, EM does not generate speed profile yet, so where SpeedData come from?

Thank you.


From the last cycle, remember this is a iteratively planning




// Cost functions for Lattice Planner #4847

In your lattice planner module, you are using 6 cost functions to contribute the whole planner cost:

// Costs:
// 1. Cost of missing the objective, e.g., cruise, stop, etc.
// 2. Cost of logitudinal jerk
// 3. Cost of logitudinal collision
// 4. Cost of lateral offsets
// 5. Cost of lateral comfort
// 6. Cost of centripetal acceleration

Total cost = weighted sum of the costs above, weight of each cost function is defined manually:

// Lattice Evaluate Parameters
DEFINE_double(weight_lon_objective, 10.0, "Weight of longitudinal travel cost");
DEFINE_double(weight_lon_jerk, 1.0, "Weight of longitudinal jerk cost");
DEFINE_double(weight_lon_collision, 5.0,
"Weight of logitudinal collision cost");
DEFINE_double(weight_lat_offset, 2.0, "Weight of lateral offset cost");
DEFINE_double(weight_lat_comfort, 10.0, "Weight of lateral comfort cost");
DEFINE_double(weight_centripetal_acceleration, 1.5,
"Weight of centripetal acceleration");

My question is that in real world, there are many scenarios. How do you pick up the best weight array?

run apollo sdk in apollo Simulation. traverse all the scenarios as many as possible?
Empiricalism based on large-scale road testing?

Currently these parameters are determined empirically, based on our daily urban and highway testing. In future, we expect to develop a more advanced approach that can dynamically determine the weights according to scenarios.




// 关于planning模块的几点疑问，想请教一下 #4763

（1）em和lattice方法实时规划出的轨迹起点是从车体原点出发的吗？如果是，请忽略问题（2）。
（2）一旦轨迹跟踪产生较大误差，如车辆偏出车道外，在其开回期望轨迹的过程中，其碰撞校验是怎么计算的？
（3）apollo是如何防止实时轨迹规划过程中产生的横向抖动？如在高速行驶时，轨迹的横向抖动可能会造成车辆失稳。


感谢提问，我来解答一下：

规划起点有两种可能， a. 车体原点 (在规划的第一周期或者当前车体偏移规划轨迹过大), b. 上一轮周期规划轨迹上本轮相对应的轨迹点。
我们假设车体原点与规划起点的差异并不大（b情况），所以碰转检测只对规划轨迹进行，而没有对实际位置进行。如果差异太大的话， 我们就以车体原点进行规划（a情况）。
横向抖动通过我们对轨迹质量的建模加以数值化，横向抖动的轨迹会赋予一个高的代价，所以一般情况下不会选取。


请看问题之后的回复：

１．em和lattice两种规划器都是您上面解答的情形吗？
是的， 规划起点的决定发生在进入具体的planner之前。

２．针对上面第一个答复：为什么要采用两种规划起点的方式呢？而不是仅仅采用a方式．
主要原因是我们期望得到连续平滑的控制输出。如果每次规划都是从当前位置作为起点的话，控制模块在计算控制误差的时候会发生不连续的现象，进而导致不连续的控制输出。

３．针对上面第一个答复＂b. 上一轮周期规划轨迹上本轮相对应的轨迹点＂：(1)如何在本轮规划时，锁定上一轮周期规划轨迹上本轮对应的轨迹点的位置？（是通过ＧＰＳ坐标锁定该点的绝对位置？）
是通过时间来决定的，由上一个周期的对应点根据周期时间向后推一个周期。

４．假设轨迹跟踪没有横向偏差，但纵向由于加速或制动等原因，使得本周期车辆实际位置偏离上周期期望达到的位置（纵向跟踪偏差）较大，这种情况也会进入起点为" a. 车体原点"的规划吗？
是的

５．假设某一静止障碍物在t~t＋N时刻N个周期内的决策都是避让(nudge)，而在t＋Ｎ＋1时刻，由于前期轨迹跟踪误差等原因，导致该时刻规划的轨迹无法通过该障碍物，从而对该障碍物的决策变为不可避让（刹停）；apollo是如何防止在规划过程中，避免这一类决策结果在前后周期的不一致性？［注：如果人工驾驶的话，对某一静止的障碍物的决策一般是固定的，要么避让，要么刹停．］
首先,对于横向误差，我们设置的临界点比较小，大概是几十厘米，所以您提到的这个问题出现的情况比较小。
其次，决策在周期间不一致的问题普遍存在（我们可能不应该称之为“问题”，“现象”更为合适）。决策需要根据当前车辆状态实时作出调整，可以想象成一个更高级的feedback control。


３．针对上面第一个答复＂b. 上一轮周期规划轨迹上本轮相对应的轨迹点＂：(1)如何在本轮规划时，锁定上一轮周期规划轨迹上本轮对应的轨迹点的位置？（是通过ＧＰＳ坐标锁定该点的绝对位置？）
是通过时间来决定的，由上一个周期的对应点根据周期时间向后推一个周期。

“上一周期的对应点向后推一个周期”得到的轨迹点是如何映射到本周期的坐标系下呢？[假设上一时刻t0的轨迹前两点依次是p1(t0)、p2(t0)，如何求得p2(t0)在当前时刻t1的(车体坐标系下的)坐标p2(t1)呢？]




// What's the difference between rtk planner, em planner, and lattice planner? #4674
RTK Planner started at Apollo 1.0, which is used to plan the pre-set trajectory for the follow-through algorithm.

EM planner started in Apollo 1.5 and has been abandoned in 3.5. Based on dynamic planning and quadratic planning, it first carries out path planning and then speed planning, which is suitable for complex scenarios.
The EM planner has been tested to work at 0 - 105 kmh (0 - 65 mph).

Lattice planner started with Apllo 2.5, and it also carries out path planning and speed planning, which is suitable for simple scenarios.





// Vehicle frame orientation #4584

It is FLU. I think what you want is in the https://github.com/ApolloAuto/apollo/blob/master/modules/planning/planner/navi/navi_planner.h




// SL coordinate intial point? #4414

The origin of SL coordinate is the center point at a starting point of a lane.
Frenet coordinates and SL coordinates are the same thing. SL coordinates can be converted to XY coordinates. Apollo has SL is used for path planning, ST is used for velocity planning, and ST is used for collision detection.



// In file /apollo/modules/planning/conf/planning_config.pb.txt, it includes two speed optimizers:

...
    task : DP_ST_SPEED_OPTIMIZER

    task : SPEED_DECIDER

    task : QP_SPLINE_ST_SPEED_OPTIMIZER
...
If we remove QP_SPLINE_ST_SPEED_OPTIMZER, the car does not move.

It is also noticed that QP_SPLINE_PATH_OPTIMIZER is not registered with any planner.

The DP_ST_SPEED_OPTIMIZER is for decision purposes. QP_SPLINE_ST_SPEED_OPTIMIZER will generate the quantitative trajectory.

Currently, we do not use QP_SPLINE_PATH_OPTIMIZER. Only poly_path is used.



// How to use/modify/implement new algorithm in planning module? #3462
// https://github.com/ApolloAuto/apollo/issues/3462

--- to change to another planner, please (1) add your planner name in planning_config.proto (line 31 - 35); (2) define the config of your own planner in planning_config.proto (after line 29); (3) change "planner_type : EM" to the one you have in planning_config.pb.txt.

--- Because the car in the rosbag has position info pre-recorded, it cannot follow your planning trajectory. If you want to make the car follow a generated path, you need to turn on the "SimControl" button (tasks->others) in the dreamview. Under the sim-control mode, the dreamview will receive the planning trajectory and help to move the car for you to mimic the driving process.


--- Since the planning results were recorded in the rosbag, you will observe the trajectory by just playing the bag. You can filter the /apollo/planning topic from the rosbag in order to test your own planning.

Thank you for being patient with me. I really appreciate your support.

This is what I done so far:
#filter the planning module
rosbag filter ./docs/demo_guide/demo_2.0.bag ./docs/demo_guide/demonew.bag "topic== '/apollo/planning'"
#run the new bag
rosbag play -l ./docs/demo_guide/demonew.bag

But in the Dreamview, nothing shows, the Default Routing button is grey out, could not select.
I think the filter should be "topic != '/apollo/planning'" (not == ). The topic will be kept if it the expression is true.

To start planning module, please (1) build the project you have modified (using bash apollo.sh build) (2) run the planning binary in a separate terminal (using bash scripts/planning.sh), or just turn on the planning from dreamview.





// DP boundary conditions ? #1614

I have a question about EM planner.
What I understood is that some boundary conditions are found by solving some optimization problem using Dynamic Programming. What exactly those boundary conditions? Are they speed limits and lane boundaries?
Thank you!

The concept of boundary conditions is used for quadratic programming (QP).
The boundaries come from the road (such as road speed limits, lane boundaries), vehicle dynamics (e.g. curvature, acceleration of a trajectory), and other obstacles.

One more thing. There is another kind of boundary --- regions in S(path)-T(time) graph that represent the mapping of obstacles in a S-T coordinate system (maybe this is the one you mentioned ?).

In a DP process, the distance between a point (s, t) and the obstacle S-T boundaries are calculated and a cost is assigned based on the distance.


DpStSpeedOptimizer: the dotted line is the optimization results from dynamic programming.
QpSplineStSpeedOptimizer: the dotted lines are some reference lines calculated in task/qp_spline_st_speed/qp_spline_st_graph.cc (line 337 and line 391).



	QpSplineStGraph::AddCruiseReferenceLineKernel
	QpSplineStGraph::AddFollowReferenceLineKernel
	QpSplineStGraph::AddYieldReferenceLineKernel



Thank you and sorry for keep bugging you.
Yes, the line of DpStSpeedOptimizer must be the optimization result of DP. Can you please provide its physical meaning? Thank you!


The result is a list of (s,t) points, meaning at some time t, we would like the vehicle to travel a distance s (starting from the current position) on the generated path.




// how to create hdmap?

Good information, but let us say I have no HDmap. How do I create one? I see there are some tools, like create_map_from_mobileye.sh and create_map_from_xy.sh (which can presumably be used with UTM coordinates), but are there any tools in Apollo that will allow you to add junctions and signals and link roads together? How did the Apollo team create the sunnyvale_loop demo maps, and is that technology available to Apollo users in any form?


There is no method or tools to generate base_map.xml directly from GPS and LiDAR data now. The steps we generate Apollo HDMap are roughly as follows:

	collect Lidar data,GPS data, IMU data, Camera data, etc. through HDMap collection vehicles;
	process point cloud data using technologies such as Lidar SLAM;
	Identify road elements through machine learning, deep learning,etc. to generate HDMap data, which may contains some misidentified data.
	manually correct the automatically generated HDMap and construct logical relationships between the map elements to generate the final HDMap.
	The technology we create the HDMaps is not available to users now, but we will open our methods how to collect map data through HDMap collection vehicles in the near future.




// Uses of PerceptionSLBoundary() and reference_line_st_boundary() #5183
Hi All,

I have been going through the Apollo code and getting confused in the below point.

PerceptionSLBoundary() -- This gives the trajectory for the Obstacle using the protobuf. (It gives 4 values start_s, end_s, start_l and end_l).

What does this reference_line_st_boundary() gives ? - Looks like it returns the class object from the class. Whether this creates a 2D box around the obstacle ?

Why there are 2 boundaries for single perceived obstacle ?

Could you please clarify my doubt?

Thank you,
KK




//	Traffice decider tasks #4838
Hi All,

Can anybody please give me some insights on how the traffic decider tasks are formulated.

If you could provide me a document with the explanation of each tasks would be great.

Directory: apollo/modules/planning/tasks/traffic_decider/

Thank you,
KK


The traffic decider tasks are to make rule-based decisions on ego vehicle and obstacles. It is highly scenario related so that you can find multiple deciders in the folder. I suggest reading codes in that folder for details. It is complex, but not complicated.





@kk2491 we do not have document or readme at this point. it is pretty much self-explanatory if you read the code, the classes at traffic_decider folder.
e.g.

for traffic light, the ego vehicle would stop behind the stop line of the specific traffic light if red light is detected.
for crosswalk, the ego vehicle would stop behind the stop line of the specific crosswalk if there is pedestrian(s) or bicycle(s) are passing through that crosswalk.
etc.
Hope the information answered your question.
Thank you for supporting Apollo!


@jmtao .. Thanks for the response. I have tried the tasks. In case of Cyclist, the autonomous car does not follow the obstacle even if the speed of the cyclist is higher than the threshold. It does stop and go.
Could you please let me know if there is any other conditions?


@kk2491 can you be more specific or post screen shot of your DreamViewer? ADC follows the bicycle if the bicycle running along ADC current reference line. If the bicycle stops ADC would stop.




// Build Stboundary , what is the difference between PathObstacle with StBoundaryMapper? #4785
@lianglia-apollo Hey I m not quite understand what is the difference between BuildTrajectoryStBoundary in path_obstacle.cc and CreateStBoundary in st_boundary_mapper? In both, they set stboundary to a given path_obstacle. So what is the differenece?

They are similar. Build Stboundary in path obstacle uses reference line to pre-build approximate st boundaries.



// What does longi_coef and longitudinal_coef mean? #4274
// https://github.com/ApolloAuto/apollo/issues/4274
Polynomial function for x/f and y/g is based on "world frame". When consider boundary constraint in reference line smoother, we should map polynomial function coordinates (x,y) into FLU frame to calculate difference of lateral distance and longitudinal distance between predict position and ground truth position.
"longi_coef" is the projection coefficient vector in L axis, while 'longitudinal_coef ' is the projection coefficient vector in F axis.
assume:
x=f(t)=a0+a1t+a2t2+a3t3+a4t4+a5t5

y=g(t)=b0+b1t+b2t2+b3t3+b4t4+b5t5
define:
A=[a0,a1,a2,a3,a4,a5]T

B=[b0,b1,b2,b3,b4,b5]T

C=[1,t,t2,t3,t4,t5]

(A,B)=(AT,BT)T
such that:
x = f(t) = CA
y = g(t) = CB

According to anchor point heading, we know:
unit direction in F axis: (cos(heading), sin(heading)), also equal to (-sin(heading-pi/2), cos(heading-pi/2))
unit direction in L axis: (cos(heading+pi/2), sin(heading+pi/2)), also equal to (-sin(heading), cos(heading))

Then map predict point from "world frame" to FLU frame, we can get lateral and longitudinal distance:
lateral distance: (-sin(heading), cos(heading)) · (CA, CB) = (-Csin(heading), Ccos(heading))(A, B)
longitudinal distance: (-sin(heading-pi/2), cos(heading-pi/2)) · (CA, CB) = (-Csin(heading-pi/2), Ccos(heading-pi/2))(A, B)

Note:
A · B means two vector inner product
AB means matrix multiplay(dot)

· above means inner product. Here (-Csin(heading), Ccos(heading)) is the "longi_coef", which is the projection coefficient vector in L axis. (-Csin(heading-pi/2), Ccos(heading-pi/2)) is the 'longitudinal_coef ' , the projection coefficient vector in F axis.

d_lateral and d_longitudinal is the lateral and longitudinal distance of ground truth point(anchor point), so we can boundary constraint:
constraint in L axis: |d_lateral - longi_coef(A,B)| <= lateral_bound
constraint in F axis: |d_longitudinal - longitudinal_coef(A,B) | <= longitudinal_bound

Remove absolute symbol:
upper lateral boundary: longi_coef(A,B) >= d_lateral - lateral_bound
lower lateral boundary: -longi_coef(A,B) >= -d_lateral - lateral_bound
upper longitudinal boundary: longitudinal_coef(A,B) >= d_longitudinal - longitudinal_bound
lower longitudinal boundary: -longitudinal_coef(A,B) >= -d_longitudinal - longitudinal_bound

By the way, will it be better to change the name "longi_coef" to "lateral_coef"?




// QpSplineReferenceLineSmoother::SetAnchorPoints and QpSplineReferenceLineSmoother::Smooth #4208

Here are the signatures of two functions:

bool QpSplineReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line)

void QpSplineReferenceLineSmoother::SetAnchorPoints(
    const std::vector<AnchorPoint>& anchor_points)
After read the related functions about how a reference line is smoothed out, I noticed there is direct relationship in between the anchor_points and the raw_reference_line to be smoothed. In fact, these anchor points have to be extracted from the raw reference line or the prefixed reference line, otherwise the resulted smoothed_reference_line is meaningless.

If this is the case, there is no need of SetAnchorPoints(). Instead, the anchor points should be extracted inside Smooth() for the given raw_reference_line and prefix_ref_line.

Here is code snippet of ReferenceLineProvider::GetAnchorPoints():

void ReferenceLineProvider::GetAnchorPoints(
    const ReferenceLine &reference_line,
    std::vector<AnchorPoint> *anchor_points) const {
  CHECK_NOTNULL(anchor_points);
  const double interval = smoother_config_.max_constraint_interval();
  int num_of_anchors =
      std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
  std::vector<double> anchor_s;
  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1,
                              &anchor_s);
  for (const double s : anchor_s) {
    anchor_points->emplace_back(GetAnchorPoint(reference_line, s));
  }
  anchor_points->front().longitudinal_bound = 1e-6;
  anchor_points->front().lateral_bound = 1e-6;
  anchor_points->front().enforced = true;
  anchor_points->back().longitudinal_bound = 1e-6;
  anchor_points->back().lateral_bound = 1e-6;
  anchor_points->back().enforced = true;
}
It shows that there is no other dependencies to generate anchor points for a reference line but itself. Anyhow, this is just a suggestion.


The current code flow is:
raw_reference_line -> anchor_points -> smoothed reference line.
Anchor points were designed as the middle layer, such that different smoothing algorithms can have a uniform constraint input (which is anchor point), without being bothered by how these anchor points are generated.

You may already notice that there is some logic in how to extract and shift these anchor points, and anchor points could come from a part of a previous reference line. This logic is orthogonal with the smoothing algorithm. It also allows us to generate anchor points from more resources than raw reference line, e.g., in places with out explicit lane info.




// about build st_boundary #4203
Since we get path_decision from dp_st_speed and already build st boundary , why we erase st boundaries and rebuild again in qp_spline_st_speed at line 87 
and what does the func " BuildFromPoints() " at line 58 in st_boundary.cc mean?
Expect for your reply, thanks


st_boundary is a concept based on a driving path. Different path corresponds to different st boundaries.
Previously, the ADC is assumed to drive following reference line,
but in qp_spline_st_speed, the ADC is assumed to drive the path created by DP PATH stage. Therefore, the st_boundary need to be refreshed.





// RelativeMap: down sampled raw navigation path data.

Apollo 3.0 uses a new smoothing algorithm to generate the navigation path, which makes the number of points in the navigation path very large. It may cause the ROS buffer to overflow and thus result in the navigation path not being sent successfully. Using a down sampling strategy can effectively solve this problem.
Instructions:

Do not use downsampling
  bash /apollo/scripts/navigator.sh left.smoothed right.smoothed --navigator_down_sample=0
Use downsampling
  bash /apollo/scripts/navigator.sh left.smoothed right.smoothed --navigator_down_sample=1
or

  bash /apollo/scripts/navigator.sh left.smoothed right.smoothed





// QP-Spline-Path Optimizer #5078
  I have a question regarding to the QP-Spline-Path Optimizer. For each lane segment, we have 3 initial conditions and 3 end point conditions, which is enough to determine the coefficients of a 5th order polynomial, then how do we optimize our cost since the program is already over constrained?

I understand that the smoothness constraints was used to smooth out the joint, however, it is essentially the same thing as the initial constraint for the following lane segment. I'm wondering why should we still bring up the smoothness joint constraint in this context?

Thanks so much, I'm looking forward to your reply.
Only when you already know the exact value of the 3 initial + 3 end conditions, a 5d polynomial is determined. But here you only have the smooth condition: y0=y1, dy0=dy1... without knowing their value. You can use a qp-solver to find the exact value.

Only when you already know the exact value of the 3 initial + 3 end conditions, a 5d polynomial is determined. But here you only have the smooth condition: y0=y1, dy0=dy1... without knowing their value. You can use a qp-solver to find the exact value.



// Maps File Format and Location #5055
I managed to convert an osm map of a campus to .xodr(open drive format). Could I use this file as a new map in Apollo? I read that it uses the open drive format but I could not find where these maps are so that I could add my own. I may be totally wrong with this. Do you have any advice as how to take a real world map and use it?


The current code takes file ends with '.xml' for open drive format map. Two places which is loaded are as the following:

	for path planning
	int HDMapImpl::LoadMapFromFile(const std::string& map_filename) {
	  Clear();
	  // TODO(startcode) seems map_ can be changed to a local variable of this
	  // function, but test will fail if I do so. if so.
	  if (apollo::common::util::EndWith(map_filename, ".xml")) {
	    if (!adapter::OpendriveAdapter::LoadData(map_filename, &map_)) {
	      return -1;
	    }
	  } else if (!apollo::common::util::GetProtoFromFile(map_filename, &map_)) {
	    return -1;
	  }
	  return LoadMapFromProto(map_);
	}

	for routing
	bool GraphCreator::Create() {
	  if (common::util::EndWith(base_map_file_path_, ".xml")) {
	    if (!hdmap::adapter::OpendriveAdapter::LoadData(base_map_file_path_,
	                                                    &pbmap_)) {
	      AERROR << "Failed to load base map file from " << base_map_file_path_;
	      return false;
	    }
	  } else {
	    if (!common::util::GetProtoFromFile(base_map_file_path_, &pbmap_)) {
	      AERROR << "Failed to load base map file from " << base_map_file_path_;
	      return false;
	    }
	  }

	  ...
	}
The map files should be put under /apollo/modules/map/data/blahblah. And you also need to change /apollo/modules/common/data/global_flagfile.txt to add your map folder:

--map_dir=/apollo/modules/map/data/blahblah

After these are done, relaunch the planning module and routing service. Or simply you can restart everything.

I have not tried this format myself. Good luck and please let me know if it works.



// Question about planning and control. #4992
Hello,
I have a question about trajectory points in planning and control.

In planning, the points in trajectory is the center of rear axle of vehicle.
In control, the points used in MPC controller is the centroid of vehicle.
Can they work well to follow the path?

Thanks.

When a vehicle deviates from the planned trajectory, usually a heading error (rather than a lateral error) appears first, causing a forward-moving vehicle to veer off a predicted course, which translates to a lateral error. In other words, lateral errors lag heading errors. By moving the controller origin forward, lateral errors appear sooner upon vehicle deviation, which improves controller stability.

Planning, in theory, does not need to know where a controller thinks the vehicle origin is. All it cares is that the vehicle is able to stay on a planned trajectory with small predictable deviations.



// 关于EMPlanner的一个问题 #5796

EM规划算法分为路径规划和速度规划2个阶段，在做路径规划的时候就已经避开了障碍物，速度规划的时候为什么还要考虑前方障碍物来确定跟车超车策略呢？
路径规划时避开的不是障碍物，比如限行路段、封闭路段等，避开的是道路

 path planning could not completely avoid some moving obstacles. We need to use speed planning to avoid collision.




// What are concept of 'overtake_obstacle' & 'guard_obstacle' in ChangeLane rule of TrafficDecider #5994
I can't understand the concept of 'overtake_obstacle' & 'guard_obstacle' in ChangeLane rule of TrafficDecider. The logics don't match with their name very much, the codes are confusing.

Appreciated if given some pointers on these.

overtake_obstacle
From the logic, I think the obstacle behind the ADC and beyond the threshold distance should be taken care of as 'overtake_obstacle'. Why need to take of those far behind ADC instead of the nearing obstacles?
How would these 'overtake_obstacle' be taken care of in the planing logic after?
guard_obstacle
From the logic, I think only the obstacles behind ADC and going to run into the front of ADC within a threshold distance should be taken care of as 'guard_obstacle'. Can you describe a little bit of the scenario in term of the behaviour? Why those end up beyond the threshold distance should not be taken care of?

guard and overtake obstacle are vehicles in front of or at the back of the ego vehicle when making a lane change. Please refer to the change_lane.h for more details, e. g. "

@brief This function will extend the prediction of the guard obstacle to
*guard lane change action. Due to the ST path may drive on the forward lane
*first, then slowly move to the target lane when making lane change, we need
*to make sure the vehicle is aware that it actually occupies the target lane,
*even when it is not on the target lane yet."



// question about apollo map module #6048
code version: apollo 3.0
Dear apollo developer,
when i trace planning module code,i find in navigation mode, apollo use relative map,
so planning module receive the map information from relative map, i do not understand the lane and the path what both of means, and segment ,
is there some document to describe lane 、path and segment information? such as lane-id and path-id,what is means?

According to my understanding, those three concepts in map may can be shown as following:
This is a path from A to B

     ----------------------------------------------------------------------------------------
                        lane 0
     ----------------------------------------------------------------------------------------
A                       lane 1                                                                   B
     ----------------------------------------------------------------------------------------
        |       seg0     |       seg1       |       seg2       |
     ----------------------------------------------------------------------------------------
The path in our life, is something like xxxx Road, and the the lane is how many cars can run side by side at the same time, segment is the smaller unit defined by apollo team, I think maybe it is easier for the car to change lane or overtake.



@xinwf thanks，so when we use command :
python navigator.py path_demo_2.0.bag.txt.smoothed
and open relative map module ,planning module will receive one lane and one segment.
if use command:
python navigator.py path_demo_2.0.bag.txt.smoothed path_demo_2.0.bag.txt.smoothed
planning module will receive two lane: lane 0 and lane 1,and one segment.


@freeclouds , no, in our relative map, there is no segment, just one lane, I think the segment concept is meaningful in HdMap, we haven't tried add two lanes by this way, I don't know whether it will success or not.




// decision.proto - MainChangeLane deprecated ? #6075


Also kindly let me know if the below proto buffer message is being used anywhere.

message ChangeLaneStatus {
  enum Status {
    IN_CHANGE_LANE = 1; // during change lane state
    CHANGE_LANE_FAILED = 2; // change lane failed
    CHANGE_LANE_SUCCESS = 3; // change lane failed
  }
  optional Status status = 1;
  // the id of the route segment that the vehicle is driving on
  optional string path_id = 2;
  // the time stamp when the state started.
  optional double timestamp = 3;
}
I tried to extract the lane change status using the above proto buffer messages. However it was not successful. Also I tried to put breakpoint in change_lane_decider.cc code where this message is being used, unfortunately control doesnt event go inside that code.

Could you please help me understanding whether this message is being used to fetch the status of lane change. Also is there any other to get the details.

Thank you,
KK



Change lane is supported through the combination effect from reference line provider, frame, reference line info, and a class called "ChangeLaneDecider".
Please first verify that the routing has a lane change region, otherwise the lane change will not take effect.




// Apollo planning TrafficDecider question #6090

Dear apollo developer,
apollo code version:3.0
about module : planning
I do not  understand in BuildPlanningTarget function, why include obstacle->obstacle()->IsVirtual() judegement,if the obstacle is not virtual so will always false ,how to calculation the min_s?
code:
void TrafficDecider::BuildPlanningTarget(
ReferenceLineInfo *reference_line_info) {
double min_s = std::numeric_limits::infinity();
// std::cout << "min_s"<< min_s<<std::endl;
StopPoint stop_point;
for (const auto *obstacle :
reference_line_info->path_decision()->path_obstacles().Items()) {
if (obstacle->obstacle()->IsVirtual() &&
obstacle->HasLongitudinalDecision() &&
obstacle->LongitudinalDecision().has_stop() &&
obstacle->PerceptionSLBoundary().start_s() < min_s) {
AINFO << "BuildPlanningTarget enter";
min_s = obstacle->PerceptionSLBoundary().start_s();
const auto &stop_code =
obstacle->LongitudinalDecision().stop().reason_code();
if (stop_code == StopReasonCode::STOP_REASON_DESTINATION ||
stop_code == StopReasonCode::STOP_REASON_CROSSWALK ||
stop_code == StopReasonCode::STOP_REASON_STOP_SIGN ||
stop_code == StopReasonCode::STOP_REASON_YIELD_SIGN ||
stop_code == StopReasonCode::STOP_REASON_CREEPER ||
stop_code == StopReasonCode::STOP_REASON_REFERENCE_END ||
stop_code == StopReasonCode::STOP_REASON_SIGNAL) {
stop_point.set_type(StopPoint::HARD);
ADEBUG << "Hard stop at: " << min_s
<< "REASON: " << StopReasonCode_Name(stop_code);
} else if (stop_code == StopReasonCode::STOP_REASON_YELLOW_SIGNAL) {
stop_point.set_type(StopPoint::SOFT);
ADEBUG << "Soft stop at: " << min_s << " STOP_REASON_YELLOW_SIGNAL";
} else {
ADEBUG << "No planning target found at reference line.";
}
}
}


If the obstacle is not virtual, its SL boundary will be calculated when we initialize frame.



// What does passages mean? Could you please give some graphic expressions? #6578

Sure. I mean the concept of passage in Map. It should be consisted of lanes.
"You can think 'passage' as a list of connected lanes. The vehicle can just drive forward without change lane in a passage." Is this right?

@KevinYuk There is no "passage" in map. I guess you are referring to "passage" in routing.
You can find the definition here:
https://github.com/ApolloAuto/apollo/blob/master/modules/routing/proto/routing.proto
Basically, lanes segment < passage < road

A "passage" is a path generated based on a set of nodes/points. The algorithm can be found in
https://github.com/ApolloAuto/apollo/blob/master/modules/routing/core/result_generator.cc

There might be lane-changes inside one passage.




// Attempt to modify planning_config.pb.txt failed while attempting to write a new planner #6725

This is the change that I made inside planning_config.pb.txt
standard_planning_config {
planner_type: PUBLIC_ROAD
planner_type: OPEN_SPACE
planner_type: PUBLIC_ROAD_DUMMY
planner_public_road_config {
scenario_type: LANE_FOLLOW
scenario_type: SIDE_PASS
scenario_type: STOP_SIGN_UNPROTECTED
}
}



enum PlannerType {
RTK = 0;
PUBLIC_ROAD = 1; // public road planner
OPEN_SPACE = 2; // open space planner
NAVI = 3; // navigation planner
LATTICE = 4; // lattice planner
PUBLIC_ROAD_DUMMY = 5; //
}



Okay the above error was because or wrong index I was using
planner_factory_.CreateObject( planning_config.standard_planning_config().planner_type(3))

I replace 3 with 2 to get that problem resolved, but got stuck on this new error saying

I resolved it by registering the new planner inside planner_dispatcher.cc

planner_factory_.Register(PlannerType::PUBLIC_ROAD_DUMMY, -> Planner* {
return new PublicRoadPlannerDummy(); });



// how to use modules/tools/mapshow/mapshow.py #4391
root@in_dev_docker:/apollo/modules/tools/mapshow# python mapshow.py /apollo/modules/map/data/sunnyvale_big_loop/base_map.bin





// how to get more data sent to dreamview #4411

I try to get more data from planning module send to dreamview. I see that dreamview frontend get messages sent from backend contains planningdata, which only include "dpPolyGraph" "path" "speedPlan" "stGraph"
The planningdata (module/planning/proto/planning_internal.proto) contains more information (init_point, routing, chasis...), how can I retrieve that info

I made a simple python use rospy.Subscriber('/apollo/planning',ADCTrajectory, callback), and I am able to get all that info. I want to get all that info in dreamview also.

I figured it out. I though apollo send the data from planning module directly to front end, but I am wrong. Data from all modules will be processed by the backend before sending to frontend (to reduce the size of sending data, I guess).




// path optimization and piecewise jerk optimization #9599

hello, i have two question here.

i read the EM planner paper, it use the DP to build a convex space, then optimize by QP, i think it should be great, BUT in apollo2.0, remove the qp optimization, i wonder why?
the piecewise jerk optimization seems not the convex optimization, i wonder whether the it can reach the global optimum in 4000 interaction? And 4000 interaction can make sure for the succeed in path optimization? and if failed in path optimization, what will happen?
OS Platform and Distribution (e.g., Linux Ubuntu 16.04):
Apollo installed from (source or binary):
Apollo version (1.5, 2.0, 3.5):




// Question: How to handle the situation when the size of QpSplineReferenceLineSmoother::t_knots is less than 3?
planning/reference_line/qp_spline_reference_line_smoother.cc


bool QpSplineReferenceLineSmoother::Sampling() {
  const double length = anchor_points_.back().path_point.s() -
                        anchor_points_.front().path_point.s();
  uint32_t num_spline =
      std::max(1u, static_cast<uint32_t>(
                       length / config_.qp_spline().max_spline_length() + 0.5));
  for (std::uint32_t i = 0; i <= num_spline; ++i) {
    t_knots_.push_back(i * 1.0);
  }
  // normalize point xy
  ref_x_ = anchor_points_.front().path_point.x();
  ref_y_ = anchor_points_.front().path_point.y();
  return true;
}


bool QpSplineReferenceLineSmoother::AddConstraint()

bool Spline2dConstraint::AddSecondDerivativeSmoothConstraint() 

bool ReferenceLineProvider::SmoothPrefixedReferenceLine()



We came across this issue during our testing on route (the speed profile experienced was a bit unexpected). The last lane segment of our map is about 40 meters and we set the destination to be the beginning of this last segment (using routing_request.py). As we were reaching the destination, we got the error msg "Failed to smooth prefixed reference line with anchor points". The car kept moving at its current speed and suddenly stopped at the destination point (very sharp braking).

After tracing the code a bit, I noticed it was caused by failed call to Spline2dConstraint::AddSecondDerivativeSmoothConstraint() due to t_knots.size() < 3. The default smoother_config_.max_spline_length() is 25 meters.

Here is log message:

E0523 13:03:21.062803    14 qp_spline_reference_line_smoother.cc:181] Add jointness constraint failed.
E0523 13:03:21.062803    14 qp_spline_reference_line_smoother.cc:60] Add constraint for spline smoother failed
E0523 13:03:21.062803    14 reference_line_provider.cc:587] Failed to smooth prefixed reference line with anchor points
W0523 13:03:21.063802    14 reference_line_provider.cc:444] Failed to smooth forward shifted reference line




// Laplacian smoothing inside create_map.py smoothies out the actual curvature of the lane #4235
// /modules/tools/create_map/


// about build st_boundary #4203
Since we get path_decision from dp_st_speed and already build st boundary , why we erase st boundaries and rebuild again in qp_spline_st_speed at line 87 
and what does the func " BuildFromPoints() " at line 58 in st_boundary.cc mean?
Expect for your reply, thanks

st_boundary is a concept based on a driving path. Different path corresponds to different st boundaries.
Previously, the ADC is assumed to drive following reference line,
but in qp_spline_st_speed, the ADC is assumed to drive the path created by DP PATH stage. Therefore, the st_boundary need to be refreshed.






// Question: Why need two speed optimizers in planning_config.pb.txt #4069
In file /apollo/modules/planning/conf/planning_config.pb.txt, it includes two speed optimizers:

...
    task : DP_ST_SPEED_OPTIMIZER

    task : SPEED_DECIDER

    task : QP_SPLINE_ST_SPEED_OPTIMIZER
...
If we remove QP_SPLINE_ST_SPEED_OPTIMZER, the car does not move.

It is also noticed that QP_SPLINE_PATH_OPTIMIZER is not registered with any planner.

The DP_ST_SPEED_OPTIMIZER is for decision purposes. QP_SPLINE_ST_SPEED_OPTIMIZER will generate the quantitative trajectory.

Currently, we do not use QP_SPLINE_PATH_OPTIMIZER. Only poly_path is used





// Difference between `path_obstacle` and `path_decision' #3918
There are two similar looking classes path_obstacle' and 'path_decision.
Can you please explain what are these two classes for?
Thank you again!

path_obstacle is an obstacle has projected properties on a particular path
path_decision is the decision made for obstacles on a particular path.

I am confused because path_obstacle has AddLongitudinalDecision and AddLateralDecision as a member function and path_decision also has AddLongitudinalDecision and AddLateralDecision as well. Looks like both decisions about the obstacle are made in both?

Thank you!

Decisions about obstacle are made somewhere else, e.g., by different tasks.
PathObstacle and PathDecision are classes to help organize and to manage the properties, and object decision is just one of such properties.



// about to compute whether the object is on one side of the reference line or not in the mode of planning #3769

// modules/planning/common/obstacle.cc
// BuildTrajectoryStBoundary() 

// /modules/planning/reference_line/reference_line.cc
// /modules/planning/reference_line/reference_line.cc
(1) For each obstacle, we are building an SL bounding box, where s, l are defined relative to the reference line. The definition of SLBoundary is at modules/planning/proto/sl_boundary.proto.


// /modules/planning/common/obstacle.cc
  // skip if object is entirely on one side of reference line.
    constexpr double kSkipLDistanceFactor = 0.4;
    const double skip_l_distance =
        (object_boundary.end_s() - object_boundary.start_s()) *
            kSkipLDistanceFactor +
        adc_width / 2.0;

    if (!IsCautionLevelObstacle() &&
        (std::fmin(object_boundary.start_l(), object_boundary.end_l()) >
             skip_l_distance ||
         std::fmax(object_boundary.start_l(), object_boundary.end_l()) <
             -skip_l_distance)) {
      continue;
    }

 (2) It means we are having more nudge distance for larger vehicles.

// /modules/planning/common/obstacle.cc   
third, what is the meaning when has_low and has_high is set to true and polygon_points is emplaced back?
has_low = object_moving_box.HasOverlap( {low_ref, low_ref.heading(), adc_length, adc_width});
has_high = object_moving_box.HasOverlap( {high_ref, high_ref.heading(), adc_length, adc_width});

(3) These code are used to map the obstacles to S-T graph. Each obstacle will be represented as a region on S-T graph, where a point (s, t) in the region means that the obstacle will be at a distance s on ego vehicle has path at time t. The definition of StBoundary is at modules/planning/common/speed/st_boundary.h.

What is the difference between GetApproximateSLBoundary and GetSLBoundary?
GetApproximateSLBoundary will run faster but the calculated SL boundary values are not as accurate as those from GetSLBoundary.





// How to satisfy the vehicle nonhonomic contraints #3706
modules/planning/tasks/dp_poly_path/dp_road_graph.cc
After I read the code in Dp-planning, I hava some questions:
1.In the dp planning modules, how to satisfy the vehicle contraints? I find that the dp-planning uses the QuinticPolynomialCurve1d to generate the path between two points which get from dynamic planning, but how can we know the curve has curvature is smaller than the max curvature of vehicle?
2.If all the curve in Dp-planning is out of boundary, it will also choose a curve, but actually, this curve(path) will go out of road. And is there a good way to solve this?
Also, I want to ask something about ReferenceLineSmoother, exspacially in QpSplineReferenceLineSmoother, Do this has contraint of vehicle nonholonomic in qp problem?

Thanks.

//modules/planning/reference_line/qp_spline_reference_line_smoother.cc
(1) No, it is not guaranteed. We need to check the curvature at the end of planning cycle to make sure it is smaller than the max curvature.
(2) We need to sample enough points to make the possibility that all paths are out of boundary low enough. We can check again at the end of path generation to make sure the path is valid.
(3) No, it has no constraints from vehicle.






// what does the "heuristic_speed_data" mean in the function trajectory_cost? #3577
modules/planning/tasks/optimizers/road_graph/trajectory_cost.cc

heuristic_speed_data_.EvaluateByTime(time_stamp, &speed_point);

As we are doing path and speed optimization separately, we do not know about speed at dp_poly_path. As a result, we use heuristic_speed_data and imagine it is the speed profile our car will drive. The heuristic speed can be a speed profile from last cycle.






// Question: How does apollo figure out absolute path for config files? #3040

I am trying to run canbus from command line as the following:

# export APOLLO_ROOT_DIR=/apollo
# canbus --flagfile=/apollo/modules/canbus/conf/canbus.conf --log_dir=/apollo/data/log
modules/common/data/global_flagfile.txt: No such file or directory
For instance, here is the code to define FLAGS_canbus_adapter_config_filename:

DEFINE_string(canbus_adapter_config_filename,
              "modules/canbus/conf/adapter.conf", "The adapter config file");
How does "/apollo" get prepended to the relative path? A more general question: how does apollo figure out the absolute path for those config files? Which environment variables are needed?

In the past we generally assume that the PWD is always the apollo root, so we define the paths as relative path. Recently we are trying to change them to absolute path because docker container is independent with user host environment.

To make it simple, you can change the flag to "/apollo/modules/...".





// Why path_out_lane_cost is not used? #9218

path_out_lane_cost is not used when calculate path cost, and this would cause the ADC slowly come back to lane after it out of lane.

  dp_poly_path_config {
       sample_points_num_each_level: 7
       step_length_max: 40.0
       step_length_min: 20.0
       lateral_sample_offset: 0.5
       lateral_adjust_coeff: 0.5
       eval_time_interval: 0.1
       path_resolution: 1.0
       obstacle_ignore_distance: 20.0
       obstacle_collision_distance: 0.5
       obstacle_risk_distance: 2.0
       obstacle_collision_cost: 1e8
       path_l_cost: 6.5
       path_dl_cost: 8e3
       path_ddl_cost: 5e1
       path_l_cost_param_l0: 1.50
       path_l_cost_param_b: 0.40
       path_l_cost_param_k: 1.5
       path_out_lane_cost: 1e8
       path_end_l_cost: 1.0e4
       sidepass_distance: 2.8
       navigator_sample_num_each_level: 3
   }



modules/planning/tasks/optimizers/road_graph/trajectory_cost.cc
ComparableCost TrajectoryCost::CalculatePathCost()

@KevinYuk the value of "path_out_lane_cost" is huge and it was intended to "force" adc to keep in-lane. But that is not always what we really want, e.g when adc side-pass obstacles or change lanes...
and "path_l_cost" can help "keep" adc closer to reference_line, or in lane.
BTW, in Apollo 5.0, we are not using dp_poly_path_optimizer any more.




// Why replaced DP+Optimization path planning method with Bound+optimization method #8992
It looks like DP+QP Optimization path planning method was discarded and replaced with path Bound + Optimization method in 3.5. Could you give an indication of why to make such a big change. Thank you.

Thank you for your question. The reason is the past DP + QP path optimization cannot handle complex urban driving scenarios, while current implementation (path bound + piecewise jerk method) achieves firm controlling of the path shape, thus is able to produce more flexible paths.





// Why removed side_pass scenario ?. Has it been merged to other scenarios? #8885
Thanks for the question. We had hope that side-pass of any static/low-speed obstacle can be a "infrastructure-ish" functionality rather than a stand-alone and exclusive scenario. Now the side-pass feature is incorporated as part of the path-bounds decider task. You can choose to turn it on or off by properly configuring the path-lane-borrow decider task.
Therefore, it is not an accurate description to say that side-pass is merged into lane-follow; rather, side-pass is merged into almost all the other scenarios, including but not limited to: traffic lights, stop signs, etc. Whether side-pass is turned on or off is contingent upon your specific needs. For example, if you want the vehicle to be agile, then turn side-pass on for all scenarios; if you feel it not safe to side-pass in intersections, then turn it off for those related scenarios.
Hope that adequately answers your question. Thanks.







// Any easy way to make ADC change lane and pass slow speed car ahead? #8808
Currently the lane_borrow (or sidepass) decider is only valid for static obstacle.
If there is only one referenceline, the ADC will use that path and follow the slow car now.

Is there any way to make ADC change lane so it can pass the slow speed car ahead?

In previous version there is dp_poly_path/dp_poly_path_optimizer.cc which can be tuned to consider dynamic cost weight.
In latest master, insider the path_bounds_decider.cc,
// modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.cc
// 4. Adjust the boundary considering dynamic obstacles
// TODO(all): may need to implement this in the future.




// In PathDecider solver, I found that you only add decision for static obstacles, especially for pedestrain and bycycle. How about dynamic obstacles? #6849
 
//  /modules/planning/tasks/deciders/path_decider/path_decider.cc
bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     const std::string &blocking_obstacle_id,
                                     PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  if (!MakeStaticObstacleDecision(path_data, blocking_obstacle_id,
                                  path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

Currently, we make decisions(Nudge/Stop) for static obstacles in PathDecide.
The decisions(Yield/Overtake and etc) for moving obstacles are made by STGraph.

 I read the code, and found that moving obstacles decision is made in speedDecider solver. Just as you said it's made by STGraph. However, why don't make decision for static and moving obstacles in PathDecider solver at the same time? In PathDecider, we have already got moving obstacle prediction trajectory from Prediction Module ros msg broadcast, even in SL graph (not ST graph), we could make decision(Yield/Overtake and etc) for moving obstacle.


we plan path first, and then speed based on path.
Currently we don't have SL graph. We have SL info, but that's not enough to make decision for moving obstacles without dimension of T(time).





// PathDecider tag: not-in-s, not-in-l, nearest-stop, not-nearest-stop. What do they mean? #7053
// /modules/planning/tasks/deciders/path_decider/path_decider.cc
not-in-s: the obstacle's s is not in the s range along the path we are planning, either too far away or behind us. the corresponding decision is IGNORE
not-in-l: the obstacle's l is not in the l range along the path we are planning. the corresponding decision is IGNORE
nearest-stop: this STOP obstacle decision is closest (in terms of distance) to us, compared to the STOP decisions from other obstacles, and therefore its info will be used as the STOP decision for main decision (e.g., if there are multiple STOP decisions from multiple obstacles, main decision will use the one whose stop line is ahead of us and closest to us).
not-nearest-stop: otherwise



// Regarding path decider, what's the difference between "side pass" decision and "overtake" decision? #
"SidePass" is a "lateral" object decision, similar to "Nudge". These decisions help ego vehicle to dudge obstacle(s) in lateral direction while passing them.
"SidePass" may cross lane boundaries little bit, and "Nudge" would be within the current lane.

"Overtake"/"Yield"/"Stop" are "longitudinal" object decisions when ego vehicle and other obstacles has path may cross.
OVERTAKE: ego vehicle shall arrive certain point BEFORE another vehicle/moving obstacle.
YIELD: ego vehicle shall arrive certain point AFTER another vehicle/moving obstacle.
STOP: ego vehicle shall STOP behind another vehicle/obstacle.


For more object decisions, please refer to:
https://github.com/ApolloAuto/apollo/blob/master/modules/planning/proto/decision.proto






// Any easy way to make ADC change lane and pass slow speed car ahead? #8808
/modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.cc

Thanks for the question. The current path-decision can take care of slow-speed obstacles. Here are a few parameters that you can tune to achieve your goal:


path_decider_obstacle_utils.cc line 40: Here you can modify it by deleting the "IsStatic()" check and increase the FLAGS_static_obstacle_speed_threhold to be the slow-speed threshold you want. The ADC will try to side-pass any obstacle below this given speed.
/modules/planning/tasks/deciders/utils/path_decider_obstacle_utils.cc
  // Obstacle should not be moving obstacle.
  if (!obstacle.IsStatic() || obstacle.speed() > FLAGS_static_obstacle_speed_threshold) {
    return false;
  }



/modules/planning/tasks/deciders/path_lane_borrow_decider/path_lane_borrow_decider.cc

path_lane_borrow_decider.cc line 103: The FLAGS_lane_borrow_max_speed is the speed for the ADC to start side-pass. If you want the ADC to be able to start side-pass at a faster speed, you can increase this value (at the cost of reduced safety so please be careful).
bool PathLaneBorrowDecider::IsWithinSidePassingSpeedADC(const Frame& frame) {
  return frame.PlanningStartPoint().v() < FLAGS_lane_borrow_max_speed;
}



path_lane_borrow_decider.cc line 110: Only when the ADC thinks the blocking obstacle is a long-term one, it will start side-pass. Here you can specify how many frames an obstacle is blocking can it be defined as a "long-term" blocking obstacle. For example, if you specify it to be 10, then only when the ADC sees a slow-car ahead for 10frames (1sec) will the ADC start to pass the slow-car.
bool PathLaneBorrowDecider::IsLongTermBlockingObstacle() {
  if (PlanningContext::Instance()
          ->path_decider_info()
          .front_static_obstacle_cycle_counter() >=
      FLAGS_long_term_blocking_obstacle_cycle_threhold) {
    ADEBUG << "The blocking obstacle is long-term existing.";
    return true;
  } else {
    ADEBUG << "The blocking obstacle is not long-term existing.";
    return false;
  }
}




path_bound_decider
path_bounds_decider.cc line 270: Here you see that the boundary is determined by lane-width, if you want, you can replace it with the function GetBoundaryFromRoads, what that will do is to use the entire road (likely consisting multiple lanes) as the boundary, ADC will try to use any path that it can find on the road, without considering whether it needs to borrow neighbor lane or not, etc. Normally, we don't suggest use this because it might create some safety issues. But as you can see, at line 319, when the ADC needs to pull-over, especially in emergency situation, it will use this method to find a path.

  // 2. Decide a rough boundary based on lane info and ADC's position
  if (!GetBoundaryFromLanesAndADC(reference_line_info, lane_borrow_info, 0.1,
                                  path_bound, borrow_lane_type)) {
    const std::string msg =
        "Failed to decide a rough boundary based on "
        "road information.";
    AERROR << msg;
    return msg;
  }


 // 2. Decide a rough boundary based on road boundary
  if (!GetBoundaryFromRoads(reference_line_info, path_bound)) {
    const std::string msg =
        "Failed to decide a rough boundary based on road boundary.";
    AERROR << msg;
    return msg;
  }






// Planning: lane change with SLT graph and obstacle filter #8576

I would like to ask some questions about lane change implementation the path planning. It looks like a SLT-graph is used for the path planning, but I'm confused about the lateral distance changes in function of time between two lanes. Uniformly? Didn't understand very clearly from the existing code.

Moreover, how is the moving obstacle avoidance considered during the lane change with SLT-graph. It looks like some obstacle filtering happens around it.

Thank for your help in advance.

SL graph is based on a single lane frenet frame rather than two lanes.
Moving obstacles only in ST graph.

I see, actually always project moving obstacles into the reference line.







// The usage of PathData in EM planner #8354
I am using Apollo 3.0.
I would like to ask the question about the usage of "PathData" in EM planner.
In the first step of EM planner-- inside "DpPolyPathOptimizer(...)", there is a function "bool DPRoadGraph::FindPathTunnel(...)" to generate the "tunnel" of "PathData path_data". The resolution of the "frenet_frame" in the tunnel is set as "path_resolution / 2".

My question is: if I use my own planner to replace "DpPolyPathOptimizer(...)" and generate this "tunnel" with a sequence of "frenet_frame"s and the distance between two neighbouring "frenet_frame"s is a not a constant value (for example, the distance between two neighbouring "frenet_frame"s fluctuates around 0.4 to 0.6), will this new PathData generated by myself with non-constant "path_resolution" be still suitable for following steps in EM planner (e.g. PathDecider, DpStSpeedOptimizer(), SpeedDecider(), QpSplineStSpeedOptimizer(), PolyStSpeedOptimizer())?

@hezudao23 path planning is independent from speed planning. So the path generated by your customized or different resolution shall still work with the speed planning algorithms followed.
BTW, in Apollo 5.0, we are not using dp_poly_path_optimizer any more.





// Why don't you use B-Spline curve to generate continuously differentiable path #1226
In 3D reconstruction, there is a famous clean implementation (contrast to PINV-K algo) to generate B-Spine curve Autonomous Vehicles at O(nlogn) cost where n represents the number of points. I do not understand why you used dynamic programming for spline curve generation, because DP means nearly brute force search and easy to write but no more benefits brought.

Another problem I am concerned about is that there is no detailed benchmark test (time consuming, latency, precision etc.) report about task optimizers. Could you give me more hints on big idea how you implemented it?

There are still some gaps between Perception and "Planner". I am trying to use log trace back the data flow. However, since I am not familiar with ROS, it might take times. Any help will be appreciated. For example, once caffe accepts raw features draw from Lidar points and provide features help Object Cluster to update tracker lists, and then RPC call might finish and I have no idea what happened before data frame enter into EM planner optimizers main loop.


Thanks a lot for your comments. I will provide a brief answer to the B-spline part.

In Apollo algorithm, trajectory generation is divided into two parts -- dynamic programming and quadratic programming. The DP algorithms are used to provide decisions on obstacles, which are transformed to boundary conditions in QP. We believe that with fine tuning costs, DP is good to make decisions (on obstacles, including left nudge, right nudge, overtake, yield, etc ...). We are unable to directly reach the step of curve generation.

Many methods, including B-spline, can be used for smooth curve generation, but it will be difficult to generate curves that cover all corner cases from varies environments without pre-calculated decisions.

If you have any suggestion on how to use B-spline curves to replace the current algorithm, please let us know.






I have created a random library (supporting generate control points and knots sparsely using statistic sampling algorithms ). It draws smooth curve by using De Boor algorithm adaptively (you don't need to explicitly specify how many sampling points you want)
Currently I used it for opendrive format alike map generation, and curves planning (to escape unwanted points).

If you don't have plan to support it, let me do the job. Currently it supports "Rad", "Snake", alike shapes .


@lianglia-apollo In B-spline curves, control points except the first and the last ones will never be touched by generated curves. Hence we can sample points in the corners, lanes, obstacles. I have implemented them a long time ago but I know the following truths:

it is not hard by following "de Boor" scheme or "divide and conquer" scheme based on bezier Castelijau scheme , I have codes loose coupled with OpenGL demo
by sampling control points, you can avoid anything you want
you can adjust it by increasing or decreasing degrees





// EM and Dp #1403

Hi Apollo team,

I would like to ask two questions. What is the role of EM(Expectation Maximization) here? I could not find its implementation in planning layer. Also, it seems that DP(Dynamic Programming) is being used in planning, what is the problem that DP is used to solve?
Thank you!


The current algorithm is composed of two DP and two QP(quadratic programming). The two DPs are used as a decision provider and QPs are used to generate smooth trajectories based on the decision results. The results of one planning cycle can be feed into the next planning cycle, and that is what EM refers to.




// Why planning in demo off from the lane center? #2392
In demo_2_0, I have seen that the planning trajectory is off from the lane center, especially when the vehicle making a corner. Is this intended?
Thank you.


(1) The trajectory is possible to overlap with virtual lane boundaries.
(2) The lane change is determined in routing. Routing will provide a "lane change" area that encourages the vehicle to change lanes. The routing in this bag has two lane changes in this area for demonstration purposes.
(3) It is possible to off the center if the straight lane is connected with some turns.





// relative_time check in DiscretizedTrajectory::Evaluate(const double relative_time) #2738

// /modules/planning/common/trajectory/discretized_trajectory.cc
TrajectoryPoint DiscretizedTrajectory::Evaluate(
    const double relative_time) const {
  CHECK(!trajectory_points_.empty()); 
  CHECK(trajectory_points_.front().relative_time() <= relative_time &&
        trajectory_points_.back().relative_time() <= relative_time)
      << "Invalid relative time input!";

  auto comp = [](const TrajectoryPoint& p, const double relative_time) {
    return p.relative_time() < relative_time;
  };

  auto it_lower =
      std::lower_bound(trajectory_points_.begin(), trajectory_points_.end(),
                       relative_time, comp);

  if (it_lower == trajectory_points_.begin()) {
    return trajectory_points_.front();
  }
  return util::interpolate(*(it_lower - 1), *it_lower, relative_time);
}


Considering trajectory_points_.front().relative_time() <= trajectory_points_.back().relative_time(), is the first part of compare trajectory_points_.front().relative_time() <= relative_time needed? I can be wrong about this: should the input relative_time in between front().relative_time and back().relative_time to be valid?

BTW, what is minimum number of trajectory points to form a valid trajectory? Is a single point trajectory meaningful ( CHECK(!trajectory_points_.empty());)?


The minimum number of points should be at least 2.
I guess it should be front().relative_time() > relative_time
I will create a fix






// 


























/////////////////////////////////////////////////////

./integration_tests
./integration_tests/navigation_mode_test.cc
./integration_tests/planning_test_base.cc
./integration_tests/sunnyvale_loop_test.cc
./integration_tests/planning_test_base.h
./integration_tests/garage_test.cc
./integration_tests/sunnyvale_big_loop_test.cc


./navi/decider/navi_path_decider_test.cc
./navi/decider/navi_speed_decider_test.cc
./navi/decider/navi_speed_ts_graph_test.cc
./navi/decider/navi_obstacle_decider_test.cc


./planner/navi/navi_planner_test.cc
./planner/navi_planner_dispatcher_test.cc
./planner/public_road/public_road_planner_test.cc
./planner/open_space/open_space_planner_test.cc
./planner/std_planner_dispatcher_test.cc
./planner/rtk/rtk_replay_planner_test.cc


./reference_line/cos_theta_reference_line_smoother_test.cc
./reference_line/spiral_reference_line_smoother_test.cc
./reference_line/qp_spline_reference_line_smoother_test.cc

./tasks/rss/decider_rss_test.cc
./tasks/deciders/side_pass_safety_test.cc
./tasks/deciders/side_pass_path_decider_test.cc
./tasks/optimizers/proceed_with_caution_speed/proceed_with_caution_speed_generator_test.cc
./tasks/optimizers/poly_vt_speed/piecewise_poly_vt_speed_sampler_test.cc
./tasks/optimizers/poly_vt_speed/piecewise_poly_speed_curve_test.cc
./tasks/optimizers/poly_vt_speed/piecewise_poly_speed_profile_test.cc
./tasks/optimizers/dp_st_speed/dp_st_graph_test.cc
./tasks/optimizers/st_graph/st_boundary_mapper_test.cc
./tasks/optimizers/st_graph/speed_limit_decider_test.cc
./tasks/optimizers/st_graph/st_graph_data_test.cc
./tasks/optimizers/road_graph/comparable_cost_test.cc
./tasks/optimizers/road_graph/trajectory_cost_test.cc


./math/curve_math_test.cc
./math/finite_element_qp/fem_1d_expanded_jerk_qp_problem_test.cc
./math/finite_element_qp/fem_1d_expanded_qp_problem_test.cc
./math/finite_element_qp/fem_1d_linear_qp_problem_test.cc
./math/curve1d/quartic_polynomial_curve1d_test.cc
./math/curve1d/cubic_polynomial_curve1d_test.cc
./math/curve1d/piecewise_quintic_spiral_path_test.cc
./math/curve1d/quintic_polynomial_curve1d_test.cc
./math/smoothing_spline/spline_2d_solver_test.cc
./math/smoothing_spline/piecewise_linear_constraint_test.cc
./math/smoothing_spline/active_set_spline_1d_solver_test.cc
./math/smoothing_spline/osqp_spline_2d_solver_test.cc
./math/smoothing_spline/spline_1d_constraint_test.cc
./math/smoothing_spline/spline_1d_kernel_test.cc
./math/smoothing_spline/spline_2d_constraint_test.cc
./math/smoothing_spline/piecewise_linear_kernel_test.cc
./math/smoothing_spline/osqp_spline_1d_solver_test.cc
./math/smoothing_spline/spline_2d_kernel_test.cc


./tuning/autotuning_mlp_net_model_test.cc
./tuning/speed_model/autotuning_speed_feature_builder_test.cc
./tuning/speed_model/autotuning_speed_mlp_model_test.cc
./tuning/autotuning_raw_feature_generator_test.cc


./open_space/coarse_trajectory_generator/hybrid_a_star_test.cc
./open_space/coarse_trajectory_generator/node3d_test.cc
./open_space/coarse_trajectory_generator/reeds_shepp_path_test.cc
./open_space/trajectory_smoother/distance_approach_ipopt_interface_test.cc
./open_space/trajectory_smoother/dual_variable_warm_start_ipopt_interface_test.cc
./open_space/trajectory_smoother/distance_approach_problem_test.cc
./open_space/trajectory_smoother/dual_variable_warm_start_problem_test.cc


./scenarios/traffic_light/right_turn_unprotected/traffic_light_right_turn_unprotected_scenario_test.cc
./scenarios/side_pass/side_pass_scenario_test.cc
./scenarios/stop_sign/stop_sign_unprotected/stop_sign_unprotected_scenario_test.cc
./scenarios/lane_follow/lane_follow_scenario_test.cc


./testdata
./testdata/sunnyvale_big_loop_test
./testdata/navigation_mode_test
./testdata/garage_test
./testdata/sunnyvale_loop_test



./common/speed_limit_test.cc


// ./common/frame_test.cc



// ./common/obstacle_test.cc








//./common/indexed_list_test.cc














./common/speed_profile_generator_test.cc


// ./common/trajectory/publishable_trajectory_test.cc


// ./common/trajectory/discretized_trajectory_test.cc


// ./common/ego_info_test.cc

// ./common/reference_line_info_test.cc


// ./common/indexed_queue_test.cc


// ./common/testdata   include: sample_prediction.pb.txt


// ./common/path/discretized_path_test.cc   




// ./common/path/frenet_frame_path_test.cc







./common/speed/st_boundary_test.cc