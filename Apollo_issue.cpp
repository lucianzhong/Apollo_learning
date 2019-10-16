
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

 once the path has been planed, we make decision for moving obstacles based on STGraph, and a OVERTAKE/YIELD/STOP decision (which would be achieved via speed) would be made if their paths may cross ego vehicle  path.

In your second graph, ADC may slowdown(follow) or yield to that moving obstacle depending on its path/speed/heading.



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
// 请问lattice_planner里面的Auto Tuning是用来干什么的? #5730
Auto Tuning部分的目的是通过机器学习的方式来给代价函数的参数调优，现在相关工作还在开发之中。
这种学习可以理解为根据场景自动为那六个代价函数分配权值吗？
按照我的理解，Auto-tuning是学习如何根据场景来评估规划出来的trajectory，从中选择出一条最优的。它并不是去学习如何分配权重，而是学习如何给出一个cost



// Question about planning and control. #4992
Hello,
I have a question about trajectory points in planning and control.

In planning, the points in trajectory is the center of rear axle of vehicle.
In control, the points used in MPC controller is the centroid of vehicle.
Can they work well to follow the path?

Thanks.

When a vehicle deviates from the planned trajectory, usually a heading error (rather than a lateral error) appears first, causing a forward-moving vehicle to veer off a predicted course, which translates to a lateral error. In other words, lateral errors lag heading errors. By moving the controller origin forward, lateral errors appear sooner upon vehicle deviation, which improves controller stability.

Planning, in theory, does not need to know where a controller thinks the vehicle origin is. All it cares is that the vehicle is able to stay on a planned trajectory with small predictable deviations.





// how do you define smoothness cost in trajectory.cc #5017

Hi,

I'm reading apollo's code for the planning module, however got stuck on figuring out how you properly define the trajectory cost, especially smoothness cost. could you share some academic paper to refer to?

Thanks so much for your help, really appreciate it.

Best

Here are two books I have learned, one is "Learning with Kernels: Support Vector Machines, Regularization, Optimization, and Beyond", the other is "kernel methods for pattern analysis".

Hope this would help you:)




// Trajectory stitching, why? #5033

Hi Team Apollo,

Can you please explain what is the purpose of stitching trajectories in planning?
Is stitching between lastly published trajectory and something else?
Thank you.

You dont want the car change trajectory every time planner run -> car will change direction frequently
So you keep a portion of previous plan trajectory and stitch it to new trajectory.

Trajectory is much longer than the length of the ego car.



// 

source /apollo/scripts/apollo_base.sh

cyber_launch start /apollo/modules/dreamview/launch/dreamview.launch







1. 

  RTK Replay planning mode with additional planning tasks and driving scenarios
  It is it possible to configure and run planning module in RTK Replay mode with ability to change line or emergency stop to avoid obstacles collision?
  No, you need use standard mode for those functions.



2.

  Hello,

  The following code confuses me, anyone can explain its grammar？

  []() -> LocalizationBase* { return new RTKLocalization(); });

  []()-> means what?
  It is in Localization.cc of apollo1.0 and line 38.

  thanks.

   @krishnatoshniwal
   
  krishnatoshniwal commented on May 6
  Its a lambda function with no input parameter and the return object having the type LocalizationBase*. Look up on lambda expressions here
  https://en.cppreference.com/w/cpp/language/lambda



3.

    Steps to reproduce the issue:
    Please use bullet points and include as much details as possible:
    bash ./docker/scripts/dev_start.sh
    bash ./docker/scripts/dev_into.sh
    bash ./apollo.sh build
    bash scripts/bootstrap.sh
    Tasks: Sim Control: ON
    Module Controller: Routing: ON
    Module Controller: Planning: ON
    Default Routing Sunnyvale Loop: ON
    Gives me the screen shot below. Planning automatically OFF.

      

4.
  // canbus component does not exit gracefully #8053

Describe the bug
As subject

To Reproduce
Steps to reproduce the behavior:

Launch canbus component using mainboard
>>mainboard -d /apollo/modules/canbus/dag/canbus.dag
Shutdown it using Ctrl+C
The console shows:
terminate called without an active exception
Abort 
Cause
No cleanup of those handles: vehicle_controller_, can_receiver_, can_sender_, and can_client_.

Fix
The cyber component has a cleanup hook function called Clear() which will be called inside ComponentBase::Shutdown(). Override of this Clear() function inside CanbusComponent fixes the issue.

void CanbusComponent::Clear() {

  can_sender_.Stop();
  can_receiver_.Stop();
  can_client_->Stop();
  vehicle_controller_->Stop();

  AINFO << "Cleanup Canbus component";
}



5. 

There are three relative map mode in apollo 2.5:
Mode 1: perception
Mode 2: navigator + perception
Mode 3: HDMap + navigator + perception.

Does apollo 3.5 also support these modes?

Those modes are not supported in 3.5 yet.



6. // How can I test the control module without a real car?


Apollo version:3.5
I have been trying to test the performance of the sample vehicle lincoln mx8 under a given complex path. After I compiled control_component_test.cc in apollo/modules/control and manually input a straight path which started at (0,0,0) and ended at (4.6,0,0), I found the .INFO file like this:

Log file created at: 2019/03/25 13:34:28
Running on machine: in_dev_docker
Log line format: [IWEF]mmdd hh:mm:ss.uuuuuu threadid file:line] msg
I0325 13:34:28.812129 11484 control_component.cc:42] Control init, starting ...
I0325 13:34:28.837960 11484 control_component.cc:48] Conf file: /apollo/modules/control/testdata/conf/control_conf.pb.txt is loaded.
I0325 13:34:28.837970 11484 control_component.cc:50] Conf file: is loaded.
I0325 13:34:28.837975 11484 controller_agent.cc:36] Only support MPC controller or Lat + Lon controllers as of now
I0325 13:34:28.838163 11484 lat_controller.cc:84] Using LQR-based Lateral Controller
I0325 13:34:28.838241 11484 lon_controller.cc:76] LON_CONTROLLER used.
I0325 13:34:28.838587 11484 lat_controller.cc:250] Lateral control gain scheduler loaded
I0325 13:34:28.838889 11484 lat_controller.cc:155] LQR-based Lateral Controller begin.
I0325 13:34:28.838897 11484 lat_controller.cc:156] [LatController parameters] mass_: 2080, iz_: 4208.3, lf_: 1.4224, lr_: 1.4224
I0325 13:34:28.838944 11484 controller_agent.cc:94] Controller init done!
I0325 13:34:28.838956 11484 lon_controller.cc:128] Control calibration table loaded
I0325 13:34:28.838973 11484 lon_controller.cc:129] Control calibration table size is 2694
I0325 13:34:28.840996 11484 controller_agent.cc:94] Controller <LON_CONTROLLER> init done!
I0325 13:34:28.850347 11484 control_component.cc:98] Control resetting vehicle state, sleeping for 1000 ms ...
I0325 13:34:29.850587 11484 control_component.cc:104] Control default driving action is STOP
E0325 13:34:29.852226 11491 control_component.cc:265] Chassis msg is not ready!
I0325 13:34:29.868499 11495 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.874752 11493 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.885205 11491 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.896140 11501 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.907552 11500 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
I0325 13:34:29.917747 11486 lon_controller.cc:413] the last point found in path and speed > speed_deadzone
...

Now I'm wondering if I have missed some important configuration, or it's just unable to test the control module without a real car (with real GPS message input)?


Not as of now, but stay tuned ! 






// 导航模式下路径规划问题 #5805

代码版本：apollo 3.0
问题描述：
在使用导航模式时，在dreamview里面仿真，打开相关模块，回放录制的数据包
rosbag play /apollo/data/bag/2018-09-25-20-04-55.bag --topics /apollo/perception/obstacles /apollo/localization/pose /apollo/sensor/gnss/corrected_imu /apollo/sensor/gnss/imu /apollo/sensor/gnss/odometry /apollo/prediction /apollo/canbus/chassis
当数据包已经播放完，但是planning模块、relative map模块仍有数据输出



当在实车测试时，如果车已经到了终点，需要人手动关闭planning等相关模块吗？如果不关闭此时control模块仍不断接收planning模块输出的数据，车无法停止。

在路测中，到达终点后需要人工接管。目前相对地图的逻辑是 完成指引线后，会继续延实时车道线自动驾驶


从前面的提供的信息看，如果车行驶完通过指引线生成的道路后，会继续使用感知得到的信息生成实时车道线，如果想将车在行驶完通过指引线生成的道路停车，需要人工执行停车动作。我们想让车在行驶完通过指引线生成的道路停车，从code中看到在
从planning模块中输出的/apollo/planning topic信息：
decision {
main_decision {
stop {
reason_code: STOP_REASON_DESTINATION
reason: "stop by REF_END_0_Path from navigation line index 0"
stop_point {
x: 21.9748137816
y: 0.106479256295
}
stop_heading: 0.00285595784381
change_lane_type: FORWARD
}
}
object_decision {
decision {
id: "REF_END_0_Path from navigation line index 0"
perception_id: -1025593075
object_decision {
stop {
reason_code: STOP_REASON_DESTINATION
distance_s: -0.5
stop_point {
x: 21.9748137816
y: 0.106479256295
z: 0.0
}
stop_heading: 0.00285595784381
}
}
}
}
vehicle_signal {
turn_signal: TURN_NONE
}
}

前面这个信息时一个停车的信号，但是这个信号不是在行驶结束发出，在行驶过程中就会通过planning 的topic输出。所以这个reason_code：STOP_REASON_DESTINATION，主要的功能是什么？主要是在dreamview上面显示停止信号？如果要实现行驶完指引线就停车，能否让control模块检测这个reason_code：STOP_REASON_DESTINATION实现停车功能？
另外在code中发现apollo中将终点虚拟成一个obstacle，如果我们车在相对地图模式行驶过程中遇到前方有人或其他障碍物，控制车停车的信息也是通过planning的topic将reason_code发出吗？在potobuf文件中
看到关于traffice rule的stop枚举其中有STOP_REASON_OBSTACLE，想请教一下apollo在相对地图这种模式中遇到障碍物control模块是从哪儿接收的停车信息


/modules/planning/proto/decision.proto
enum StopReasonCode {
  STOP_REASON_HEAD_VEHICLE = 1;
  STOP_REASON_DESTINATION = 2;
  STOP_REASON_PEDESTRIAN = 3;
  STOP_REASON_OBSTACLE = 4;
  STOP_REASON_PREPARKING = 5;
  STOP_REASON_SIGNAL = 100;  // only for red signal
  STOP_REASON_STOP_SIGN = 101;
  STOP_REASON_YIELD_SIGN = 102;
  STOP_REASON_CLEAR_ZONE = 103;
  STOP_REASON_CROSSWALK = 104;
  STOP_REASON_CREEPER = 105;
  STOP_REASON_REFERENCE_END = 106;  // end of the reference_line
  STOP_REASON_YELLOW_SIGNAL = 107;  // yellow signal
  STOP_REASON_PULL_OVER = 108;      // pull over
  STOP_REASON_SIDEPASS_SAFETY = 109;
  STOP_REASON_PRE_OPEN_SPACE_STOP = 200;
  STOP_REASON_LANE_CHANGE_URGENCY = 201;
}


目前还不支持目的地停车。如果要修改这样的逻辑： 大概可以看下 3.0下的
/planning/toolkits/deciders/destination.cc: 48行
/planning/reference_line/reference_line_provider.cc: 408


1、/home/alan/apollo/modules/map/pnc_map/route_segments.cc
bool RouteSegments::StopForDestination() const { return stop_for_destination_; }

void RouteSegments::SetStopForDestination(bool stop_for_destination) {
stop_for_destination_ = stop_for_destination;
}
2、/home/alan/apollo/modules/planning/common/frame.cc
if (segments_iter->StopForDestination()) {
is_near_destination_ = true;
3、/home/alan/apollo/modules/planning/tasks/traffic_decider/destination.cc
if (!frame->is_near_destination()) {
return Status::OK();
}
4、/home/alan/apollo/modules/planning/common/frame.h
const bool is_near_destination() const { return is_near_destination_; }
frame->is_near_destination()这个对终点的判断是来自于routing模块中读取的终点信息进行的判断，在navigation模式中没有routing模块，通过对frame->is_near_destination()相关信息进行处理能够判断车是否到终点？
code中看在导航模式下没有路由信息，在每一次规划的路径段中都会输出STOP_REASON_DESTINATION这个信息，这个信息仅仅是在dreamview中进行显示，无法根据这个stop reason做出停车的动作，我的理解对吗？








// why need traffic_light_unprotected_left(right)_turn_scenario #8559

For example, a green light with a left arrow on it, it's a protected left turn, which means we have the right of way. As we are protected and theoretically don't need to consider anyone else.(Actually we will avoid if anyone is running the red light).
But when we turn left on a junction with only solid green light without an arrow, we have to yield the right of way to an oncoming car if they are going straight. In this case, we are considered as unprotected left turn.


@HongyiSun As you mentioned even if for protected left turns you have to consider vehicles running the red light, then I wonder what exactly the difference is for the planning algorithm if there is protection or not? What I mean is, you have to yield oncoming cars (even if they are running the red light) no matter there is protection or not.


the reason we have different scenarios here is just because they have to run in different stages or be implemented in different way in some stage.
we have protected_turn vs unprotected_turn because in unprotected_turn scenario(s), we'll have a stage of "creep".
We have unprotected_right_turn vs unprotected_left_turn because we'll have different implementation to handle creep stage for right and left turn respectively. For right turn, we'll creep out of stop line of the intersection, for left turn, we'll need creep further.

A scenario is basically a sequence of stages. That is how we decide if a scenario is needed to handle a driving situation.

i known creep action, which is another part I'm not quit understand, will, let's discuss that later.
however, here, my question is why split traffic light into two parts, if we do same aciton in trafficlight junction.
as you said, wether protected or not, ego would cross stop line, yield dangers and moving slowly, why we put this issue together making it to a general case, through defining scenario?


even for a human driver, the behavior in an unprotected and protected left turn is different. When you have a Protected turn, you scan the road for a possibility of an oncoming vehicle that has overshot the light but can still drive out normally as you "assume" that people generally follow the rule.
But in an unprotected turn, you are yielding to oncoming traffic, which means you creep forward slowly until the lane is completely safe to cross, sometimes you will need to creep until the very last lane to then exit the junction, but in protected, once you have scanned the roads and are clear of doubt, you no longer creep but can drive the rest of the junction smoothly.
Let me know if you have questions.




// apollo 3.0 navigation mode question #5779

Navigation mode supports multiple lanes and lane change in Apollo 3.0. You could use following command to send out multiple navigation lines:

dev_docker:/apollo/modules/tools/navigator$python navigator.py navigation_line_1 navigation_line_2

in this example, navigation_line_1 navigation_line_2 could be navigation line on two adjacent lanes. And navigation_line_1 has higher priority than 2.





// 关于相对地图+lattice 多车道超车问题 #5897

code版本： apollo 3.0
系统版本： ubuntu 16.04
问题描述：
如issue 5779讨论：#5779
apollo 3.0相对地图导航模式支持多车道超车，我们使用relative map+ lattice+navigation模式，
在对code分析中，lattice_planner.cc是在横向-0.5~0.5，纵向10m、20m、40m、80m范围根据指引线进行路径规划，也就是lattice_planner.cc是在本车道(当前指引线)进行规划吗？
lattice_planner.cc路径：
/home/alan/apollo/modules/planning/planner/lattice/lattice_planner.cc
如果前方有障碍物需要变换车道，这个时候需要临近车道的指引线给lattice_planner.cc吗？这个指引线是通过reference_line_provider.cc提供的吗？
reference_line_provider.cc地址：
/home/alan/apollo/modules/planning/reference_line/reference_line_provider.cc
即如果前方有障碍物需要换车道时是通过变换lattice_planner.cc的指引线进行的吗？

modules/planning/planner/lattice/lattice_planner.cc


@quhezheng,感谢指点，我主要阅读了lattice相关的规划算法，在trace apollo 规划这部分代码中，下面是我的理解，你看看有没有问题：
1、规划模块从相对地图模块接收到reference line，然后将这些reference line存放在reference line info这个类中，用python navigatior xxxx.smothed 脚本播放轨迹，播放几个轨迹，便有几个reference line，也便有几个reference line info。然后对每一个reference line info求取车的横向和纵向规划轨迹束，在求取纵向轨迹的过程中会考虑三种情况即超车、跟车、遇到障碍物及交通信号等停车，根据每一种情况分别进行轨迹生成，生成完横向纵向轨迹然后通过评价函数选取每一对横向纵向轨迹束进行打分，
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
2、在初始化frame类时即Frame::Init()会调用bool Frame::CreateReferenceLineInfo()，在bool ReferenceLineProvider::GetReferenceLines中从相对地图处获取reference line和segment，在获取完信息后存放在ReferenceLineInfo类指针中。同时在bool Frame::CreateReferenceLineInfo()中通过如下判断本车道优先还是临近车道优先，
if (FLAGS_enable_change_lane_decider &&
!change_lane_decider_.Apply(&reference_line_info_)) {
AERROR << "Failed to apply change lane decider";
return false;
}
但是有个问题是这儿有一个FLAGS_enable_change_lane_decider 宏，如果盖红设置为false，不进入该逻辑，那么车道优先级是车当前车道为最优优先级吗？




// Longitudinal controller calibration and debugging #3271

I am also wondering on which vehicle has this controller been tested?

It seems that it is just doing a simple PID controller with some accounting for dead zone when breaking/applying throttle. I have never seen a vehicle that would have a linear response over 0-80mph range and would do speed control with only one PID controller.

In addition how do you tune parameters for PID and make sure that your controller is reposnsive yet it does not overshoot?


I am going to try and answer my own questions from the observations I have made these past few days.

I did not know the expression deadzone but it most probably refers to the throttle and brake deadband.
The station controller " track the station error between the vehicle trajectory reference and the vehicle position" (/docs/howto/how_to_tune_control_parameters.md).
For a given speed and a desired acceleration, the calibration table return a throttle or brake command. As mentioned previously, it can be generated thanks to a set of python scripts located in /modules/tools/calibration.
I am still looking for answers regarding my second question.

@ubercool2 I believe this controller was tested on a Lincoln MKZ only. The PIDs are configured in /modules/control/conf/lincoln.pb.txt.


Deadzone is deadband, essentially it describes the non-working zone for actuators.

The statement about the station error is right.

The similar controllers been tested on various vehicle platforms, including different sedans, minibus and more vehicle types. -- with different calibrations. The PID is used to handle the linear part while the calibration is used for non-linear part. @ubercool2 So you are right you will never see a vehicle a linear response over 0-80mph range and if you see closer to the code you will not see this assumption in Apollo controller design either.



deadzone or deadband are just two control terminologies people always use it one or another. Sorry if I use deadzone instead of headband and this cause confusion to you.
a. https://link.springer.com/article/10.1007/s11012-016-0563-3.
b. https://pdfs.semanticscholar.org/e7ec/ae5fb3b0b96331a1bd4052b9e1db9649cce7.pdf
c. https://pdfs.semanticscholar.org/7f1c/dbcf0b4be692f8bbb4d31329d9932f1f018a.pdf

The application of the controller, specific about "sedans, minibus and more vehicle types" is beyond the scope of this open-sourced code and sorry that I can not elaborate more on that. But one fact is that this controller did work well on the non-Dataspeed DBW interface cars. 　

Lacking of documentation is one significant problem in Control/Canbus/Tooling part. Another thing I should admit is that a lot of the open issues are not answered in time. We are aware of that and we will try our best to document more. Thanks and in the meanwhile we would appreciate if the community would help adding more comments or documents.


How do I use this controller? In particular how do I obtain calibration values?

You can run the controller by running the control script, ./script/control.sh. As I mentionned previously, the calibration table can be generated with a set of python scripts located in /modules/tools/calibration, it comes with documentation, /docs/howto/how_to_update_vehicle_calibration.md.

@Capri2014, thank you for the confirmations. Could you go over my second question (describing the longitudinal controller debug messages) and I will close the issue.


@bbidault2 Sorry this one previously did not have my name labeled so I had a hard time founding it back.

In short the debug messages just help to capture how good the longitudinal tracking is for offline analysis. We have station-velocity-acceleration loop with first two as close loop and last as open loop.
And for every loop, error = reference - feedback. Preview is off for 2.0 so you can ignore things about it as of now. Finally when we got acceleration, we use acceleration and current speed as the lookup table input and output is either throttle or brake.

station_reference: 1.45842000403
station_error: 1.23437740756
station_error_limited: 1.23437740756
preview_station_error: 1.40005740595
speed_reference: 0.801999986172
speed_error: 0.226802970228
speed_controller_input_limited: 0.350240710985
preview_speed_reference: 0.84399998188
preview_speed_error: 0.268802965936
preview_acceleration_reference: 0.436101042652
acceleration_cmd_closeloop: 0.141147006527
acceleration_cmd: 0.577248049179
acceleration_lookup: 0.577248049179
speed_lookup: 0.577000021935
calibration_value: 18.061075906
throttle_cmd: 18.061075906
brake_cmd: 0.0
is_full_stop: false
slope_offset_compensation: -0.000468702950871


@bbidault2 It is actually used to log and graph, if you turn on "PnC monitor" under dream view, you can see both the real time plot for planning/control ( under different tabs)




// Reference for Trajectory Cost Calculation #8680

Could you please provide a reference document how the cost is being calculated for trajectories in trajectory_cost.cc file. It would be great help in understanding the overall planning of the Ego vehicle.
/modules/planning/tasks/optimizers/road_graph/trajectory_cost.cc



// when ADC is sidepassing obstacle A, at stage_pass_obstacle, perception module gives a new obstacle at the front of obstacle A, can apollo 3.5 deal with this scence? #6991

@guoweiwan In Apollo 3.5, the SidePass feature supports to side-pass static obstacle(s) if doable, and therefore we don't expect the targeted obstacle change or move. Once we identify the obstacle to side-pass, we plan a trajectory based on its location, size etc. And therefore if there's a NEW obstacle appears in front of it (I assume you meant to say very close), we won't "re-plan" for that. If our originally planned trajectory doesn't work well to side-pass this NEW obstacle at the same time, we may fail.
We will provide better support in future release though.

Hope this answered your question.
Thanks for supporting Apollo!





// Spline 1d solver: kernel cost function construction #8600
I am trying to use the spline smoothing math package and I have a question about that. About using spline1d solver, is there any cost function for optimization besides the reference trajectory from AddReferenceLineKernelMatrix(x_coord, fx_guide, weight). I mean if we don't specify the reference line, does the solution of solver remain unique?

I have checked a while in the kernel header file and .cc file and didn't figure out very confidently.

Thanks so much, I am looking forward to your reply.

OK, I see, actually minimum jerk or minimum acc could be added.








// planning in navigation mode #7098
Is the trajectory point generated by the planning module in the navigation mode a point in the enu coordinate system or the flu coordinate system ?

@natashadsouza Thanks for your reply!But i still have a question about how to compute longitudinal Errors in the file /apollo/modules/control/controller/lon_controller.cc.

auto matched_point = trajectory_analyzer->QueryMatchedPathPoint(
VehicleStateProvider::instance()->x(),
VehicleStateProvider::instance()->y());

I think x,y represents the point in the enu coordinate system,in the navigation mode the trajectory point is in the flu coordinate system.And i do not see Coordinate transformation,so how are they calculated in the different coordinate system?

@quuen the algorithms are all written in the FLU coordinate system. And the Coordinate transformation does happen within the code itself.





// Prediction: In move_sequence_predictor, for computing longitudinal end state, why is the time returned for max_curvature point? #9547

In move_sequence_predictor, while fitting the longitudinal polynomial, we predict an end state in the longitudinal direction. This computation is done using the curvature information, to predict the velocity for the end state. There are multiple cases in this particular function, most of them return the estimated time as being equal to the prediciton horizon. In one case it returns the time that will be taken to reach the point with maximum curvature. Can someone explain the reasoning behind this? Look at the code below to understand what I am referring to.

std::pair<double, double> MoveSequencePredictor::ComputeLonEndState(
    const std::array<double, 3>& init_s, const LaneSequence& lane_sequence) {
  // Get the maximum kappa of the lane.
  double max_kappa = 0.0;
  double s_at_max_kappa = 0.0;
  for (int i = 0; i < lane_sequence.path_point_size(); ++i) {
    const PathPoint& path_point = lane_sequence.path_point(i);
    if (path_point.s() < init_s[0] + FLAGS_double_precision) {
      continue;
    }
    if (max_kappa < path_point.kappa()) {
      max_kappa = path_point.kappa();
      s_at_max_kappa = path_point.s();
    }
  }

  // If the max. curvature is small (almost straight lane),
  // then predict that the obstacle will keep current speed.
  double v_init = init_s[1];
  if (max_kappa < FLAGS_turning_curvature_lower_bound) {
    return {v_init, FLAGS_prediction_trajectory_time_length};
  }
  // (Calculate the speed at the max. curvature point)
  double v_end = apollo::prediction::predictor_util::AdjustSpeedByCurvature(
      init_s[1], max_kappa);
  // If the calculated speed at max. curvature point is higher
  // than initial speed, don't accelerate, just predict that
  // the obstacle will maintain current speed.
  if (v_end + FLAGS_double_precision > v_init) {
    return {v_init, FLAGS_prediction_trajectory_time_length};
  }
  // If the obstacle is already at the max. curvature point,
  // then predict that it will maintain the current speed.
  double s_offset = s_at_max_kappa - init_s[0];
  double t = 2.0 * s_offset / (v_init + v_end);
  if (t < FLAGS_double_precision) {
    return {v_init, FLAGS_prediction_trajectory_time_length};
  }
  // If the deceleration is too much,
  // then predict the obstacle follows a reasonable deceleration.
  double acc = (v_end - v_init) / t;
  if (acc < FLAGS_vehicle_min_linear_acc) {
    t = v_init / (-FLAGS_vehicle_min_linear_acc);
    return {FLAGS_still_obstacle_speed_threshold, t};
  }
  // Otherwise, predict that it takes t for the obstacle to arrive at the
  // max. curvature point with v_end speed.
  return {v_end, t};
}
Edit: I understand that the prediction is only done till the max_curvature point in the scenario I was looking at. But I am curious to understand why this is the case, as the trajectory predicted when the obsctacle is very close to this point is very short and is not reliable for planning.


@sujithvemi Thanks for your asking. As to the current trajectory generation method, we need to find an end state for polynomial fit. Typically, vehicles decelerate while it is entering a turning. Thus, we select the most representative point -- the point with the maximal curvature as the end state and set the speed at the end state as a pre-defined comfortable speed.









// Sending customized data to demo #9504

Actually, I want to control the speed of car, add some obstacles to road, control the signals and the direction of car. Can I do any of these?

https://github.com/ApolloAuto/apollo/blob/master/docs/specs/Dreamland_introduction.md





// No conrtol-module output #9431
Hi @udeto sorry I missed your reply earlier on, you need to send an auto command via https://github.com/ApolloAuto/apollo/blob/master/modules/control/proto/pad_msg.proto in order for control to be triggered for publishing control command, otherwise it would just be headers.




// Apollo and real time #3707

Dear all,
I have been looking at Apollo for the last couple of days and I have really lots of questions. I will start posting them one by one in the issues. Let me know if you want them otherwise.

In http://apollo.auto/ it is stated that Apollo is open, reliable and secure software. It is also stated that the by 2017-12 Apollo was gonna be ready for deployment in "Simple Urban Road Conditions". Now one of the most elementary things to have the system act reliably (which you want on public roads) is to have it act in real-time and deterministically. For the definition of the latter 2 please see this excellent talk: https://vimeo.com/236186712.

1. I see that you apparently use kernel with RT PREEMPT but I see no where in the code call to set the scheduler (e.g. FIFO) or to set the real time priorities. Does it mean that you just run on the normal desktop ubuntu where your programs run and share resources with wifi, bluetooth and such drivers?
2.I see logging to files or file manipulation being used everywhere: https://github.com/ApolloAuto/apollo/search?utf8=%E2%9C%93&q=fopen&type=. All of these calls are blocking and not safe at all
3.I see new and delete operator being used everwhere: https://github.com/ApolloAuto/apollo/search?l=C%2B%2B&q=delete&type=. Also on the runtime which means that you fragment the memory all the time. Which in turns means that after hours of operation you will not be able to allocate even a simple string anymore.
4.I see std::strings being used everywhere (they can not be made static).
5.I see mutexes being used everywhere: https://github.com/ApolloAuto/apollo/search?utf8=%E2%9C%93&q=mutex&type=. They can not be pre-empted or controlled with priority and hence can not be used in real time code
6.I see lots of variadic functions (e.g. printf, fprintf, ...). See why this is a dangerous flaw: https://stackoverflow.com/questions/3555583/passing-variable-number-of-arguments-with-different-type-c.
7.I do not see any memory locking (mlockall) or heap trimming (mallopt) operation that would allow memory allocation on the start only and not yield any memory back to the OS.
8. I do not see any memory supervision tools being used
9.I do not see any tracing tools being used (e.g. LTT)




// Apollo and software development process #3820
Hi, I have couple of questions with respect to Apollo SW development process. Having well defined and followed SW development process is the key to safety and possible certification.

1.Requirements. These can be high and low level and serve as an input to trace an intended feature development to its implementation and testing. I do not see them. For automotive standards such as ISO26262 this is a must.
2.Use case(s) define in which context the SW is to be used. E.g. car performing a maneuver through a 4 way stop. I also didn’t find any use case.
3.Code reviews. Are there any guidelines on how many people should review a particular pull request and what should they focus on? E.g. algorithm, style, …
4.Testing. I see quite some tests written in Apollo. But do you have an idea how much of code is tested and untested? I see that you write unit and integration tests. It would be great if you would use a code coverage tool (e.g. gcov/lcov) and display unit test coverage and integration test coverage.
5.Testing2. Do you run any SIL, HIL, torture or burn-in tests? Having coverage for those would also be great.
6.Linting. I see some mentioning of linters but couldn’t quite figure it out which ones do you use. Do you use static code analysers such as e.g. klockworks?
7.Design articles - I saw some how-tos but they do not seem to be explicitly linked to code. Design articles explains how a particular feature is to be used and what e.g. its limitations are.
8.Builds. I see that you have travis server that builds PRs.
9.There is much more checks to be applied if the code is to be automotive grade (e.g. MISRA compliance, MCDC code coverage, signed builds, etc..) but above is the bare minimum.

There is much more checks to be applied if the code is to be automotive grade (e.g. MISRA compliance, MCDC code coverage, signed builds, etc..) but above is the bare minimum.


Hi @ubercool2 , thank you for taking the time to explore Apollo and draft these questions for us. I would like to apologize that we did not get back to your questions sooner. These are all great questions and I will try to answer them individually:

The autonomous driving industry is continually evolving and if you have followed Apollo throughout, you will know that we release newer versions continuously. Our new ReadMe has a high-level description of the system and sensor requirements with both the Hardware and Software installation guides exploring these requirements in detail. If you have more specific questions about what requirements you are looking for, please let us know.
Could you provide a more specific explanation of what you are looking for? If you are talking about where you can test the driving scenarios, our simulation platform has hundreds of use-cases which you could explore once you have set-up the platform. Let me know if you need help to get that started. If you have suggestions on additional scenarios, please list them out so that we could review and hopefully include them as it will greatly benefit the developer community.
Yes, we do have code reviews. During the PR, it clearly states that “At least 1 approving review is required to merge this pull request.” Also, we have designated engineers performing this review, therefore we have not released guidelines on it as the reviewers are well aware of our guidelines and standards. Should your review not be approved, the reviewer would definitely leave a comment that includes improvements.
Great suggestion. We are currently in the process of exploring such code coverage tools. If you have a recommendation along with reasons for the recommendation, please share the same with us.
Once again, this is something we are currently in the process of exploring. Your expertise will be much appreciated.
We use Cpplint currently. No, we do not currently use Static code Analyzers.
We have increased the number of how-tos to support our developer community. If you have additional specific suggestions on documentation, please include them. We will be happy to increase our documentation to help the community.
You are correct.
Hope above information answered some of your questions. Please feel free to leave comments below if you have any additional questions. Again, we apologize for taking so long to get back to you, and thank you for your support in the Apollo Project.




// Question about making obstacles in Dreamview Simulation Mode #4822
Hi guys, I get great help every time on this board. Thanks!

I am trying to make some obstacles in Dreamview Simulation Mode and I have some questions about it.

I opened issue (#4626) and @mickeyouyou gave me the answer thankfully, so now I can make one obstacle in Dreamview Simulation Mode.

I have two more questions below about making obstacles.

Q1. How can I make several obstacles ?
I tried to make several obstacles. So I opened some terminal windows and executed the following commands.

# Terminal 1 (in modules/tools/perception)
python replay_perception.sh static_obstacle_1.json
# Terminal 2 (in modules/tools/perception)
python replay_perception.sh static_obstacle_2.json
I set id_value and x,y_coordinate_values for the two json files differently.
But it seems that they do not work properly.
Can you tell me how to fix it?

Q2. Is there any tools for creating path(trace value in json file) for the obstacle ?
In example file garage_onroad_vehicle_3.json, there are trace values and obstacle in dreamview simulationworld moves along that path. I wonder about how Apolloteam made that trace values. (using rosbag file..?)

Thanks!



@hashim19 Hi, replay_perception.sh file has been removed from master branch as @xiaoxq mentioned above. You can find that file in r3.0.0 branch.
I made two obstacles by following command line after running Dreamview.

(in dev docker, /apollo/modules/tools/perception)
python replay_perception.sh garage_onroad_static_4.json garage_onroad_vehicle_3.json
You can run replay_perception.sh file with arguments(e.g. garage_onroad_static_4.json , garage_onroad_vehicle_3.json, ...) which you want to generate.
Further, you can make your own json files.




// Any example for making obstacles in Dreamview simulation environment? #4626
I want to simulate some algorithms with Dreamview simulation environment (SimControl mode).

I wonder about how can I make obstacles in Dreamview.

I found out that there's previous similar question (#4338), but I'm not sure how to add new obstacles.

From that question (#4338), @unacao said user have to follow 2 steps.

Fill in the obstacle information according to the proto (perception_obstacle.proto).
Publish the messages.
I want to know there's any examples for following the steps.

Also, it looks like there's some example files for filling the obstacle information.
config.pb.txt or 1_perception_obstacles.pb.txt

Can I use these files or is there any reference case for using it?


hi @CCodie , in directory module/tools/perception, you can find some scripts like replay_perception.sh that can mock some obstacle and publish the message /apollo/perception/obstacles that descrbed in files garage_*.json, you can execute like :

# in modules/tools/perception
python replay_perception.sh garage_onroad_vehicle_3.json
that will publish vehicle to topic /apollo/perception/obstacles in 10hz.

or execute python replay_perception.sh garage_*.json, that will publish different obstable on or off road.




// control message distance error_code: OK msg: "planning has no trajectory point." #9568

Hi,
I want to use apollo (3.0) with the carla simulator (0.9.5.).

I have successfully connected apollo and carla, but when I pass the waypoints to apollo the car does not move.

I am sending the waypoints on the ros topic /apollo/routing_request and get a "Routing success!" message on the monitor as a result.

The planning module has the following error:

E0831 10:11:57.656193  7432 trajectory_stitcher.cc:159] the distance between matched point and actual position is too large. Replan is triggered. lat_diff = 2.10425e-07, lon_diff = 5.0594
E0831 10:12:01.297992  7432 trajectory_stitcher.cc:159] the distance between matched point and actual position is too large. Replan is triggered. lat_diff = 2.10425e-07, lon_diff = 5.1219



However the planning module publishes the message on /apollo/planning:
`trajectory_point {
path_point {
x: -3.24922968276
y: 28.2791210075
theta: 1.55190478421
kappa: -0.000431162972088
s: 24.4705144127
dkappa: 0.000184319246155
ddkappa: 0.0
}
v: 5.95111747937
a: 1.66466648484
relative_time: 6.70252118111
}
trajectory_point {
path_point {
x: -3.23777273409
y: 28.8827493509
theta: 1.55163883064
kappa: -0.000307242230296
s: 25.0742514772
dkappa: 0.000203804291745
ddkappa: 0.0
}
v: 6.12670687636
a: 1.84965984662
relative_time: 6.80252118111
}
trajectory_point {
path_point {
x: -3.22579030435
y: 29.5048805608
theta: 1.55147580696
kappa: -0.000164938721502
s: 25.696498069
dkappa: 0.000220667105494
ddkappa: 0.0
}
v: 6.32156659837
a: 2.05016769888
relative_time: 6.90252118111
}
trajectory_point {
path_point {
x: -3.2133788803
y: 30.1475223905
theta: 1.55141123627
kappa: -1.11946580009e-05
s: 26.33925974
dkappa: 0.000235441429993
ddkappa: 0.0
}
v: 6.53727650853
a: 2.26675832291
relative_time: 7.00252118111
}
trajectory_point {
path_point {
x: -3.20050131583
y: 30.8128409064
theta: 1.55143698202
kappa: 0.000154116151633
s: 27.0047028702
dkappa: 0.000248336079839
ddkappa: 0.0
}
v: 6.77547329806
a: 2.5
relative_time: 7.10252118111
}
decision {
main_decision {
cruise {
change_lane_type: FORWARD
}
}
object_decision {
}
vehicle_signal {
turn_signal: TURN_NONE
}
}
latency_stats {
total_time_ms: 5.72896003723
task_stats {
name: "DpPolyPathOptimizer"
time_ms: 1.55305862427
}
task_stats {
name: "PathDecider"
time_ms: 0.00238418579102
}
task_stats {
name: "DpStSpeedOptimizer"
time_ms: 0.0715255737305
}
task_stats {
name: "SpeedDecider"
time_ms: 0.00214576721191
}
task_stats {
name: "QpSplineStSpeedOptimizer"
time_ms: 0.707387924194
}
task_stats {
name: "ReferenceLineProvider"
time_ms: 1.27267837524
}
init_frame_time_ms: 0.00264549255371
}
routing_header {
timestamp_sec: 1567237881.59
module_name: "routing"
sequence_num: 1
}
right_of_way_status: UNPROTECTED
lane_id {
id: "8_1_-1"
}
lane_id {
id: "1_1_-1"
}
lane_id {
id: "2_1_-1"
}
engage_advice {
advice: KEEP_ENGAGED
}
trajectory_type: NORMAL

(this is of course only the end of the message)

To me it looks like the planning module is publishing a valid trajectory.

The control module however publishes the following control command on /apollo/control:
header { timestamp_sec: 1567240253.25 module_name: "control" sequence_num: 966 lidar_timestamp: 0 camera_timestamp: 0 radar_timestamp: 0 status { error_code: OK msg: "planning has no trajectory point." } } throttle: 0.0 brake: 50.0 speed: 0.0 gear_location: GEAR_DRIVE signal { turn_signal: TURN_NONE } latency_stats { total_time_ms: 0.40864944458 total_time_exceeded: false } engage_advice { advice: READY_TO_ENGAGE }


The header time for planning modules is:

timestamp_sec: 1567528938.23
module name: std_planning
sequence num: 3265
I am passing a pre defined route as a list of waypoints to the routing module on the /apollo/routing_request topic:

header {
  timestamp_sec: 1567529914.0
  module_name: "dreamview"
  sequence_num: 1
}
waypoint {
  pose {
    x: -3.45605635643
    y: 3.177764893
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45609807968
    y: 3.17774963379
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45632362366
    y: 4.17774963379
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45654964447
    y: 5.17774963379
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45677518845
    y: 6.17774963379
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45700120926
    y: 7.17774963379
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45722675323
    y: 8.17774963379
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45745277405
    y: 9.17774963379
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45767831802
    y: 10.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45790433884
    y: 11.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45812988281
    y: 12.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45835590363
    y: 13.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.4585814476
    y: 14.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45880746841
    y: 15.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45903301239
    y: 16.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.4592590332
    y: 17.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45948457718
    y: 18.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45971059799
    y: 19.1777496338
  }
}
waypoint {
  id: "2_1_-1"
  pose {
    x: -3.45980858803
    y: 20.1764068604
  }
}
I thought that maybe there was something wring with the waypoints, but the routing module seems to work fine.


@udeto Your routing is fine, the problem is the header time diff between planning (1567528938.23) and control ( 1567240253.25), they have 288684.98 sec difference...


I figured it out, the problem was that the control module threw an "estop" error when the trajectory was not yet planned. So when the planning module published the trajectory the control module was still stuck at "estop" and therefore sending the "estop" brake command constantly. I needed to send a reset command via the control pad to get the control module to re-evaluate its status. After that the control module follows the planned trajectory.

Only one problem remains, the car is moving very slow, never exceeding 6 kmph. Is there a possibility to set a higher target velocity?

then you need to dive into the details of planning/control/mapdata, the reasons can be in all three modules, I would recommend start by checking the map speed limits.





// Regarding path decider, what's the difference between "side pass" decision and "overtake" decision? #6837

"SidePass" is a "lateral" object decision, similar to "Nudge". These decisions help ego vehicle to dudge obstacle(s) in lateral direction while passing them.
"SidePass" may cross lane boundaries little bit, and "Nudge" would be within the current lane.

"Overtake"/"Yield"/"Stop" are "longitudinal" object decisions when ego vehicle and other obstacles path may cross.
OVERTAKE: ego vehicle shall arrive certain point BEFORE another vehicle/moving obstacle.
YIELD: ego vehicle shall arrive certain point AFTER another vehicle/moving obstacle.
STOP: ego vehicle shall STOP behind another vehicle/obstacle.



"avoid" decision is a special decision we design/plan for ESTOP.
When ADC is unable to self-driving properly, it'll enter estop scenario. It may hard-brake, or cruise via a safe route if possible. During this period, it still need to dudge/avoid obstacles, but the algrithmn/handling can be very different from Nudge decision in normal situation. That's the decision of AVOID.

The "avoid" decision is "lateral" object decision in safe route cruise scenario. Am I right?

@KevinYuk yes. you are right. the "avoid" decision is a "lateral" object decision in e-stop mode. (we do not have a "safe route cruise" scenario though.)




// Planning/Control issue: right turn is not working right #5879

Through the turning, seems that the front wheel is following the planned trajectory. Why is the center of the vehicle not following the trajectory? I am not sure the problem is from planning or control or the parameter of the car?

I faced the same issued when playing with the simulator. I think it may come from several sources.

Mapping from the simulator to Apollo map has an offset.
Perception did not detect the pole.
Delay between Apollo and simulator which may cause the controller delay -> sim car could not follow trajectory.
These are what I could think the problem come from. Need further investigation to verify.


I agree that the delay might be the reason since we observed that the car  longitudinal motion has a delay and you are right the pole may not be detected by Apollo. I am not very clear about mapping, so I need to read more about HDmap.

Please try Apollo 3.5. I remember one of the fix is related to this issue. Thanks.




// Who triggers the change of the reference line? #3843

Hello,

What event makes the switch of the reference line?
It is perhaps related to main_decision but not quite follow.
Thank you!


Reference line was mainly derived from routing and map.
Not sure what you mean by 'switch reference line'. It updates itself when it expands on the road or change lane.
Most of the logic is in https://github.com/ApolloAuto/apollo/blob/master/modules/map/pnc_map/pnc_map.cc and https://github.com/ApolloAuto/apollo/blob/master/modules/planning/reference_line/reference_line_provider.cc


Thank you @startcode,

By just looking at demo_2_0 play, I see the reference line is switched to the target lane. So, I was wondering lane change event triggers the switching reference line or some other event (rules?) triggers switching the reference line and hence lane change happens.

Can you elaborate little more on lane change algorithm? What triggers lane change?
For example, perception sees obstacle->traffic_rule->lane change decision made-> reference line is changed-> planning to change lane.

I see lane change related potions in several places, main_decision, change_lane_decision, pnc_map and so on.

Thank you for the great help!


Lane change is a planning option given by routing.
When there is a change lane region in routing, we will create two reference lines, one on current lane, and another on the target lane. The planning algorithm will try to make one planning trajectory on each reference line, and it will eventually select one based on traffic rules and cost functions.


lane change triggered by route from pnc map (This is true), or presence of obstacle on the lane (this feature not supported yet). Obstacle does not trigger change lane now, but it will affect change lane decision.

"reference line is changed" is not accurate. I would say "reference line is selected". Reference line is a static information derived from route and map. Multiple reference lines in a frame indicate that change lane is an option in the current frame. Planning selects reference line based on traffic condition and obstacles, and a lot of other logic.



// apollo/docs/demo_guide/





// dp_st_graph #3562

I find a little problem in dp_st_graph.cc:
in line 218:

const double delta_s_upper_bound = v0 * unit_t_ + vehicle_param_.max_acceleration() * speed_coeff;//vt + at^2

Is that should be vt+at22?
/modules/planning/tasks/dp_st_speed/dp_st_graph.cc

In our finite element method, we are using the assumption that the speed between each time points is constant, and speed can make an abrupt change between segments. So the estimated delta_s_upper_bound is represented as the function as in line 218. It should not be divided by 2.

You can make other assumptions on the speed and accelerations on a discretized S-T graph. If you assume that the acceleration between time points is constant, then you can calculate the upper bound using with the 1/2 coefficient.



// Switching lanes in custom created map #4008
Hi! I have created a map according to the modified opendrive standards of Apollo and want to run it in dreamview. Everything works except the car can't route between lanes, eg. all I want it to do is to change from lane -1 to lane -2. I've tried adding them as neighbours and in different constellations of laneOverlapGroups but nothing has worked thus far. The road is a 100m long straight. Any tips?

Cheers!


Please check the results of routing and verify "ChangeLaneType" is marked correctly. The current routing module requires a long enough range to change lane. Please also verify that the lane -1 and lane -2 have overlap long enough (several tens of meters).





// planning模块发布数据的设置 #1504

您好，
现在在看planning模块，大体执行流程可以看得明白，大体如下：
超时时执行Planning::OnTimer---> Planning::RunOnce在这里定义了trajectory_pb--->Planning::Plan()--->EMPlanner::Plan()--->optimizer->Execute()在这里调用各个task。

看到trajectory_pb在Planning::RunOnce()中定义之后，然后在Planning::Plan函数中设置其中的debug，latency_stats和decision项，然后调用planner_->Plan(stitching_trajectory.back(), frame_.get(),
&reference_line_info);
在此函数中trajectory_pb并没有作为参数，其主要的数据成员trajectory_point和path_point是在哪里设置的？
还有stitching_trajectory的作用是什么？感觉planning模块难度比较大，有没有比较详细些的资料介绍？谢谢。



正如你所见，planner_->Plan() 接收了reference_line_info，而且其实它也来自 frame_ 的成员。完成一轮计算后会调用 Frame::FindDriveReferenceLineInf() 找到一条cost最小的reference line, 调用PopulateTrajectoryProtobuf(trajectory_pb)填入进去。

stitching_trajectory是为了缝合上一帧trajectory 与当前帧，使过渡平滑。参考TrajectoryStitcher::ComputeStitchingTrajectory
You can refer docs/specs/qp_spline_path_optimizer.md, docs/specs/qp_spline_st_speed_optimizer.md, and docs/specs/reference_line_smoother.md for the major algorithms in planning module. We will try our best to complete a full document for planning in a near future.


Router will return a response containing road segments searched by A*star, an a planner will digest it to generate fine tuned trajectory data. @ahuer2435 was asking where "trajectory_pb" comes from. I guest it populated by the planner algorithm for publish. Am I right?

The "trajectory_pb" comes from the plan function in planning.cc.

Status Planning::Plan(const double current_time_stamp,
const std::vector& stitching_trajectory,
ADCTrajectory* trajectory_pb);






// Clarification needed - traffic_decider change_lane.cc #5211

/modules/planning/traffic_rules/change_lane.cc



    if (obstacle->PerceptionSLBoundary().end_s() <  adc_sl_boundary.start_s() - std::max(config_.change_lane().min_overtake_distance(), obstacle->speed() * min_overtake_time)) {
      overtake_obstacles_.push_back(obstacle);
    }

I think this overtake not mean adc want to perform overtake behavior, But when ego want to change lane, we should consider the obstacles that want to overtake the ego vehicle.The case you described is considered in the file front_vehicle.cc




// apollo1.5 创建添加HD map #641
您好，
请教两个问题，对于一个特定场地，如何创建hd map？ 如何将创建的hd map添加到apollo中？

主流HDMap制作方案主要借助于激光雷达、Camera等设备，借助于点云处理算法、图像视频算法、SLAM等技术来制作。国内的高精度地图受到相关法律法规的约束，如果有大规模的场景应用需求可以考虑与百度合作。
在Apollo中使用HDMap 可以参考文档https://github.com/ApolloAuto/apollo/blob/master/modules/map/data/README.md


嗯，看了下https://github.com/ApolloAuto/apollo/blob/master/modules/map/data/README.md，有几问题请教下：

/modules/map/data/下的地图目录中通常包含这几个文件base_map.bin，base_map.xml，routing_map.txt，base_map.lb1， default_end_way_point.txt，sim_map.bin，base_map.txt，routing_map.bin， sim_map.txt，想知道每个文件的具体作用是什么？与仿真环境中的哪些有对应关系？
贵司提供的这些文件是如何生成的？具体需要哪些传感器？apollo1.5中是否提供了相应的工具生成这些文件。
我们计划在一个简单的场地上，先把整个系统跑起来，所以想知道如何根据一个已有场地，生成可以在apollo中使用的hd map，谢谢。



如 README 所示，xml, bin, txt, lb1 都是不同的文件格式，适配不同的读取器，内容是一致的。
base_map 是基础地图;
routing_map 是routing模块所需的、对 base_map 预处理得来的地图;
sim_map 是simulation模块所需的、对 base_map 降采样以提高传输和渲染效率;
default_end_way_point 是一个proto，定义了该地图默认的终点，用户可以一键导航到这个终点。
我们建议这些文件都成套提供，避免不必要的麻烦。

如 msbeta 所述，base_map的制作比较复杂，需要的设备较多，我们没有包含在代码中，而是作为一项服务。
从base_map制作routing_map，见 modules/routing/topo_creator/topo_creator.cc
从base_map制作sim_map，见 modules/map/tools/sim_map_generator.cc
选取 default_end_way_point 可以用 modules/tools/mapshow/mapshow.py 来选择 lane_id 和 s.




建议的地图制作流程为：

原始数据采集(视觉、激光雷达、GPS等)以及处理；
地图数据生成。从步骤一生成的数据通过算法或者人工的方式获取地图数据；
地图格式组织。将地图数据转换为Apollo的高精度地图格式（可以参照base_map.xml格式,其他的地图都可以从base_map.xml生成）
这三个步骤的工具均需要自己开发，如果只是小规模的简单测试，也可以参照base_map.xml格式手工组织数据。



@ahuer2435 hello, currently I have base_map.bin, by using:
routing_map has the topology of the lanes in base_map. It can be generated by command:

 dir_name=modules/map/data/demo # example map directory
 ./scripts/generate_routing_topo_graph.sh --map_dir ${dir_name}
sim_map is a light weight version of base_map for dreamview visualization. It has reduced data density for better runtime performance. It can be generated by command:
dir_name=modules/map/data/demo # example map directory
bazel-bin/modules/map/tools/sim_map_generator --map_dir=${dir_name} --output_dir=${dir_name}

apollo/modules/map/tools/

  proto_map_generator.cc 
  sim_map_generator.cc





// Planning: pull over problem. #5857


Steps to reproduce the issue:
I'm working on Apollo planning module with customized simulator. I'm trying to make ego vehicle pulling over the curbside of the road shown in right side of it in picture 1. But this curbside is drivable by manual operation. Our HDmap doesn't have information of curbside shown in Picture 2. I checked that BuildPullOverStop() function (shown in picture 3) was called. The vehicle doesn't go to curbside and just stop on the reference line.

1.Do I need to add map info on HDMap to make pull over working? I also tried to reduce some parameters like reducing pull_over_plan_distance 55.00 to 15.00, plan_distance from 55.0 to 15.0 and operation_length from 50.0 to 10.0 to make pull over working but failed.

2.What is the expected pull over behavior? Does the vehicle stop near to the curbside?
In the planning.INFO log message, I could see message like the follow.
[DEBUG] pull_over_status: in_pull_over: true

3.Whenver the vehicle pulls over, inlane_dest_point is set. Does this mean that the vehicle does not stop near to curbside but stop on the reference line?



I think you should create a route for the curbside.
Right now, planner module in Apollo heavily depends on ref line, so consider the curbside as another lane.

Ref_lines do not work as you think. From routing, the planner will create ref_line in each lane, so there is only one ref_line in each lane, the ref_line will be similar to the routing line. When change lane, the car will use the ref_line in the target lane to plan the motion.

Turn on PNC monitor (in tasks)
While running the planner, can you see any reference line (thin blue line) in your parking lane? if not, planner did not create any ref_line in that lane, that why the car does not move there.


After testing a lot of cases, I made conclusions like the followings:
(If you have some feedback on this, let me know that.)
1.Even if there is virtual obstacle wall is on the other lane, pull over doesn't activate the lane change.
This is because routing destination on same lane doesn't make reference line on the other lane.
Only if there is destination on the other lane, there are more than one reference line.

In my test, I was setting -3.8 as buffer_buffer_to_boundary.

2.Pull over task is not different with normal stop at the destination.
Pull over implementation seems not done.


I set the lane_23 as SIDEWALK to make lane_22 right-most-lane and -3.8 as buffer_buffer_to_boundary to make the virtual obstacle wall on the other lane, 2nd lane.

Lane change:
When the destination is on the other lane, there are two lanes where lane change is working (Figure4). When the destination is on the same lane, only one lane has route unlike Figure 4. Even if there is virtual obstacle wall is on the 1st lane, there is only one red line.

Pull over:
When planning modules starts, the virtual obstacle wall is on the 1st lane. As the ADC is moving, the virtual obstacle wall moves to the 2nd lane. It seems that the ADC is not pulling over to the 2nd lane.

Conclusion:
Pull over is not activating lane change and stops at the same lane started from the starting lane as the way like normal stop behavior. (I want to check this fact!)




// Point Cloud Coordinate #8279

The cloud points that are published on /apollo/sensor/lidar128/compensator/PointCloud2 in which coordinate format are expressed?
Right-hand rule or RFU?



// Continental ARS 408 CAN总线接口协议 #8331
想知道大陆408的接口协议，怎么解析出速度，距离等信息的?




// Planning: avoiding obstacles #5732

I tried testing the planner's capability of avoiding obstacles using the /apollo/modules/tools/perception/replay_perception.py script (which is on branch 3.0) and what I found is that:

the prediction module must also be turned on(if I don't, the car just goes through obstacles)
it detects the obstacles and it stops when it gets near enough
if the obstacle takes only a small part of the lane, the car sometimes goes by it, other times it completely stops
I couldn't see quite clearly that the car does any categoric evasive maneuvres to avoid the obstacle. Is it maybe because the lane is not wide enough? Also, will the car switch lane in order to avoid obstacles?
I'm not sure how obstacle avoidance work and which planner I should use. I tested with EM. Could you check that what I found is true or not and maybe provide a little more insight?
Thank you



I would like to give some points here as I have tested some of the behaviors of the Car.

Car would stop if the lane is completely blocked or blocked more than half of the lane.
Car would nudge the obstacle if lane is partially blocked and there is sufficient room in the same lane to pass through.
Car would change the lane if the obstacle covers less than half of the lane and there exists a right or left neighboring lane. There is a wait time, car comes to stop and waits for that amount of time and then changes the lane.
Thank you,



@SanduIrina Hi Sandulrina, How did you add obstacles to the dreamview? Thank you.

Hello, I ran /apollo/modules/tools/perception/replay_perception.py from docker after starting dreamview. Inside the script you can find the coordinates and adjust them as you like


Thank you for your answer. You mean inside the script, is garage script in the perception/tools or just replay_perception.py? Thank you very much


.Hi, the command should look similar to this:
python replay_perception.sh garage_onroad_vehicle_3.json
You can edit the coordinates and size of the obstacles in the garage_onroad_vehicle_3.json file.
Sorry for the late response.




// How to debug in dreamland #5853

I am using dreamland to verify my motion planning algorithm. My algorithm works well in dream view simulation, but it could not run in dreamland. The result is failed because of "error".
How do I detect what is the error here? it does not give any more information other than that.

thank you


thank you,

I was using a clock timer as the terminate condition for a while loop, that might the reason it stuck in an infinite loop. After I change the terminate condition (not using a clock timer), there is no problem.
My code (with clock timer) works fine in my machine, so the clock function might not work properly in the server. Can you guys please check that?

Thank you for your support.


oh, it is really simple, I want the code to "do something in 50ms"
double start_time_ = Clock::NowInSeconds();
while (Clock::NowInSeconds() - start_time_ < 0.05) {
$do something
}

So I think Clock::NowInSeconds() does not work the same way in the server as in a local machine


I see, so this is expected as in simulation the Clock mode is set as MOCK so that NowInSeconds will always return the same time. I recommend calling system library to get real time, something like:

double RealTimeNow() {
  auto now = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::seconds>(now).count();
}



// Questions on traffic light module #8286

When a traffic light has different shapes as shown above, the traffic light will be recognized purely by the color rather than location. Therefore, yes Apollo can recognize them. however, the probability of recognition will be lower and neighbor traffic lights will boost the confidence.

Thanks for the reply @techoe . I had noticed that all the traffic lights were being focused onto, even though each lane had its own traffic light. So I had the question, Apollo does not choose just one traffic light according to the lane the ego vehicle is in, instead it focuses on all the traffic lights in that direction, does it?

Correct. When self-driving car (SDV) is going thru an intersection, all
traffic lights in the HDmap were detected and recognized. After majority
voting, the final traffic light of the intersection is determined.





// Is the Hybrid A * path optimized by distance approach problem necessarily a collision-free path? #9555


Hello,everyone!
when I tried to run the trajectory_smoother in the open space planner,I found that sometimes Hybrid A * path optimized by distance approach is the collision path,Is this an inherent flaw in the algorithm?
Moreover,What is iterative anchoring smoother designed in the trajectory_smoother used for ？

If you can provide the collision case for us to check, it would be easier to see what is going on. The possible reasons might include, numeric accuracy issue in Ipopt, slack penalty on collision avoidance in crowded area.

Iterative anchoring is another world frame collision avoidance path planning algorithm we are working on



The green line represents Hybrid A* Generated path,the red line represents Optimized path,the Pink line represents obstacle,as you saw,the optimized path overlaps with the obstacle.
Here are some of my parameters:
vehicle parameters:
`vehicle_param {
brand: LINCOLN_MKZ

front_edge_to_center: 1.6
back_edge_to_center: 0.3
left_edge_to_center: 0.65
right_edge_to_center: 0.15

length: 1.9
width: 0.8
height: 1.48
min_turn_radius: 0.8
max_acceleration: 2.0
max_deceleration: -6.0
max_steer_angle: 8.20304748437
max_steer_angle_rate: 8.55211
min_steer_angle_rate: 0
steer_ratio: 16
wheel_base: 1.4
wheel_rolling_radius: 0.335
max_abs_speed_when_stopped: 0.2

brake_deadzone: 15.5
throttle_deadzone: 18.0
}Algorithm parameter：warm_start_config {
xy_grid_resolution: 0.3
phi_grid_resolution: 0.1
next_node_num: 10
step_size: 0.5
traj_forward_penalty: 1.0
traj_back_penalty: 1.0
traj_gear_switch_penalty: 10.0
traj_steer_penalty: 0.0
traj_steer_change_penalty: 0.0
grid_a_star_xy_resolution: 0.5
node_radius: 0.5
s_curve_config {
acc_weight: 1.0
jerk_weight: 0.0
dkappa_penalty_weight: 100.0
ref_s_weight: 0.1
ref_v_weight: 0.0
}
}
dual_variable_warm_start_config {
weight_d: 1.0
ipopt_config {
ipopt_print_level: 0
mumps_mem_percent: 6000
mumps_pivtol: 1e-06
ipopt_max_iter: 100
ipopt_tol: 1e-05
ipopt_acceptable_constr_viol_tol: 0.1
ipopt_min_hessian_perturbation: 1e-12
ipopt_jacobian_regularization_value: 1e-07
ipopt_print_timing_statistics: "yes"
ipopt_alpha_for_y: "min"
ipopt_recalc_y: "yes"
}
qp_format: OSQP
min_safety_distance: 0.0
debug_osqp: false
}
distance_approach_config {
weight_steer: 0.3
weight_a: 1.1
weight_steer_rate: 3.0
weight_a_rate: 2.5
weight_x: 2.3
weight_y: 0.7
weight_phi: 1.5
weight_v: 3.75
weight_steer_stitching: 1.75
weight_a_stitching: 3.25
weight_first_order_time: 4.25
weight_second_order_time: 13.5
weight_end_state: 1.0
min_safety_distance: 0.0
max_speed_forward: 2.0
max_speed_reverse: 1.0
max_acceleration_forward: 2.0
max_acceleration_reverse: 1.0
min_time_sample_scaling: 0.8
max_time_sample_scaling: 1.2
use_fix_time: false
ipopt_config {
ipopt_print_level: 0
mumps_mem_percent: 6000
mumps_pivtol: 1e-06
ipopt_max_iter: 1000
ipopt_tol: 0.0001
ipopt_acceptable_constr_viol_tol: 0.1
ipopt_min_hessian_perturbation: 1e-12
ipopt_jacobian_regularization_value: 1e-07
ipopt_print_timing_statistics: "yes"
ipopt_alpha_for_y: "min"
ipopt_recalc_y: "yes"
ipopt_mu_init: 0.1
}
enable_constraint_check: false
enable_hand_derivative: false
enable_derivative_check: false
enable_initial_final_check: false
distance_approach_mode: DISTANCE_APPROACH_IPOPT
enable_check_initial_state: false
enable_jacobian_ad: true
}`
Can you find out what is wrong?





How did you formulate your obstacle constraints? In distance approach. Every obstacle has to be in convex set form (Ax<b). If not, distance approach would fail.

Each obstacle region is determined by a line segment in my implemention, so you can find there is two obstacles,o1,o2 in the picture below, obstacle vertices are stored clockwise, as the picture shown below,v1,v2 vertices:

Solve the analytical expression of the line segment consisting of v1, v2 vertices ,y=a*x+b,so I can determine the A and b of obstacle.



You should formulate o1 and o2 as one convex obstacle, otherwise your implementations make the upper left space wrt v0 the only feasible configuration set for collision avoidance, which will surely fail distance approach. Plz check out open_space_roi for more details



Thank you for your patience.
According to your instruction,I have the following issue:
1.Can a convex obstacle be determined by a line segment or at least two line segments?
2.Is there a relevant documentation of open_space_roi as mentioned above in apollo?



The success of distance approach trajectory smoothing relies on the correct convexification of obstacle. There are some ways to do that. Plz check the term “Convex Decomposition” for more detail.

You are free to check our implementation of open_space_roi for your own usage of our distance approach trajectory smoothing algorithm.

To answer your previous question. Yes, an obstacle can be be present as one hyperplane or a polygon bounded by a few hyperplanes.



@JackRen88 Also you can modify and verify your algorithm against the open space scenarios we created via http://bce.apollo.auto/, I did not fully understand your drawing in above, but looks like a simple scenario and should be covered in simulation mentioned above.



If you can present the three green line obstacle as three convex sets. Just like how we add perception obstacles in open_space_roi.cc.





// What are concept of 'overtake_obstacle' & 'guard_obstacle' in ChangeLane rule of TrafficDecider #5994

I can't understand the concept of 'overtake_obstacle' & 'guard_obstacle' in ChangeLane rule of TrafficDecider. The logics don't match with their name very much, the codes are confusing.

Appreciated if given some pointers on these.

overtake_obstacle
From the logic, I think the obstacle behind the ADC and beyond the threshold distance should be taken care of as 'overtake_obstacle'. Why need to take of those far behind ADC instead of the nearing obstacles?
How would these 'overtake_obstacle' be taken care of in the planing logic after?
guard_obstacle
From the logic, I think only the obstacles behind ADC and going to run into the front of ADC within a threshold distance should be taken care of as 'guard_obstacle'. Can you describe a little bit of the scenario in term of the behaviour? Why those end up beyond the threshold distance should not be taken care of?


guard and overtake obstacle are vehicles in front of or at the back of the ego vehicle when making a lane change. Please refer to the change_lane.h for more details, e. g. 

 brief This function will extend the prediction of the guard obstacle to
*guard lane change action. Due to the ST path may drive on the forward lane
*first, then slowly move to the target lane when making lane change, we need
*to make sure the vehicle is aware that it actually occupies the target lane,
*even when it is not on the target lane yet.





//  Why the planning module can't be triggered based on a timer under the Cyber RT framework? #6572


Under the Cyber RT framework, the update of the planning module is changed from the timer triggering mechanism to the message updating mechanism. In other words, a specific planner such as NaviPlanner of the planning module only works (via planning_base_->RunOnce (local_view_, &adc_trajectory_pb);) in the case where all three messages prediction::PredictionObstacles, canbus::Chassis and localization::LocalizationEstimate have arrived.
It causes a problem for debugging the planning algorithm. Usually, a recorded data bag file only contains two types of messages: canbus::Chassis and localization::LocalizationEstimate; the prediction::PredictionObstacles messages does not exist in general. If we start the Perception and Prediction modules to generate such a message, the following core dump message will appear:

#0  0x00007f5437d21c37 in __GI_raise (sig=sig@entry=6) at ../nptl/sysdeps/unix/sysv/linux/raise.c:56
#1  0x00007f5437d25028 in __GI_abort () at abort.c:89
#2  0x00007f54398e7284 in google::DumpStackTraceAndExit() () from /usr/local/lib/libglog.so.0
#3  0x00007f54398deec0 in google::LogMessage::Fail() () from /usr/local/lib/libglog.so.0
#4  0x00007f54398dee07 in google::LogMessage::SendToLog() () from /usr/local/lib/libglog.so.0
#5  0x00007f54398de7de in google::LogMessage::Flush() () from /usr/local/lib/libglog.so.0
#6  0x00007f54398e183b in google::LogMessageFatal::~LogMessageFatal() () from /usr/local/lib/libglog.so.0
#7  0x00007f54148c159d in apollo::perception::common::SensorManager::SensorManager (this=0x60bb2d0) at modules/perception/common/sensor_manager/sensor_manager.cc:36
#8  0x00007f542a3ec351 in apollo::perception::common::SensorManager::Instance(bool)::{lambda()#1}::operator()() const (__closure=0x7ffc8003528b) at ./modules/perception/common/sensor_manager/sensor_manager.h:86
#9  0x00007f542a3f1c58 in std::_Bind_simple<apollo::perception::common::SensorManager::Instance(bool)::{lambda()#1} ()>::_M_invoke<>(std::_Index_tuple<>) (this=0x7ffc8003528b)
    at /usr/include/c++/4.8/functional:1732
#10 0x00007f542a3f0515 in std::_Bind_simple<apollo::perception::common::SensorManager::Instance(bool)::{lambda()#1} ()>::operator()() (this=0x7ffc8003528b) at /usr/include/c++/4.8/functional:1720
#11 0x00007f542a3ee6f2 in std::__once_call_impl<std::_Bind_simple<apollo::perception::common::SensorManager::Instance(bool)::{lambda()#1} ()> >() () at /usr/include/c++/4.8/mutex:754
#12 0x00007f54388e1a80 in pthread_once () at ../nptl/sysdeps/unix/sysv/linux/x86_64/pthread_once.S:103
#13 0x00007f542a3e82c2 in __gthread_once (__once=0x7f542a7f4178 <apollo::perception::common::SensorManager::Instance(bool)::flag>, __func=0x7f543837b110 <__once_proxy>)
    at /usr/include/x86_64-linux-gnu/c++/4.8/bits/gthr-default.h:699
#14 0x00007f542a3ed257 in std::call_once<apollo::perception::common::SensorManager::Instance(bool)::{lambda()#1}>(std::once_flag&, apollo::perception::common::SensorManager::Instance(bool)::{lambda()#1}&&) (
    __once=..., 
    __f=<unknown type in /apollo/bazel-bin/modules/perception/onboard/component/../../../../_solib_k8/libmodules_Sperception_Sonboard_Scomponent_Slibperception_Ucomponent_Uinner_Ulidar.so, CU 0x68780e, DIE 0x84c375>) at /usr/include/c++/4.8/mutex:786
#15 0x00007f542a3ec3d6 in apollo::perception::common::SensorManager::Instance (create_if_needed=true) at ./modules/perception/common/sensor_manager/sensor_manager.h:86
#16 0x00007f542a3e8c99 in apollo::perception::onboard::SegmentationComponent::InitAlgorithmPlugin (this=0x5a2a330) at modules/perception/onboard/component/segmentation_component.cc:73
#17 0x00007f542a3e8877 in apollo::perception::onboard::SegmentationComponent::Init (this=0x5a2a330) at modules/perception/onboard/component/segmentation_component.cc:48
#18 0x00007f542a3fef85 in apollo::cyber::Component<apollo::drivers::PointCloud, apollo::cyber::NullType, apollo::cyber::NullType, apollo::cyber::NullType>::Initialize (this=0x5a2a330, config=...)
    at ./cyber/component/component.h:138
#19 0x0000000000413484 in apollo::cyber::mainboard::ModuleController::LoadModule (this=0x7ffc80035920, dag_config=...) at cyber/mainboard/module_controller.cc:99
#20 0x00000000004138b2 in apollo::cyber::mainboard::ModuleController::LoadModule (this=0x7ffc80035920, path="/apollo/modules/perception/production/dag/dag_streaming_perception.dag")
    at cyber/mainboard/module_controller.cc:128
#21 0x0000000000412eca in apollo::cyber::mainboard::ModuleController::LoadAll (this=0x7ffc80035920) at cyber/mainboard/module_controller.cc:64
#22 0x0000000000412ad4 in apollo::cyber::mainboard::ModuleController::Init (this=0x7ffc80035920) at cyber/mainboard/module_controller.cc:33
#23 0x000000000040f462 in main (argc=3, argv=0x7ffc80035ad8) at cyber/mainboard/mainboard.cc:41


We are using data to trigger each module to minimize the end-to-end time delay. You can add a dummy prediction to provide some prediction topic so that the planning can be triggered.







// How to use "planning" module in a closed area? #2732

If it is only repeating a recorded trajectory, there is no need to use EM planner. The current (2.0) version of planner requires to load hdmap in planning.cc.

Maybe a better way to do this is to entirely separate each planner from the beginning. For now, I think you can comment the following code in the Init() function of planning.cc:

hdmap_ = apollo::hdmap::HDMapUtil::BaseMapPtr();
CHECK(hdmap_) << "Failed to load map file:" << apollo::hdmap::BaseMapFile();

and all other code that uses hdmap_.



@startcode Yes, I need it. Thank you. How can I make contact with you? In addition, I find the "PathDecider" class can be used to avoid static obstacles. Can I use it in the "RTKReplayPlanner"? My implementation is listed as follows ( Note that the block with the "davidhopper" tag). But it does not work. Can you give me some advice? Thanks so much.





The avoidance of obstacles are done from both dp_poly_path and path_decider. I think you need both modules to avoid obstacles. The dp path will sample points on a given road, and selected the curve with minimum cost. The path decider will generate a stop decision when certain obstacles are hard to avoid.






// What is the basic idea of the function: DPRoadGraph::SamplePathWaypoints() #3069

 apollo/modules/planning/tasks/optimizers/road_graph/dp_road_graph.cc 


bool DPRoadGraph::SamplePathWaypoints( const common::TrajectoryPoint &init_point,  std::vector<std::vector<common::SLPoint>> *const points) {
  CHECK_NOTNULL(points);

  // 最小采样距离
  const double kMinSampleDistance = 40.0;
  // 总长度 = min(初始点 + max(初始速度 × 8, 最小采样距离), 参考线长度）
  const double total_length = std::fmin( init_sl_point_.s() + std::fmax(init_point.v() * 8.0, kMinSampleDistance), reference_line_.Length());

  // 采样前视时长
  constexpr double kSamplePointLookForwardTime = 4.0;
  // 采样步长 = 初始速度 × 采样前视时长，要求：
  // step_length_min(默认值：8) <= 采样步长 <= step_length_max(默认值：15)
  const double level_distance =  common::math::Clamp(init_point.v() * kSamplePointLookForwardTime,  config_.step_length_min(), config_.step_length_max());
  // 累计轨迹弧长
  double accumulated_s = init_sl_point_.s();
  // 上次轨迹弧长
  double prev_s = accumulated_s;
  // 累计轨迹弧长小于总长度时，将累计轨迹弧长每次加上采样步长，进行循环采样
  for (std::size_t i = 0; accumulated_s < total_length; ++i) {
    accumulated_s += level_distance;
    if (accumulated_s + level_distance / 2.0 > total_length) {
      accumulated_s = total_length;
    }
    // 本次轨迹弧长：取累计轨迹弧长与总长度之间的最小值
    const double s = std::fmin(accumulated_s, total_length);
    // 最小允许采样步长
    constexpr double kMinAllowedSampleStep = 1.0;
    // 若本次轨迹弧长与上次轨迹弧长间的差值小于最小允许采样步长，跳过本次采样
    if (std::fabs(s - prev_s) < kMinAllowedSampleStep) {
      continue;
    }
    prev_s = s;

    // 左车道宽
    double left_width = 0.0;
    // 右车道宽
    double right_width = 0.0;
    reference_line_.GetLaneWidth(s, &left_width, &right_width);

    // 边界缓冲
    constexpr double kBoundaryBuff = 0.20;
    const auto &vehicle_config = common::VehicleConfigHelper::instance()->GetConfig();
    const double half_adc_width = vehicle_config.vehicle_param().width() / 2.0;
    // 右侧允许宽度 = 右车道宽 - 半车宽 - 边界缓冲
    const double eff_right_width = right_width - half_adc_width - kBoundaryBuff;
    // 左侧允许宽度 = 左车道宽 - 半车宽 - 边界缓冲
    const double eff_left_width = left_width - half_adc_width - kBoundaryBuff;
    // 每步采样点数
    const size_t num_sample_per_level =  FLAGS_use_navigation_mode ? config_.navigator_sample_num_each_level() : config_.sample_points_num_each_level();
    // 默认横向采样间隔
    double kDefaultUnitL = 1.2 / (num_sample_per_level - 1);
    if (reference_line_info_.IsChangeLanePath() && !IsSafeForLaneChange()) {
      kDefaultUnitL = 1.0;
    }
    // 横向采样距离
    const double sample_l_range = kDefaultUnitL * (num_sample_per_level - 1);
    // 右采样边界（车辆右侧为负值）
    double sample_right_boundary = -eff_right_width;
    // 左采样边界（车辆左侧为正值）
    double sample_left_boundary = eff_left_width;
    // 参考线正在改变车道时
    if (reference_line_info_.IsChangeLanePath()) {
      // 右采样边界取右采样边界与初始点横向偏移之间的最小值
      sample_right_boundary = std::fmin(-eff_right_width, init_sl_point_.l());
      // 左采样边界取左采样边界与初始点横向偏移之间的最大值
      sample_left_boundary = std::fmax(eff_left_width, init_sl_point_.l());

      // 若初始点横向偏移 > 左侧允许宽度，则将右侧采样边界设置为右侧采样边界与（初始点横向偏移 - 横向采样距离）之间的最大值
      if (init_sl_point_.l() > eff_left_width) {
        sample_right_boundary = std::fmax(sample_right_boundary,  init_sl_point_.l() - sample_l_range);
      }
      // 若初始点横向偏移 < 右侧允许宽度，则将左侧采样边界设置为左侧采样边界与（初始点横向偏移 + 横向采样距离）之间的最小值
      if (init_sl_point_.l() < eff_right_width) {
        sample_left_boundary = std::fmin(sample_left_boundary,  init_sl_point_.l() + sample_l_range);
      }
    }

    // 横向采样距离数组
    std::vector<double> sample_l;
    // 参考线正在改变车道且改变车道不安全时，将当前参考线到其他参考线的偏移值存储到横向采样距离数组
    if (reference_line_info_.IsChangeLanePath() && !IsSafeForLaneChange()) {
      sample_l.push_back(reference_line_info_.OffsetToOtherReferenceLine());
    } else {
      // 其他情形，从右采样边界到左采样边界，按照每步采样点数进行均匀采样，并将结果存储到横向采样距离数组
      common::util::uniform_slice(sample_right_boundary, sample_left_boundary,
                                  num_sample_per_level - 1, &sample_l);
      if (HasSidepass()) {
        // currently only left nudge is supported. Need road hard boundary for
        // both sides
        sample_l.clear();
        switch (sidepass_.type()) {
          case ObjectSidePass::LEFT: {
            // 左侧绕行：将（左侧允许宽度 + 左侧绕行距离）存储到横向采样距离数组
            sample_l.push_back(eff_left_width + config_.sidepass_distance());
            break;
          }
          case ObjectSidePass::RIGHT: {
            // 右侧绕行：将-（右侧允许宽度 + 右侧绕行距离）存储到横向采样距离数组
            sample_l.push_back(-eff_right_width - config_.sidepass_distance());
            break;
          }
          default:
            break;
        }
      }
    }
    // 本次采样点数组
    std::vector<common::SLPoint> level_points;
    planning_internal::SampleLayerDebug sample_layer_debug;
    for (size_t j = 0; j < sample_l.size(); ++j) {
      // 横向偏移值
      const double l = sample_l[j];
      constexpr double kResonateDistance = 1e-3;
      common::SLPoint sl;
      // 若为奇数采样点或者（总长度 - 累计轨迹弧长）几乎为0即已抵达采样终点，
      // 则直接将当前采样点坐标设置为(s, l)
      if (j % 2 == 0 ||
          total_length - accumulated_s < 2.0 * kResonateDistance) {
        sl = common::util::MakeSLPoint(s, l);
      } else {
        // 其他情形，将当前采样点坐标设置为（min(总长度，s+误差)，l）
        sl = common::util::MakeSLPoint(
            std::fmin(total_length, s + kResonateDistance), l);
      }
      sample_layer_debug.add_sl_point()->CopyFrom(sl);
      // 将当前采样点坐标存储到本次采样点数组
      level_points.push_back(std::move(sl));
    }
    // 若参考线未改变车道且不绕行，则将横向偏移值为0的采样点（即沿参考线方向的采样点）也加入本次采样点数组
    if (!reference_line_info_.IsChangeLanePath() && !HasSidepass()) {
      auto sl_zero = common::util::MakeSLPoint(s, 0.0);
      sample_layer_debug.add_sl_point()->CopyFrom(sl_zero);
      level_points.push_back(sl_zero);
    }

    if (!level_points.empty()) {
      planning_debug_->mutable_planning_data()
          ->mutable_dp_poly_graph()
          ->add_sample_layer()
          ->CopyFrom(sample_layer_debug);
      // 将本次的所有采样点存储到总采样点数组
      points->emplace_back(level_points);
    }
  }
  return true;
}



@lianglia-apollo I have three questions according to the following code snippet. First, what is the meaning of the variable of "kResonateDistance"? Second, Why the "s" value is different at the same sampling level? I think all "s" values should be same but "l" values different. Third, from what I understand, "s + kResonateDistance" is almost the same as "s". What is the necessity of this line: sl = common::util::MakeSLPoint(std::fmin(total_length, s + kResonateDistance), l)? What I mean is that we can use sl = common::util::MakeSLPoint(s, l); to replace if (j % 2 == 0 || total_length - accumulated_s < 2.0 * kResonateDistance) { sl = common::util::MakeSLPoint(s, l); } else { sl = common::util::MakeSLPoint(std::fmin(total_length, s + kResonateDistance), l);} .

We made s slightly different to avoid discontinuity issues at lane boundaries.


for the second question：there is a for loop

  for (std::size_t i = 0; accumulated_s < total_length; ++i) {}

So,it first slice s,then slice l.
for the third question.the type of the s and l is double,MakeSLPoint could generate a proto :SLPoint
it could be found in modules/common/utils/utils.cc





// Question: re-create reference_line_provider_ #4873

HI Apollo Team,

我在學習Planning Source Code 中有個疑問,
關於eference_line_provider_ 在Init() 產生後, 每一段時間會在RunOnce()中重新產生新的reference_line_provider_

我的疑問是在Start()中, 對reference_line_provider_ ->Start()呼叫了這個Function 產生Thread出來工作
但又在之後的RunOnce()中產生新的reference_line_provider_, 如此舊的reference_line_provider_就被release了, 而新的reference_line_provider_也沒呼叫Start()

這樣的程序流程是正確的嗎?

謝謝


Status Planning::Start() {
timer_ = AdapterManager::CreateTimer(
ros::Duration(1.0 / FLAGS_planning_loop_rate), &Planning::OnTimer, this);
// The "reference_line_provider_" may not be created yet in navigation mode.
// It is necessary to check its existence.
if (reference_line_provider_) {
reference_line_provider_->Start();
}
start_time_ = Clock::NowInSeconds();
AINFO << "Planning started";
return Status::OK();
}

创建定时器，之后调用RunOnce做相关planning规划工作，包括reference_line_provider_赋值等
如果reference_line_provider_ 不为null，启动reference_line_provider_ -Start()，理解为伴随planning进程的整个生命周期，获取启动参考线信息的获取



HI @douqian

整個流程為 Planning::Init() -> Planning::Start() -> spin()
所以整個Planning 的生命週期中
(A)reference_line_provider_->Start() 只會執行一次並產生ReferenceLineProvider::GenerateThread

我的困惑就是在於OnTimer() 中的RunOnce()內會一直重新產生新的(B)reference_line_provider_並呼叫(A)reference_line_provider_ 的destructor, 如此(A)reference_line_provider_ 的GenerateThread消失了
而(B)reference_line_provider_也沒機會產生GenerateThread

在學習的過程中對這段的不解這段設計的核心意義, 因為GenerateThread只存在非常短暫的時間

謝謝



RunOnce（）中，在navigation mode的情况下会reference_line_provider_ = std::make_unique(hdmap_);， 非navigation mode在init()中创建。
navigation mode会实时生成基于车身坐标系的地图（使用以车辆当前位置为原点的相对坐标系）供planning使用。

if (FLAGS_use_navigation_mode) {
// recreate reference line provider in every cycle
hdmap_ = HDMapUtil::BaseMapPtr();
// Prefer "std::make_unique" to direct use of "new".
// Reference "https://herbsutter.com/gotw/_102/" for details.
reference_line_provider_ = std::make_unique(hdmap_);
}






// Initialization of kernel_* matrices in spline_seg_kernel.cc #4268
/modules/planning/math/smoothing_spline/spline_seg_kernel.cc








// Frame::CreateDestinationObstacle() is called in every loop once the ego_vehicle is on the last route segment #4082

/modules/planning/common/frame.cc

In bool Frame::CreateReferenceLineInfo(), Frame::CreateDestinationObstacle() is called in every loop once the ego_vehicle is on the last route segment as the following:

bool Frame::CreateReferenceLineInfo() {
...

  reference_line_info_.clear();
  bool near_destination = false;
  auto ref_line_iter = reference_lines.begin();
  auto segments_iter = segments.begin();
  while (ref_line_iter != reference_lines.end()) {
    if (segments_iter->StopForDestination()) {
      near_destination = true;
    }
    reference_line_info_.emplace_back(vehicle_state_, planning_start_point_,
                                      *ref_line_iter, *segments_iter);
    ++ref_line_iter;
    ++segments_iter;
  }
  if (near_destination && CreateDestinationObstacle() < 0) {
    AERROR << "Failed to create the destination obstacle";
    return false;
  }
...
}
In CreateDestinationObstacle(), it checks the newest routing response and creates a 2d box to represent a virtual obstacle object.

int Frame::CreateDestinationObstacle() {
  const auto &routing =
      AdapterManager::GetRoutingResponse()->GetLatestObserved();
  if (routing.routing_request().waypoint_size() < 2) {
    ADEBUG << "routing_request has no end";
    return -1;
  }
  const auto &routing_end = *routing.routing_request().waypoint().rbegin();
  const auto lane = hdmap_->GetLaneById(hdmap::MakeMapId(routing_end.id()));
  if (!lane) {
    AERROR << "Failed to find lane for destination : "
           << routing_end.ShortDebugString();
    return -2;
  }

  double dest_lane_s =
      std::max(0.0, routing_end.s() - FLAGS_virtual_stop_wall_length -
                        FLAGS_stop_distance_destination);
  auto dest_point = lane->GetSmoothPoint(dest_lane_s);
  double left_width = 0.0;
  double right_width = 0.0;
  lane->GetWidth(dest_lane_s, &left_width, &right_width);
  // check if destination point is in planning range
  Box2d destination_box{{dest_point.x(), dest_point.y()},
                        lane->Heading(dest_lane_s),
                        FLAGS_virtual_stop_wall_length,
                        left_width + right_width};
  // add destination's projection to each reference line info
  AddStaticVirtualObstacle(FLAGS_destination_obstacle_id, destination_box);
  return 0;
}
If the destination is not changed, there is no need to create this virtual object in every loop. In fact, the newly created object is not going to be added to the static virtual obstacle list anyway (AddStaticVirtualObstacle()) if it is already existed with the same id. It is better to remember what the current destination is and update the virtual obstacle only when it is changed.


The code snippet was from r.2.0.0 branch. It seems the current master branch behaviors pretty much the same.

It is fine if by design. However, the final destination is not something changing all time (very different from those on-route dynamic objects). It can save some computation cycles to create the axis-aligned bounding box every frame for this virtual obstacle.








// Traffic light right turn unprotected scenario triggered safety mode #9143

We have already checked the planning log and mainborad log, there is no error. We have no idea why this scenario will trigger safety mode. Could you please tell us how can we find the error that triggered the safety mode?

Is Apollo3.5 supporting the traffic light unprotected right turn scenario or this scenario has fixed in Apollo5.0?

@louis016 this happens where there's some error from planning.
code:
https://github.com/ApolloAuto/apollo/blob/master/modules/monitor/software/functional_safety_monitor.cc
line 34-42.

We don't have enough context on what is going on in your case. It would be much easier if you just debug by yourself. Let us know if you need further support.





// cyber_recorder has no 'echo' command #9523

yeah. work with cyber started from 3.5
you can try source /apollo/cyber/setup.bash






// /ApolloAuto/apollo/tree/master/modules/tools

Apollo Tools
Prerequisites
The tools are mostly written in Python and relying on compiled proto modules. So generally you need to do the following steps to make it function well.

Note that all scripts in this page are referenced from Apollo root directory.

# Compile everything including python proto libs.
apollo.sh build

# Setup PYTHONPATH properly.
source scripts/apollo_base.sh





// bug in demo_2.5.bag #8100

Describe the bug
The position of obstacle should be referred to world frame according to the proto definition. But the /apollo/perception/obstacles messages in demo_2.5.bag have really small values for positions. So I guess they were referred to vehicle frame instead. Is that a bug?

To Reproduce
Steps to reproduce the behavior:
rosbag play demo_2.5.bag
rostopic echo /apollo/perception/obstacles

Expected behavior
positions are really small values


Obstacles are referred to vehicle frame for Navi mode of Apollo 2.5.








// Control module couldn't follow steering command in vallet parking #8161

System information
OS Platform and Distribution (e.g., Linux Ubuntu 14.04): Linux Ubuntu 14.04
Apollo installed from (source or binary): Source
Apollo version (1.0, 1.5, 2.0, 2.5, 3.0): 3.5
Commit number: 0075579
Using LGSVL Simulator.
Steps to reproduce the issue:
1.Made parking space on hdmap.
2.Checked parking operation worked with Simcontrol on dreamview.
3.Ran Localization, Perception, Planning, Prediction, Routing, Traffic Light, Transform.
4.The vehicle stops near to the parking space.
5.The planning modules started vallet parking and made trajectory.

Issue: The control module couldn't follow the target steering command. Steering target is small number. Then the controller doesn't make car moving. Sometimes, the car is moving but steering is applied too much. It does not stop at the moment when the vehicle is supposed to stop steering. How can I make it working? Any suggestion?


Lol, the control-in-loop is not available yet, so what you saw via Dreamview is under "perfect control", a.k.a planning trajectory directly re-interpreted as chassis & localization information.



There is a map, provided by Apollo, which has parking space. The map name is sunnyvale_with_two_offices. In the map directory, there is default routing file defining point of interest. Its file name is default_end_way_point.txt. You can refer to this file and notice how this file defines waypoints to trigger parking scenario.

You can do parking on sim control on dreamview.

When you test parking scenario, follow these steps.
1.Run dreamview.
2.In Tasks tab, turn sim control on.
3.In Module Controller, run Routing and Planning modules.
4.In Default Routing, you can choose one of parking waypoints.
Then you can see the vehicle doing parking.

I'm working on vehicle parking with LGSVL simulator. In apollo 3.5, Planning module doesn't seem to be done with vallet parking.




// LQR solver in lateral controller cannot converge to a solution #8290

The control log always warning linear_quadratic_regulator.cc:56] LQR solver cannot converge to a solution, last consecutive result diff. is:0.998318

Any suggestion for this?

@xmyqsh need to set config parameters of the vehicle, kinematic model(bicycle model, here) and dynamics model, as well as some calibration.




//  What is the meaning of `the car is pointing North` ? #8352

// euler_angles.yaw() is zero when the car is pointing North, but
// the heading is zero when the car is pointing East.
return NormalizeAngle(euler_angles.yaw() + M_PI_2);

What's the meaning of that the car is pointing North?
Can I interpret the comment above as
// euler_angles.yaw() is zero when the right side of the car is pointing North, but
// the heading (the front side) of the car is zero when the right side of the car is pointing East.

And the definition of yaw should be:

--------------N(0)-------------------

W(90)--------------------E(-90)-----

-------------S(180)------------------

That's to say when the euler_angles.yaw() is zero, the car should be moving to the West, right?


@xmyqsh Not that.
The definition of heading in apollo is the angle to the x-axis in vehicle coordinate, while the coordinate of vehicle is RFU.




//Question about Component::Proc function #8491

How can i trigger the Proc function, such as Localization component ?

Hi, @wanggujin you can read this example, the CommonComponentSample component subscribes message from channel /apollo/prediction，which is in common.dag. When the message arrives，it will trigger the proc func.
https://github.com/ApolloAuto/apollo/tree/master/cyber/examples/common_component_example







// computation in component is much lower #7795

We are facing a problem that, computation is much lower if executed in a CyberRT component.

We just write an image undistortion program using opencv (cpu mode) in docker. The computation takes less than 10ms only. However, when translating to a component, the undistortion process takes more than 100ms. We put the computation in the Proc function of the component, and use another component to send message only. Then, we config the scheduler of this component, results not change. Only the mentioned two components are launched in docker.

Addition, we change the processor_num in the scheduler_conf file, but changes not occure.

So, what is the problem? and why the scheduler conf is not work?


I think maybe these are two problems. First，the conf file corresponds to the process name, the scheduler conf is not work, I guess it may not be the conf file for your process. If the processor does not have the corresponds conf, it will use the default_proc_num in cyber.pb.conf, normally there's nothing wrong here. Then, can you show the detail of your case ? Otherwise, it's not good to locate the problem


@zhangchaoltt Thanks. The test codes are as follows:

TestComponent.cc [dir: apollo/modules/test/]
bool TestComponent::Proc(const std::shared_ptr& msg){
double start_time = cyber::Time::Now().ToSecond();
undistort_one_image();
std::cout << "diff time: " << cyber::Time::Now().ToSecond() - start_time<< std::endl;
return true;
}

dag file [dir: apollo/modules/test/]
module_config {
module_library : "/apollo/bazel-bin/modules/test/libtest_component.so"
components {
class_name : "TestComponent"
config {
name : "undist_img"
readers {
channel: "/apollo/test"
}
}
}
}

launch file [dir: apollo/modules/test/]


test
<dag_conf>/apollo/modules/test/test.dag</dag_conf>
<process_name>test_undist</process_name>

scheduler conf file [dir: apollo/cyber/conf/test_undist.conf]
scheduler_conf {
policy: "classic"
classic_conf {
groups: [
{
name: "undistortion_test"
processor_num: 8
affinity: "range"
cpuset: "0-7"
processor_policy: "SCHED_OTHER"
processor_prio: 0
tasks: [
{
name: "undist_img"
prio: 0
}
]
}
]
}
}



You can use cyber::Time::Now().ToNanosecond() instead of cyber::Time::Now().ToSecond() to calculate time-consuming


@zhangchaoltt I update cyberRT to the latest version, and build cyber using your command. it is ok now, I change the routine_num from 100 to 10, and cyber_launch my test component, time consuming reaches 7ms. Thanks for your reply.




// Planning trajectory moves away from car when a car stops or moves slowly #6121

https://www.youtube.com/watch?v=QMSlQw5owi0&feature=youtu.be

During testing on our car we faced with situation when a lateral error is increased without any reasons.
I have investigated it and found that when a car velocity is under 5-7 km/h the start point of planning trajectory moves ahead from car. When a different between start point of planning trajectory and car position becomes more than 5 meters, the planning module recalculates the planning trajectory from car position.

You can see described situation on this video. At 5 sec the start point of planning trajectory jumps on car position and lateral error jumps to about zero.

As I understand the start point of planning trajectory is shifting too fast.

Are there any parameters to solve this issue?

System information
Linux Ubuntu 14.04 :
Apollo installed from source (branch r3.0.0):
Apollo version 3.0:


You can look into trajectory_stitcher.cc for this issue.



@lianglia-apollo Thank you!

Finally, I have solved the issue by adding new condition into ComputeStitchingTrajectory for replanning:

  if (matched_point.v() > vehicle_state.linear_velocity() + 0.5)
  {
    ADEBUG << "Current velocity smaller than matched point velocity. Replan is triggered. matched_point.v: " << matched_point.v()  
      << "  vehicle_state.linear_velocity(): " <<  vehicle_state.linear_velocity();
    return ComputeReinitStitchingTrajectory(vehicle_state);
  }


Could you tell me please, what is "matched_point"? How can I get it?
Thank you!



@JJRedmond
It seems I have a little bit another version of trajectory_stitcher.
In r3.0.0 branch matched_point calculates here
https://github.com/ApolloAuto/apollo/blob/r3.0.0/modules/planning/common/trajectory/trajectory_stitcher.cc#L140




// The use of adc_sl_boundary #7154

I have a question about the use of const SLBoundary& AdcSlBoundary() const; and const SLBoundary& VehicleSlBoundary() const which both in the reference_line_info.h.

In my opinion, maybe we should use VehicleSlBoundary() which mean the real time location more often, but not the AdcSlBoundary() which means the planning start point. such as obstacle_blocking_analyzer.cc

@YaoQii they serve different purposes as you already described clearly. :D






// Does relative map need Lidar? #6644

System information

-OS Platform and Distribution: Linux Ubuntu 14.04

Apollo installed from: source
Apollo version: 3.0
We want to use relative map for we do not have HDMap.
When we make our own relative map, does the hardware LIDAR is necessary?
What hardware we need to set up on the ADC for rekative map?

Hi @YafeiWangAlice you not need a Lidar to create the relative map. Thanks!

What else hardware is necessary for produce relative map? Camera? GPS?
Thanks~~



@YafeiWangAlice you need Cameras and GPS as you mentioned.

The GPS+IMU which Apollo suggested is too expensive (over 200,000RMB) for me.
As I can't afford that and I don't need the precision of GPS is that high, can I use a cheaper GPS+IMU instead to produce and use relative map?

That is OK. You may need to modify the code that parses the data in the modules / drivers / gnss directory




// Piecewise jerk path optimizer failed when a vehicle is moving through a sharp turn #9684

Hi, ApolloAuto community!
Can you help me and tell how can I fix this problem?

E0915 piecewise_jerk_path_optimizer.cc:205] piecewise jerk path optimizer failed
E0915 piecewise_jerk_problem.cc:104] failed optimization status:  primal infeasible
E0915 path_optimizer.cc:41] Reference Line 1_0 is not drivable after PIECEWISE_JERK_PATH_OPTIMIZER

It happened when vehicle try to execute turn like at the picture. And after the last error it does not move.

System information
Linux Ubuntu 16.04:
Apollo installed from source:
Apollo version 5.0:

The primal infeasible means the optimizer proved that it is impossible to find a solution for that problem. Could you please make sure,

If it is feasible for your vehicle to make that turn (or say if the turning is too sharp to follow)?
Or did you ever tried to change the vehicle params under modules/common/data/vehicle_param.pb.txt if you are using your own vehicle with more flexible control?

@HongyiSun, thanks for your reply, it is very helpful.

I used "Sim control" mode with the MKZ vehicle model and it seems this turn is not feasible.
When parameters was changed under modules/calibration/data/vehicle_param.pb.txt, ADC passed this turn.











// Is there sensor data in demo_3.5.record? #8531

When I use cyber_record command to replay demo_3.5.record, cyber_monitor shows no sensor data created. The FrameRatio of /apollo/sensor/camera/front_6mm, /apollo/sensor/lidar16/compensator/PointCloud2 and some other channels are 0.

So when I launched perception module separately, no obstacle data were published because of the missing input sensor data. Where can I find the sensor data?

System information
OS Platform and Distribution: Ubuntu16.04
Apollo installed from (source or binary): source
Apollo version: 3.5
Steps to reproduce the issue:
build apollo and launch dreamview
cyber_record play -f demo_3.5.record -l
cyber_monitor


@yfzm you are correct, the demo record does not contain any sensor data.





@natashadsouza Thank you for your reply.

I found sensor data at data open platform. Is there any tutorials about how to use it in apollo 3.5?  ///http://data.apollo.auto/?locale=en-us&lang=en


PS: I have tried to use some data set of the platform:

Vehicle System Demo Data: it still does not contains any camera data.
Calibrate Demo Data: when I use rosbag_to_record to convert the format, it reports Error: the input file is not a ros bag file.


You can try conversion tool "rosbag_to_record" from document below.
https://github.com/ApolloAuto/apollo/blob/master/docs/cyber/CyberRT_Developer_Tools.md






// 

/modules/planning/on_lane_planning.cc

When I study on_lane_planning.cc, I found a minor code scequence issue. In RunOnce() at about line 230 (shown below)

  if (util::IsDifferentRouting(last_routing_, *local_view_.routing)) {
      last_routing_ = *local_view_.routing;
      PlanningContext::Instance()->mutable_planning_status()->Clear();
      reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);
  }

  // Update reference line provider and reset pull over if necessary
  reference_line_provider_->UpdateVehicleState(vehicle_state);
the scequence is:

reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);
reference_line_provider_->UpdateVehicleState(vehicle_state);


modules/planning/reference_line/reference_line_provider.cc
in the function of reference_line_provider_->UpdateRoutingResponse(*local_view_.routing); we can see

bool ReferenceLineProvider::UpdateRoutingResponse(const routing::RoutingResponse &routing) {
  std::lock_guard<std::mutex> routing_lock(routing_mutex_);
  routing_ = routing;
  has_routing_ = true;
  return true;
}


/modules/planning/on_lane_planning.cc
And there is another thread called "GenerateThread" inside reference_line_provider, which use has_routing_ to adjudge to do things

void ReferenceLineProvider::GenerateThread() {
  while (!is_stop_) {
    constexpr int32_t kSleepTime = 50;  // milliseconds
    cyber::SleepFor(std::chrono::milliseconds(kSleepTime));
    const double start_time = Clock::NowInSeconds();
    if (!has_routing_) {
      AERROR << "Routing is not ready.";
      continue;
    }
    std::list<ReferenceLine> reference_lines;
    std::list<hdmap::RouteSegments> segments;
    if (!CreateReferenceLine(&reference_lines, &segments)) {
      AERROR << "Fail to get reference line";
      continue;
    }
    UpdateReferenceLine(reference_lines, segments);
    const double end_time = Clock::NowInSeconds();
    std::lock_guard<std::mutex> lock(reference_lines_mutex_);
    last_calculation_time_ = end_time - start_time;
  }
}


we can see, if has_routing_ is true, it will call CreateReferenceLine() function, which shown below


modules/planning/reference_line/reference_line_provider.cc

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
    std::lock_guard<std::mutex> lock(pnc_map_mutex_);
    if (pnc_map_->IsNewRouting(routing)) {
      is_new_routing = true;
      if (!pnc_map_->UpdateRoutingResponse(routing)) {
        AERROR << "Failed to update routing in pnc map";
        return false;
      }
    }
  }

  if (!CreateRouteSegments(vehicle_state, segments)) {  // use vehicle_state, and call CreateRouteSegments(vehicle_state, segments)  // So if reference_line_provider_->UpdateRoutingResponse(*local_view_.routing) is called, the CreateRouteSegments(vehicle_state, segments) function in GenerateThread will be called
    AERROR << "Failed to create reference line from routing";
    return false;
  }
  if (is_new_routing || !FLAGS_enable_reference_line_stitching) {
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!SmoothRouteSegment(*iter, &reference_lines->back())) {
        AERROR << "Failed to create reference line from route segments";
        reference_lines->pop_back();
        iter = segments->erase(iter);
      } else {
        Vec2d vec2d(vehicle_state.x(), vehicle_state.y());
        common::SLPoint sl;
        if (!reference_lines->back().XYToSL(vec2d, &sl)) {
          AWARN << "Failed to project point: " << vec2d.DebugString()
                << " to stitched reference line";
        }
        Shrink(sl, &reference_lines->back(), &(*iter));
        ++iter;
      }
    }
    return true;
  } else {  // stitching reference line
    for (auto iter = segments->begin(); iter != segments->end();) {
      reference_lines->emplace_back();
      if (!ExtendReferenceLine(vehicle_state, &(*iter),
                               &reference_lines->back())) {
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



we can see it begins to use vehicle_state, and call CreateRouteSegments(vehicle_state, segments).
So if reference_line_provider_->UpdateRoutingResponse(*local_view_.routing) is called, the CreateRouteSegments(vehicle_state, segments) function in GenerateThread will be called

Is it possible of the following code scequence be happened?

reference_line_provider_->UpdateRoutingResponse(*local_view_.routing) is called
CreateRouteSegments(vehicle_state, segments) is called in GenerateThread
reference_line_provider_->UpdateVehicleState(vehicle_state);
If it happened, the vehicle_state in 2nd step maybe old or wrong data, because the newest vehicle_state data is updated at 3rd step

So I think the right code scequence in on_lane_planning.cc is below

  // Update reference line provider and reset pull over if necessary
  reference_line_provider_->UpdateVehicleState(vehicle_state);

  if (util::IsDifferentRouting(last_routing_, *local_view_.routing)) {
      last_routing_ = *local_view_.routing;
      PlanningContext::Instance()->mutable_planning_status()->Clear();
      reference_line_provider_->UpdateRoutingResponse(*local_view_.routing);
  }
just for your reference, thank you :-)





@s894330 all the attributes of vehicle_state is independent of routing or planning or  reference line.
https://github.com/ApolloAuto/apollo/blob/master/modules/common/vehicle_state/proto/vehicle_state.proto
reference line is generated based on routing. we just want to make sure vehicle_state gets updated when we generate reference line. The sequence (update vehicle_state before or after new routing received) does not really matters here.




// non-consistent static obstacle projection on hdmap::path across two consective frames causes planning node crash #5878

 apollo/modules/planning/reference_line/reference_line.cc 
 modules/map/pnc_map/path.cc

ReferenceLine::GetSLBoundary() --> ReferenceLine::XYToSL() -> map_path_.GetProjection()


To reproduce the issue is a bit tricky. But once it is reproduced, it can be consistently reproduced.
In our setup, there is only one DESTINATION static obstacle. It is represented as a 2d box internally. The 4 corners of the box is projected onto the reference line in each frame to verify whether it is within the range. Here is the calling sequences:
ReferenceLine::GetSLBoundary() --> ReferenceLine::XYToSL() -> map_path_.GetProjection()



The frame in which projections are still fine:

W1016 13:02:57.962399    15 reference_line.cc:478] corner: 440752.7832 , 5018675.203
W1016 13:02:57.963398    15 path.cc:615] path::getProjection s: -35.353 l:171.136 min_index: 0 num_segments: 629 total length: 179.883
W1016 13:02:57.963398    15 reference_line.cc:478] corner: 440748.4133 , 5018672.773
W1016 13:02:57.963398    15 path.cc:615] path::getProjection s: -31.8085 l:167.61 min_index: 0 num_segments: 629 total length: 179.883
W1016 13:02:57.963398    15 reference_line.cc:478] corner: 440748.4619 , 5018672.686
W1016 13:02:57.964398    15 path.cc:615] path::getProjection s: -31.7379 l:167.681 min_index: 0 num_segments: 629 total length: 179.883
W1016 13:02:57.964398    15 reference_line.cc:478] corner: 440752.8318 , 5018675.116
W1016 13:02:57.965399    15 path.cc:615] path::getProjection s: -35.2825 l:171.207 min_index: 0 num_segments: 629 total length: 179.883

The 's' projection for 4 corners are (-35.353 , -31.8085, -31.7379, -35.2825).
....

The last frame in which projections are unexpected:

W1016 13:02:58.063395    15 reference_line.cc:478] corner: 440752.7832 , 5018675.203
W1016 13:02:58.063395    15 path.cc:615] path::getProjection s: -35.6943 l:171.135 min_index: 0 num_segments: 629 total length: 179.662
W1016 13:02:58.063395    15 reference_line.cc:478] corner: 440748.4133 , 5018672.773
W1016 13:02:58.064394    15 path.cc:615] path::getProjection s: -32.1497 l:167.609 min_index: 0 num_segments: 629 total length: 179.662
W1016 13:02:58.064394    15 reference_line.cc:478] corner: 440748.4619 , 5018672.686
W1016 13:02:58.065394    15 path.cc:615] path::getProjection s: 173.734 l:170.708 min_index: 574 num_segments: 629 total length: 179.662
W1016 13:02:58.065394    15 reference_line.cc:478] corner: 440752.8318 , 5018675.116
W1016 13:02:58.065394    15 path.cc:615] path::getProjection s: -35.6238 l:171.206 min_index: 0 num_segments: 629 total length: 179.662

The 's' projection for 4 corners are (-35.6943 , -32.1497, 173.734,  -35.6238).
Specially, the 's' projection of the third corner ( 440748.4619 , 5018672.686) is now 173.734 which has a big diff from other three corner. In turn, it causes the following crash at FrenetFramePath::EvaluateByS().
...
W1016 13:02:58.073395    15 path_decider.cc:126] obstacle start s: -35.6943 Id: DEST
W1016 13:02:58.073395    15 frenet_frame_path.cc:64] s: -35.6943 back_s: 67.7514 front_s: 29.7514
F1016 13:02:58.073395    15 frenet_frame_path.cc:67] Check failed: s < points_.back().s() + 1.0e-6 && s > points_.front().s() - 1.0e-6




The l value are very large (~170). Are you trying to project obstacles near to the reference line?


@lianglia-apollo Thanks for the quick reply. I did not modify any logic of the code. You are right. The 'l' value are very large which mean the DEST obstacle is far away from the reference line.




modules/planning/common/reference_line_info.cc

PathObstacle* ReferenceLineInfo::AddObstacle(const Obstacle* obstacle) {
  if (!obstacle) {
    AERROR << "The provided obstacle is empty";
    return nullptr;
  }
  auto* path_obstacle = path_decision_.AddPathObstacle(PathObstacle(obstacle));
  if (!path_obstacle) {
    AERROR << "failed to add obstacle " << obstacle->Id();
    return nullptr;
  }

  SLBoundary perception_sl;
  if (!reference_line_.GetSLBoundary(obstacle->PerceptionBoundingBox(), &perception_sl)) {  // The obstacle's SL boundary is computed by calling ReferenceLine::GetSLBoundary(). The 'l' value is not checked for lateralDecision at this stage. 
    AERROR << "Failed to get sl boundary for obstacle: " << obstacle->Id();
    return path_obstacle;
  }
  path_obstacle->SetPerceptionSlBoundary(perception_sl);

  if (IsUnrelaventObstacle(path_obstacle)) {
    ObjectDecisionType ignore;
    ignore.mutable_ignore();
    path_decision_.AddLateralDecision("reference_line_filter", obstacle->Id(),
                                      ignore);
    path_decision_.AddLongitudinalDecision("reference_line_filter",
                                           obstacle->Id(), ignore);
    ADEBUG << "NO build reference line st boundary. id:" << obstacle->Id();
  } else {
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




The obstacle has SL boundary is computed by calling ReferenceLine::GetSLBoundary(). The 'l' value is not checked for lateralDecision at this stage. Then later on, in the following function:



modules/planning/tasks/path_decider/path_decider.cc

bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  const auto &frenet_path = path_data.frenet_frame_path();
  const auto &frenet_points = frenet_path.points();
  if (frenet_points.empty()) {
    AERROR << "Path is empty.";
    return false;
  }

  const double half_width =
      common::VehicleConfigHelper::GetConfig().vehicle_param().width() / 2.0;

  const double lateral_radius = half_width + FLAGS_lateral_ignore_buffer;

  const double lateral_stop_radius =
      half_width + FLAGS_static_decision_nudge_l_buffer;

  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto &obstacle = *path_obstacle->obstacle();
    bool is_bycycle_or_pedestrain =
        (obstacle.Perception().type() ==
             perception::PerceptionObstacle::BICYCLE ||
         obstacle.Perception().type() ==
             perception::PerceptionObstacle::PEDESTRIAN);

    if (!is_bycycle_or_pedestrain && !obstacle.IsStatic()) {
      continue;
    }

    if (path_obstacle->HasLongitudinalDecision() &&
        path_obstacle->LongitudinalDecision().has_ignore() &&
        path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_ignore()) {
      continue;
    }
    if (path_obstacle->HasLongitudinalDecision() &&
        path_obstacle->LongitudinalDecision().has_stop()) {
      // STOP decision
      continue;
    }
    if (path_obstacle->HasLateralDecision() &&
        path_obstacle->LateralDecision().has_sidepass()) {
      // SIDE_PASS decision
      continue;
    }

    if (path_obstacle->reference_line_st_boundary().boundary_type() ==
        StBoundary::BoundaryType::KEEP_CLEAR) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    const auto &sl_boundary = path_obstacle->PerceptionSLBoundary();

//************************No lateral distance check yet*********************************************
    if (sl_boundary.end_s() < frenet_points.front().s() ||
        sl_boundary.start_s() > frenet_points.back().s()) {
      path_decision->AddLongitudinalDecision("PathDecider/not-in-s",
                                             obstacle.Id(), object_decision);
      path_decision->AddLateralDecision("PathDecider/not-in-s", obstacle.Id(),
                                        object_decision);
      continue;
    }



//****************************Failed in the following call***************************
    const auto frenet_point = frenet_path.GetNearestPoint(sl_boundary);
    const double curr_l = frenet_point.l();
    if (curr_l - lateral_radius > sl_boundary.end_l() ||
        curr_l + lateral_radius < sl_boundary.start_l()) {
      // ignore
      path_decision->AddLateralDecision("PathDecider/not-in-l", obstacle.Id(),
                                        object_decision);
    } else if (curr_l - lateral_stop_radius < sl_boundary.end_l() &&
               curr_l + lateral_stop_radius > sl_boundary.start_l()) {
      // stop
      *object_decision.mutable_stop() =
          GenerateObjectStopDecision(*path_obstacle);

      if (path_decision->MergeWithMainStop(
              object_decision.stop(), obstacle.Id(),
              reference_line_info_->reference_line(),
              reference_line_info_->AdcSlBoundary())) {
        path_decision->AddLongitudinalDecision("PathDecider/nearest-stop",
                                               obstacle.Id(), object_decision);
      } else {
        ObjectDecisionType object_decision;
        object_decision.mutable_ignore();
        path_decision->AddLongitudinalDecision("PathDecider/not-nearest-stop",
                                               obstacle.Id(), object_decision);
      }
    } else if (FLAGS_enable_nudge_decision) {
      // nudge
      if (curr_l - lateral_stop_radius > sl_boundary.end_l()) {
        // LEFT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
        object_nudge_ptr->set_distance_l(FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/left-nudge",
                                          obstacle.Id(), object_decision);
      } else {
        // RIGHT_NUDGE
        ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
        object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
        object_nudge_ptr->set_distance_l(-FLAGS_nudge_distance_obstacle);
        path_decision->AddLateralDecision("PathDecider/right-nudge",
                                          obstacle.Id(), object_decision);
      }
    }
  }

  return true;
}



We also noticed this issue. We are fixing it by cutting the reference line if the curvature is too large. The new code will be released soon. We made some modifications on pnc_map to cut a segment of routing lane segment if we found the curvature is greater than some threshold.

I am thinking that maybe we do not need to check for DEST obstacle when it is very far away, which will cause some mapping issues. We are able to find the destination from routing, and extract the distance from ego vehicle to the destination point.


@lianglia-apollo : Yes, for DEST obstacle, it can be ruled out using routing info. For general static objects, projecting each corner of 2d box onto reference line might be computationally too expensive. How about build a few layers (coarse to fine) of axis aligned 2d boxes (lane width can be used here) for the reference line and then does the collision detection in between the bounding box of obstacle and the ref line? At the top layer, the reference line is represented a big 2d AABox. If there is no collision in between them, we are done. If there is a collision, check the collision to the 2nd layer AABox of ref line. If there is no collision, we are done again, otherwise go to 3rd layer. Repeat the process till the final decision can be made.





// Show Pointcloud error Dreamview #8137-


Please run a bt to get the actual stacktrace.

#0  0x00007f8e96f5dc37 in __GI_raise (sig=sig@entry=6) at ../nptl/sysdeps/unix/sysv/linux/raise.c:56
#1  0x00007f8e96f61028 in __GI_abort () at abort.c:89
#2  0x00007f8e97566535 in __gnu_cxx::__verbose_terminate_handler() () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#3  0x00007f8e975646d6 in ?? () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#4  0x00007f8e97564703 in std::terminate() () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#5  0x00007f8e97564922 in __cxa_throw () from /usr/lib/x86_64-linux-gnu/libstdc++.so.6
#6  0x0000000000fcabdd in google::protobuf::internal::LogMessage::Finish (this=0x7f8d2effeec0) at external/com_google_protobuf/src/google/protobuf/stubs/common.cc:268
#7  0x0000000000fcac18 in google::protobuf::internal::LogFinisher::operator= (this=0x7f8d2effee9f, other=...) at external/com_google_protobuf/src/google/protobuf/stubs/common.cc:276
#8  0x000000000068becf in google::protobuf::internal::RepeatedPtrFieldBase::Get<google::protobuf::RepeatedPtrField<apollo::drivers::PointXYZIT>::TypeHandler> (this=0x7f8d0c1b3c30, 
    index=10971) at external/com_google_protobuf/src/google/protobuf/repeated_field.h:1482
#9  0x000000000068922e in google::protobuf::RepeatedPtrField<apollo::drivers::PointXYZIT>::Get (this=0x7f8d0c1b3c30, index=10971)
    at external/com_google_protobuf/src/google/protobuf/repeated_field.h:1919
#10 0x000000000068737c in apollo::drivers::PointCloud::point (this=0x7f8d0c1b3c18, index=10971) at bazel-out/local-dbg/genfiles/modules/drivers/proto/pointcloud.pb.h:637
#11 0x000000000068538b in apollo::dreamview::PointCloudUpdater::UpdatePointCloud (this=0x3cbdc50, point_cloud=std::shared_ptr (count 3, weak 0) 0x7f8d0c1b3c18)
    at modules/dreamview/backend/point_cloud/point_cloud_updater.cc:166
#12 0x0000000000684fec in apollo::dreamview::PointCloudUpdater::__lambda66::operator() (__closure=0x3d46a60, msg=std::shared_ptr (count 3, weak 0) 0x7f8d0c1b3c18)
    at modules/dreamview/backend/point_cloud/point_cloud_updater.cc:129
#13 0x0000000000686314 in std::_Function_handler<void(const std::shared_ptr<apollo::drivers::PointCloud>&), apollo::dreamview::PointCloudUpdater::Start()::__lambda66>::_M_invoke(const std::_Any_data &, const std::shared_ptr<apollo::drivers::PointCloud> &) (__functor=..., __args#0=std::shared_ptr (count 3, weak 0) 0x7f8d0c1b3c18)
    at /usr/include/c++/4.8/functional:2071
#14 0x00000000006981eb in std::function<void (std::shared_ptr<apollo::drivers::PointCloud> const&)>::operator()(std::shared_ptr<apollo::drivers::PointCloud> const&) const (
    this=0x3d3b028, __args#0=std::shared_ptr (count 3, weak 0) 0x7f8d0c1b3c18) at /usr/include/c++/4.8/functional:2471
#15 0x0000000000694bd5 in apollo::cyber::Reader<apollo::drivers::PointCloud>::Init()::{lambda(std::shared_ptr<apollo::drivers::PointCloud> const&)#1}::operator()(std::shared_ptr<apollo::drivers::PointCloud> const&) const (__closure=0x3d3f620, msg=std::shared_ptr (count 3, weak 0) 0x7f8d0c1b3c18) at ./cyber/node/reader.h:142
#16 0x000000000069fd2f in std::_Function_handler<void (std::shared_ptr<apollo::drivers::PointCloud> const&), apollo::cyber::Reader<apollo::drivers::PointCloud>::Init()::{lambda(std::shared_ptr<apollo::drivers::PointCloud> const&)#1}>::_M_invoke(std::_Any_data const&, std::shared_ptr<apollo::drivers::PointCloud> const&) (__functor=..., 
    __args#0=std::shared_ptr (count 3, weak 0) 0x7f8d0c1b3c18) at /usr/include/c++/4.8/functional:2071
#17 0x00000000006981eb in std::function<void (std::shared_ptr<apollo::drivers::PointCloud> const&)>::operator()(std::shared_ptr<apollo::drivers::PointCloud> const&) const (
    this=0x3d3f6e0, __args#0=std::shared_ptr (count 3, weak 0) 0x7f8d0c1b3c18) at /usr/include/c++/4.8/functional:2471
#18 0x00000000006984b0 in apollo::cyber::croutine::RoutineFactory apollo::cyber::croutine::CreateRoutineFactory<apollo::drivers::PointCloud, std::function<void (std::shared_ptr<apollo::drivers::PointCloud> const&)> >(std::function<void (std::shared_ptr<apollo::drivers::PointCloud> const&)>&&, std::shared_ptr<apollo::cyber::data::DataVisitor<apollo::drivers::PointCloud, apollo::cyber::NullType, apollo::cyber::NullType, apollo::cyber::NullType> > const&)::{lambda()#1}::operator()() const::{lambda()#1}::operator()() const (__closure=0x3d3f6d0)
    at ./cyber/croutine/routine_factory.h:61
#19 0x00000000006a9ea2 in std::_Function_handler<void (), apollo::cyber::croutine::RoutineFactory apollo::cyber::croutine::CreateRoutineFactory<apollo::drivers::PointCloud, std::function<void (std::shared_ptr<apollo::drivers::PointCloud> const&)> >(std::function<void (std::shared_ptr<apollo::drivers::PointCloud> const&)>&&, std::shared_ptr<apollo::cyber::data::DataVisitor<apollo::drivers::PointCloud, apollo::cyber::NullType, apollo::cyber::NullType, apollo::cyber::NullType> > const&)::{lambda()#1}::operator()() const::{lambda()#1}>::_M_invoke(std::_Any_data const&) (__functor=...) at /usr/include/c++/4.8/functional:2071
#20 0x00000000004ad786 in std::function<void ()>::operator()() const (this=0x3d3f670) at /usr/include/c++/4.8/functional:2471
#21 0x0000000000d400c0 in apollo::cyber::croutine::CRoutine::Run (this=0x3d3f658) at ./cyber/croutine/croutine.h:143
#22 0x0000000000d3f715 in apollo::cyber::croutine::(anonymous namespace)::CRoutineEntry (arg=0x3d3f658) at cyber/croutine/croutine.cc:43
#23 0x0000000000000000 in ?? ()





// Cyber can monitor and evaluate the performance of each module #7968
cyber/setup.bash
How can cyber monitor the scheduling and execution time of each module? Detecting the execution time of each task can meet the requirements? I found that by setting cyber_sched_perf=1, I can open the event log of task switching. Is there a visual tool available?


If "cyber_sched_perf=1" is set, will it affect performance? How can we measure it? How can we measure the performance of each module task and arrange the scheduling strategy reasonably?

hi, zyffei, i think there is 3 questions you ask here~ answers as follows:

questions about cyber_sched_perf?
Answer: perf event is developed for scheduler debugging, it record scheduler event like notify、swap_in、swap_out etc... it's not suitable for module proc time statistic, because module's proc function(it is croutine in cyber) may swap_in && swap_out several times, it is hard to organize these events together to calc module proc time. finally, perf event certainly will affect performance(about 5%~10% E2E latency jitter), so we use it as offline debug tool when we meet framework performance issues
How can we measure the performance of each module task ?
Answer: we have a E2E latency statistic && analysis tool for inner online use, but it's not suitable for all auto-drive products for now. we will have an internal discussion of this. i suggest u can try print logs in the proc start point && end point to statistic module proc time for debug purpose~ or transform a sensor time stamp from driver to control to make statistic of E2E(end to end) latency for you to improve total performance && tail performance
arrange the scheduling strategy reasonably?
Answer: we suggest developers use classic scheduler policy for common use or for the beginning. if u are already familiar with cyber, u can try choreo schedule policy to achieve higher performance, the advantage of choreo is its determinacy by binding related tasks to unique processor and given enough computing resource, the disadvantage of choreo is if we didn't binding tasks properly(total execute time of tasks in same processor exceed the period of sensor data related to them), we will gain a poor E2E performance. u can use choreo policy as follows:
a. tasks in auto-driving car organized as a DAG(Direct Acyclic Graph);
b. the task in the critical path(most important path in DAG, for example modules like lidar driver、perception、planning、control etc.) should bind to same processor as possible for better cpu locality
c. children node should have a higher priority than it's father node
d. total execute time of tasks in same processor won't exceed the period of sensor data;
e. we can arrange other tasks which is not in the critical path into the processor pool temporary;
f. we packaged linux sched confs into cyber sched conf, u can set linux sched policy && sched priority && pin cyber sched processor to proper cpus as u need. u can set linux sched policy according to this article: https://drkp.net/papers/latency-socc14.pdf
you can access to cyber/conf/example_sched_choreography.conf && cyber/conf/example_sched_classic.conf for more details


Thank you very much. Now we analyze the execution time of proc almost according to the method mentioned in Point 2. Is your tool measurement available in actual automatic driving? Do you need to modify cyber code? Will it affect performance?


as metioned above, perf event in cyber is for offline debug purpose, it can run in actual automatic driving, but i suggest turn off this tool if we run auto driving for mpi(online)~ "export cyber_sched_perf=1" in setup.bash if u need debug offline. then follow the output to analysis framework performance according to the enum follows:
enum class SchedPerf {
SWAP_IN = 1,
SWAP_OUT = 2,
NOTIFY_IN = 3,
NEXT_RT = 4,
RT_CREATE = 5,
};





// Carla 0.9.x Apollo bridge #7774


@volprjir Hi, you can check LGSVL simulator which already supports connecting with Apollo 3.5. You can see the demo here. You can try it with their Apollo 3.5 fork.
https://github.com/lgsvl/simulator


Could we summarize the options to connect Apollo to Carla?

1.Cyber Python API (examples)
2. LGSVL Bridge
3.[Cyber C++ API] + [Carla C++ API]
This way should be better for debugging. We could move on Cyber Python API after this.
Because the attempt on Cyber Python API has not been successful. #7957

4. [Cyber C++ API] + [Carla python API], the server-client way like LGSVL Bridge, with a python wrapped C++ client or server at the Carla side.
Is there any other option?

LGSVL Bridge
https://github.com/lgsvl/apollo-3.5/tree/simulator/cyber/bridge





// Question: Can we visualize point cloud data using Dreamview? #6909


Problem:
I wonder if we can visualize our own point cloud data in Dreamview just like rviz. I have tried to visualize my own point cloud data in dreamview by publishing the data into the topic "/apollo/sensor/velodyne64/compensator/PointCloud2". I have lit up the button of "velodyne" in the Dreamview, but cannot visualize the point cloud. Have I done it in a wrong way? Or is there a correct way/tutorial about visualizing point cloud data in Dreamview?

I have solved this problem by feeding a localization message into "/apollo/localization/pose" topic. It seems that at least we need the message from these two topics to visualize our own point clouds in dreamview.






// A problem in Lat controller lateral_error calculation #9070

// modules/control/controller/lat_controller.cc
  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;


  // raw lateral error #2786
case 1: when the car is on the top of lane centerline, dx < 0 (x_current - x_des) , dy > 0. (y_current - y_des) , we can use raw_lateral_error = cos_theta * dy - sin_theta * dx;

case 2: when the car is on the down of the lane centerline, dx < 0 (x_current - x_des), , dy < 0. (y_current - y_des), in this case, maybe we can not still use this formula of raw_lateral_error = cos_theta * dy - sin_theta * dx;

The dx is always minus one, since we have to make the sedan to follow the future trajectory points which are always in front of the sedan. However, the sign of the dy depends on which side(top or down of the lane centerline) the sedan is located.





// Log Design in 3.5 #7427

There has been a lot of changes for logging from 3.0 to 3.5. I had a lot of questions regarding the design and configuration of logging in 3.5.

1. Was it intentional to have the minloglevel as part of the symlink name for latest log per module? The output from glog is /apollo/data/MODULE_NAME.MINLOGLEVEL, but I am not sure that the MINLOGLEVEL part should be there.
Answer the question No.1:
Yes, we followed the link name generation rules of glog.
In glog project: https://github.com/google/glog/blob/master/src/logging.cc#L932
In apollo project: https://github.com/ApolloAuto/apollo/blob/master/cyber/logger/log_file_object.cc#L171



2. I noticed that GLOG_alsologtostderr=0 in cyber/setup.bash. This means that DEBUG, INFO, and WARN will not be output to terminal or redirected to nohup.out. By specifying this glog parameter, there are essentially two log levels. The C++ cyber examples do not print anything to terminal, unless GLOG_alsologtostderr=1 is passed in before bazel-bin/cyber/examples/talker or bazel-bin/cyber/examples/listener. Was this intentional?

Answer the question No.2:
The "cyber/setup.bash" file contains some default environment variables. In most cases, we do not want to log to standard error because this will increase the system load. So we just default output the error log. You can customize the "cyber/setup.bash" file and set GLOG_alsologtostderr to 0.



3.Most scripts and dreamview start programs through nohup and redirect the output to a single nohup.out file in the root directory. Was it meant to go in data/log? Also, was it meant for all of the output filenames to be the same?





4. I was not  sure what module means in context of logging. It is set by copts = ["-DMODULE_NAME=\\\"module_name\\\""], in the BUILD file. It can be confusing, because common, compiled libraries will have the same module_name. In compiled libraries it is similar to __FUNCTION__, __LINE__, and __FILE__. Could those be used instead? If the header file has inline function that log, it will have different MODULE_NAME definitions. If a cyber program crashes, it is not obvious what the FATAL log file will be named, since it does not correlate to program, component, or dag name. If headers are precompiled, this macro will also slow down compilation. Also, the nohup.out will have text from other programs so a FATAL message will not be at the end of it. Should it use the dag name instead?

Answer the question No.4:
As you pointed out，the MODULE_NAME macro variable to split the log file is not a best practice.
We are developing new logging features, like split the log file according to component name or module name defined in dag file.





















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