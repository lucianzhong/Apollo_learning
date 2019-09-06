
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


	// /modules/planning/common/frame.cc
1. 在初始化frame类时即Frame::Init()会调用bool Frame::CreateReferenceLineInfo()，在bool ReferenceLineProvider::GetReferenceLines中从相对地图处获取reference line和segment，在获取完信息后存放在ReferenceLineInfo类指针中。同时在bool Frame::CreateReferenceLineInfo()中通过如下判断本车道优先还是临近车道优先，
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

  const SLBoundary& AdcSlBoundary() const;  //  mean the real time location 
  const SLBoundary& VehicleSlBoundary() const; // means the planning start point




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