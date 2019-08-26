
// https://github.com/YannZyl/Apollo-Note/blob/master/docs/planning/reference_line_provider.md
// http://www.hostmath.com/

1. 	参考线提供器: ReferenceLineProvider
	参考线提供器主要完成的工作是计算车辆在规划路径上的短期可行路径。
	在控制规划地图Pnc Map中，有一个功能是路径段RouteSegments生成最终路径Path，这个Path就是车辆在规划路径上的可行路径，但是路径是以std::vector<common::math::LineSegment2d> segments_和std::vector<MapPathPoint> path_points_的离散形式存在的，
	如果还不了解，可以参考控制规划地图Pnc Map进行了解。而本节参考线提供器就是对上述的Path进行样条函数Spline插值，得到平滑的路径曲线。
	
	对path_points_中的离散点进行样条函数插值，得到连续平滑的路径函数，主要的步骤包含有:
		路径点采样与轨迹点矫正
		knots分段与二次规划进行参考线平滑
		除了参考线平，参考线提供器还提供参考线拼接的功能。参考线拼接是针对不同时刻的RawReference，如果两条原始的RawReference是相连并且有覆盖的，那么可以不需要重新去进行平滑，只要直接使用上时刻的平滑参考线，或者仅仅平滑部分anchor point即可



2. // 功能1参考线平滑：路径点采样与轨迹点矫正

	控制规划地图Pnc Map根据当前车辆状态与Routing模块规划路径响应可以得到当前情况下，车辆的可行驶方案Path的集合(每个RouteSegments生成对应的一个Path)。
	在Path中路径以std::vector<common::math::LineSegment2d> segments_和std::vector<MapPathPoint> path_points_存在，前者是段的形式，而后者是原始离散点的形式。那么这个Path其实就是路径的离散形式表示，
	在本节中，我们需要行驶路径(参考线)的连续表示法，也就是根据这些离散点拟合一个合理的连续函数。
	路径点采样与轨迹点矫正阶段工作就是对路径做一个离散点采样，为什么不用知道的path_points_或者sample_points_，因为该阶段需要的条件不同，所以其实就是重新采样的过程，与sample_points_采样其实没太大区别，仅仅是采样的间隔不同。
	首先，已知路径的长度length_，只需要给出采样的间距interval，就能完成采样:

	// modules/planning/reference_line/reference_line_provider.cc
	void ReferenceLineProvider::GetAnchorPoints(  const ReferenceLine &reference_line,  std::vector<AnchorPoint> *anchor_points) const {
	  CHECK_NOTNULL(anchor_points);
	   // interval为采样间隔，默认max_constraint_interval=5.0，即路径累积距离每5m采样一个点
	  const double interval = smoother_config_.max_constraint_interval();
	   // 路径采样点数量计算
	  int num_of_anchors =  std::max(2, static_cast<int>(reference_line.Length() / interval + 0.5));
	  std::vector<double> anchor_s;
	  // uniform_slice函数就是对[0.0, reference_line.Length()]区间等间隔采样，每两个点之间距离为(length_-0.0)/(num_of_anchors - 1)
	  common::util::uniform_slice(0.0, reference_line.Length(), num_of_anchors - 1, &anchor_s);
	   // 根据每个采样点的累积距离s，以及Path的lane_segments_to_next_point_进行平滑插值，得到累积距离为s的采样点的坐标(x,y)，并进行轨迹点矫正
	  for (const double s : anchor_s) {
	    AnchorPoint anchor = GetAnchorPoint(reference_line, s); // GetAnchorPoint函数是如何完成采样点的坐标计算与轨迹点坐标矫正的
	    anchor_points->emplace_back(anchor);
	  }
	  anchor_points->front().longitudinal_bound = 1e-6;
	  anchor_points->front().lateral_bound = 1e-6;
	  anchor_points->front().enforced = true;
	  anchor_points->back().longitudinal_bound = 1e-6;
	  anchor_points->back().lateral_bound = 1e-6;
	  anchor_points->back().enforced = true;
	}



	AnchorPoint ReferenceLineProvider::GetAnchorPoint(const ReferenceLine &reference_line, double s) const {
	  AnchorPoint anchor;
	  anchor.longitudinal_bound = smoother_config_.longitudinal_boundary_bound();
	  auto ref_point = reference_line.GetReferencePoint(s); //首先采样点坐标计算与Pnc Map中Path::InitWidth采样点计算一致，虽然这里多了一层ReferenceLine::GetReferencePoint的函数包装，但是计算过程是一模一样的
	  // 当完成采样点的计算以后，下一步就是采样点(也就是轨迹点)的坐标矫正。为什么需要坐标矫正？因为采样点坐标是在道路的中心线上，但当道路比较宽时，车辆不能一味的在中间行驶，需要考虑到其他车辆超车情况。
	  // 在这种情况下，车辆需要靠右行驶(当然不同区域的模式不一样，部分地区是靠左形式)，所以道路过宽时，需要将轨迹点向右或者向左矫正一段距离
	  if (ref_point.lane_waypoints().empty()) {
	    anchor.path_point = ref_point.ToPathPoint(s);
	    anchor.lateral_bound = smoother_config_.lateral_boundary_bound();
	    return anchor;
	  }
	  // 当完成采样点的计算以后，下一步就是采样点(也就是轨迹点)的坐标矫正。为什么需要坐标矫正？因为采样点坐标是在道路的中心线上，但当道路比较宽时，车辆不能一味的在中间行驶，需要考虑到其他车辆超车情况。
	  // 在这种情况下，车辆需要靠右行驶(当然不同区域的模式不一样，部分地区是靠左形式)，所以道路过宽时，需要将轨迹点向右或者向左矫正一段距离
	  // 矫正的步骤如下：
	  // 1. 计算车辆宽度adc_width和半宽adc_half_width
	  const double adc_width = VehicleConfigHelper::GetConfig().vehicle_param().width();
	  const double adc_half_width = adc_width / 2.0;
	  const Vec2d left_vec = Vec2d::CreateUnitVec2d(ref_point.heading() + M_PI / 2.0);
	  auto waypoint = ref_point.lane_waypoints().front();
	  // shift to center
	  // 2. 计算车道距左边界距离left_width和距右边界距离right_width
	  double left_width = 0.0;
	  double right_width = 0.0;
	  waypoint.lane->GetWidth(waypoint.s, &left_width, &right_width); // Hd Map中查询得到
	  double total_width = left_width + right_width;	// 当前位置，车道总宽度
	  // only need to track left side width shift
	  double shifted_left_width = total_width / 2.0;

	  // shift to left (or right) on wide lanes
	  // 3.计算车辆应该右移或者左移(矫正)的距离
	  // //如果车道宽度大于车辆宽度的wide_lane_threshold_factor倍，默认为2，则需要靠边行驶，因为存在其他车辆超车的可能性 // shift to left (or right) on wide lanes
	  if (smoother_config_.wide_lane_threshold_factor() > 0 && total_width > adc_width * smoother_config_.wide_lane_threshold_factor()) {
	    if (smoother_config_.driving_side() == ReferenceLineSmootherConfig::RIGHT) {// 靠右行驶模式
	      shifted_left_width = adc_half_width + adc_width * smoother_config_.wide_lane_shift_remain_factor();
	    } else { // 靠左形式模式
	      shifted_left_width = std::fmax( adc_half_width, total_width - (adc_half_width + adc_width * smoother_config_.wide_lane_shift_remain_factor()));
	    }
	  }

	  // shift away from curb boundary
	  // 第二部分矫正是根据左右边界线是否是道路边界，如果车道左边界是道路边界线，那么shifted_left_width需要加上一个边界的缓冲距离curb_shift，默认0.2m，反之就是减去缓冲边界距离
	  auto left_type = hdmap::LeftBoundaryType(waypoint);
	  if (left_type == hdmap::LaneBoundaryType::CURB) {
	    shifted_left_width += smoother_config_.curb_shift();
	  }
	  auto right_type = hdmap::RightBoundaryType(waypoint);
	  if (right_type == hdmap::LaneBoundaryType::CURB) {
	    shifted_left_width -= smoother_config_.curb_shift();
	  }

	  // 最后矫正得到的平移距离就是shifted_left_width，如何根据这个平移距离求出车辆在世界坐标系中的矫正位置?
	  ref_point += left_vec * (left_width - shifted_left_width);
	  auto shifted_right_width = total_width - shifted_left_width;
	  anchor.path_point = ref_point.ToPathPoint(s);
	  double effective_width = std::min(shifted_left_width, shifted_right_width) -  adc_half_width - FLAGS_reference_line_lateral_buffer;
	  anchor.lateral_bound = std::max(smoother_config_.lateral_boundary_bound(), effective_width); //最后当使用二次规划来拟合轨迹点时，需要设置该点的约束，lateral_bound指的是预测的x值需要在ref_point的L轴的lateral_bound左右领域内，longitudinal_bound是预测的y值需要在ref_point的F轴的longitudinal_bound前后领域内
	  return anchor;
	}


	/*
	从代码可以看到，如果官方代码没有错误，那么：

	当driving_side为RIGHT时
	m = adc_half_width + adc_width * smoother_config_.wide_lane_shift_remain_factor()计算矫正后车辆与左边界距离
	当driving_side为LEFT时
	m = adc_half_width + adc_width * smoother_config_.wide_lane_shift_remain_factor()计算矫正后车辆与右边界距离
	这样的方式在车道宽度差异比较大的时候，车联靠边行驶，距离边界的距离(total_width - m)，这个距离不好控制。更不如使用一种更好的方法，同样是计算m，但是m的意义是距离最近边界线的距离，即靠边行驶与边界的距离，这个距离相对来说就比较好控制。只需要做一个更简单的改动，将上述代码的if-else判断条件交换一下即可，wide_lane_shift_remain_factor可以设置小于0.5的值。保证车辆边界与车到边界有一定距离(间隙)。

	*/


3.  // 功能1参考线平滑：knots分段与二次规划进行参考线平滑

	通过第一阶段路径点采样与轨迹点矫正，可以得到这条路径的anchor_point集合，里面是若干矫正过后的轨迹点，但还是离散形式。这个阶段我们需要对上述离散轨迹点进行多项式拟合。这部分内容也可以参考Apollo参考线平滑器。
	官方文档介绍的偏简单，在这里将从代码入手介绍一下参考线平滑过程

	函数的输入和输出又是什么？我们现在只anchor_point中每个点的车道累积距离差s，以及世界系坐标(x,y)。想要拟合出轨迹曲线，只能是累积距离s作为自变量，世界系坐标作为应变量。计算函数为：

	$$ x = f_i(s) = a_{i0} + a_{i1}s + a_{i2}s^2 +a_{i3}s^3 + a_{i4}s^4 + a_{i5}s^5 $$

	$$ y = g_i(s) = b_{i0} + b_{i1}s + b_{i2}s^2 +b_{i3}s^3 + b_{i4}s^4 + b_{i5}s^5 $$

	在这里是分别对x和y用多项式函数拟合，函数的参数a和b的下标i表示哪一个段(两个knots之间的anchor point)


	modules/planning/reference_line/qp_spline_reference_line_smoother.cc

	预处理：如何划分段，或者说设置knots？
	简单，anchor point是对原始Path进行采样，采样间隔为smoother_config_.max_constraint_interval()，默认5m一个点。knots的采样其实也是相似的，采样间隔为config_.qp_spline().max_spline_length()，默认25m：
	 uint32_t num_spline =
      std::max(1u, static_cast<uint32_t>(
                       length / config_.qp_spline().max_spline_length() + 0.5));
	  for (std::uint32_t i = 0; i <= num_spline; ++i) {
	    t_knots_.push_back(i * 1.0);
	  }


	 最后得到的knots节点有num_spline+1个。得到了所有的knots，也就意味着可到了所有的段，很明显这里就需要拟合num_spline个段，每个段有x和y两个多项式函数。
	此外，还需要对anchor_point的自变量s做处理，本来s是从0到length_递增，现进行如下处理: 
	 const double scale = (anchor_points_.back().path_point.s() -
                        anchor_points_.front().path_point.s()) /
                       (t_knots_.back() - t_knots_.front());
  std::vector<double> evaluated_t;
  for (const auto& point : anchor_points_) {
    evaluated_t.emplace_back(point.path_point.s() / scale);
  }

  // 不难理解，就是将自变量s从[0,length_]区间按比例映射到[0,num_spline]区间，这样每个段内anchor point的s都属于[a,a+1]内，如果在减去knots[a]那么所有自变量的取值范围就是[0,1]，事实上代码中也是这样做的


  // 同时还需要对应变量(x,y)做处理，处理方法如下
  // 可以看到x和y都需要减去Path第一个点的世界坐标系坐标，说白了2n个(2*num_spline)函数的坐标原点是Path的第一个点
    for (const auto& point : anchor_points_) {
    const auto& path_point = point.path_point;
    headings.push_back(path_point.theta());
    longitudinal_bound.push_back(point.longitudinal_bound);
    lateral_bound.push_back(point.lateral_bound);
    xy_points.emplace_back(path_point.x() - ref_x_, path_point.y() - ref_y_);
  }



4. // B. 如何设置约束条件？
	
	在上一步预处理阶段，已经知道：

		1.需要拟合的多项式函数数量为2*num_spline个，每两个knots之间会拟合x和y两个多项式

		2. 多项式最高阶数为5(qp_spline.spline_order: 5)，所以每个多项式共6个参数，参数总和：(spline_order+1)*2*num_spline

		3. 使用每个段内的anchor point去拟合多项式函数，自变量范围[0,1]，应变量相对于第一个anchor point的相对坐标。所以最后拟合出来的函数f和g的结果是相对于第一个anchor point的相对坐标。

		那么在拟合过程中还需要满足一些约束，包括等式约束和不等式约束，例如：

			1. 预测的x'和y'需要保证在真实x和y的L轴lateral_bound、F轴longitudinal_bound领域内
			2. 第一个anchor point的heading和函数的一阶导方向需要一致，大小可以不一致，但是方向必需一致！
			3. x和y的n段函数之间，两两接壤部分应该是平滑的，两个函数值(位置)、一阶导(速度)、二阶导(加速度)必须一致。


	// B.1 边界约束
		bool QpSplineReferenceLineSmoother::AddConstraint() {
		  // Add x, y boundary constraint
		  std::vector<double> headings;
		  std::vector<double> longitudinal_bound;
		  std::vector<double> lateral_bound;
		  std::vector<common::math::Vec2d> xy_points;
		  for (const auto& point : anchor_points_) {
		    const auto& path_point = point.path_point;
		    headings.push_back(path_point.theta());
		    longitudinal_bound.push_back(point.longitudinal_bound);
		    lateral_bound.push_back(point.lateral_bound);
		    xy_points.emplace_back(path_point.x() - ref_x_, path_point.y() - ref_y_);
		  }
		  const double scale = (anchor_points_.back().path_point.s() -
		                        anchor_points_.front().path_point.s()) /
		                       (t_knots_.back() - t_knots_.front());
		  std::vector<double> evaluated_t;
		  for (const auto& point : anchor_points_) {
		    evaluated_t.emplace_back(point.path_point.s() / scale);
		  }

		  auto* spline_constraint = spline_solver_->mutable_constraint();
		  // all points (x, y) should not deviate anchor points by a bounding box
		  if (!spline_constraint->Add2dBoundary(evaluated_t, headings, xy_points,
		                                        longitudinal_bound, lateral_bound)) {
		    AERROR << "Add 2d boundary constraint failed.";
		    return false;
		  }
		  // the heading of the first point should be identical to the anchor point.

		  if (FLAGS_enable_reference_line_stitching &&
		      !spline_constraint->AddPointAngleConstraint(evaluated_t.front(),
		                                                  headings.front())) {
		    AERROR << "Add 2d point angle constraint failed.";
		    return false;
		  }
		  // all spline should be connected smoothly to the second order derivative.
		  if (!spline_constraint->AddSecondDerivativeSmoothConstraint()) {
		    AERROR << "Add jointness constraint failed.";
		    return false;
		  }
		  return true;
		}

		每个anchor point相对第一个点的相对参考系坐标为(x,y)，方向为heading。而该点坐在的段拟合出来的相对参考系坐标为(x',y')，坐标的计算方式为:
			$$ x' = f_i(s) = a_{i0} + a_{i1}s + a_{i2}s^2 +a_{i3}s^3 + a_{i4}s^4 + a_{i5}s^5 $$
			$$ y' = g_i(s) = b_{i0} + b_{i1}s + b_{i2}s^2 +b_{i3}s^3 + b_{i4}s^4 + b_{i5}s^5 $$
			其中i是anchor point所在的knots段，i=1,2,...,n(n=num_spline)


	// 真实点(x,y)F和L轴投影计算
	如上图，实际情况下需要满足拟合坐标(x',y')在真实坐标(x,y)的领域内，真实点的投影计算方法比较简单，首先坐标在侧方L轴上的投影(天蓝色星星)，投影点到原点的距离，也就是侧方距离计算方式为：

	$$ x_{p,later} = (cos(\theta+\pi/2), sin(\theta+\pi/2))·(x, y) $$

	注意上述公式·为内积操作。这部分对应的代码为：
	modules/planning/math/smoothing_spline/spline_2d_constraint.cc

	const double d_lateral = SignDistance(ref_point[i], angle[i]); // ref_point[i]是第i个anchor point的相对参考系坐标，angle[i]为该点的方向heading，也是公式中的theta

	double Spline2dConstraint::SignDistance(const Vec2d& xy_point, const double angle) const {
	  return common::math::InnerProd(
	      xy_point.x(), xy_point.y(),
	      -common::math::sin(common::math::Angle16::from_rad(angle)),
	      common::math::cos(common::math::Angle16::from_rad(angle)));
	}


	注意一点，代码中的方向向量是(-sin(angle), cos(angle))，其实也可以等价为(cos(angle+pi/2), sin(angle+pi/2))，很明显，代码中的方向向量是在L轴的，所以在计算L轴上的投影距离是，直接将heading传入即可，不需要额外加上一个pi/2。最终代码的计算方式与公式是一致的。

真实点坐标在前方F轴上的投影(大红色星星)，投影点到原点的距离，也就是前方距离计算方式为：

$$ y_{p,longi} = (cos(\theta), sin(\theta))·(x, y) $$


注意上述公式·为内积操作。对应的代码为:

const double d_longitudinal = SignDistance(ref_point[i], angle[i] - M_PI / 2.0);
从L轴到F轴的方向向量，需要减去一个pi/2。






5. // B.2 方向约束



















6. //B.3 各函数接壤处平滑约束




















7. //.C. 如何设置cost函数？

由Apollo参考线平滑器可以看到Apollo使用的cost函数:

$$ cost = \sum_{i=1}^{n} \Big( \int\limits_{0}^{t_i} (f_i''')^2(t) dt + \int\limits_{0}^{t_i} (g_i''')^2(t) dt \Big) $$


/modules/planning/conf/qp_spline_smoother_config.pb.txt


qp_spline {
  spline_order: 5
  max_spline_length : 25.0
  regularization_weight : 1.0e-5
  second_derivative_weight : 200.0	 // 二阶导cost函数权重
  third_derivative_weight : 1000.0	// 三阶导cost函数权重
}



实际上，从代码和配置文件中可以看到，其实cost函数用了二阶导和三阶导，下面我们以三阶导和第k段多项式x曲线fk为例，描述cost函数的计算过程。可知fk多项式函数的0,1,2,3阶导函数分别为：

实际上，从代码和配置文件中可以看到，其实cost函数用了二阶导和三阶导，下面我们以三阶导和第k段多项式x曲线fk为例，描述cost函数的计算过程。可知fk多项式函数的0,1,2,3阶导函数分别为：

$$ x = f_k(s) = a_{k0} + a_{k1}s + a_{k2}s^2 + a_{k3}s^3 + a_{k4}s^4 + a_{k5}s^5 $$

$$ x' = f_k^{(1)}(s) = 0 + a_{k1} + 2a_{k2}s + 3a_{k3}s^2 + 4a_{k4}s^3 + 5a_{k5}s^4 $$

$$ x'' = f_k^{(2)}(s) = 0 + 0 + 2 + 6a_{k3}s + 12a_{k4}s^2 + 20a_{k5}s^3 $$

$$ x''' = f_k^{(3)}(s) = 0+ 0 + 0 + 6 + 24a_{k4}s^1 + 60a_{k5}s^2 $$

先做如下标记：

$$ Ds_0 = [1, s, s^2, s^3, s^4, s^5] $$

$$ Ds_1 = [0, 1, 2s, 3s^2, 4s^3, 5s^4] $$

$$ Ds_2 = [0, 0, 2, 6s, 12s^2, 20s^3] $$

$$ Ds_3 = [0, 0, 0, 6, 24s, 60s^2] $$

$$ A_k = [a_{k0}, a_{k1}, a_{k2}, a_{k3}, a_{k4}, a_{k5}]^T $$

$$ B_k = [b_{k0}, b_{k1}, b_{k2}, b_{k3}, b_{k4}, b_{k5}]^T $$

最终cost可以变为：

$$ cost = \sum_{k=1}^{n} \Big( \int\limits_{0}^{t_k} (Ds_3A_k)^T(Ds_3A_k) dt + \int\limits_{0}^{t_k} (Ds_3B_k)^T(Ds_3B_k) dt \Big) $$

$$ cost = \sum_{k=1}^{n} \Big( \int\limits_{0}^{t_k} {A_k}^T{Ds_3}^TDs_3A_k dt + \int\limits_{0}^{t_k} {B_k}^T{Ds_3}^TDs_3B_k dt \Big) $$

以第k段多项式函数fk为例，接下来的难点就是如何求解

$$ \int\limits_{0}^{t_k} {A_k}^T{Ds_3}^TDs_3A_k dt = {A_k}^T\Big(\int\limits_{0}^{t_k} {Ds_3}^TDs_3 dt\Big)A_k $$

等价于求解

$$ PP_k = \int\limits_{0}^{t_k} {Ds_3}^TDs_3 dt $$

上述公式就跟上述的约束一样，也是一个系数矩阵。

现令：$Pk = {Ds_3}^TDs_3 $ 可以思考一下，Pk中的任意一个元素$Pk_{ij}$，他的完整计算方式为：

$$ Pk_{ij} = Ds_3[i] * Ds_3[j] = i*(i-1)*(i-2)s^{i-3} * j*(j-1)*(j-2)s^{j-3} = cs^{i+j-6} $$

上述公式中的参数 $ c = i*(i-1)*(i-2)*j*(j-1)*(j-2) $

那么对于这个选一项积分，可以得到：

$$ \int\limits_{0}^{t_k} Pk_{ij} dt = \int\limits_{0}^{t_k} cs^{i+j-6} dt = \left. \frac{c}{i+j-5}s^{i+j-5} \right| _{s=t_k} - \left. \frac{c}{i+j-5}s^{i+j-5} \right| _{s=0} = \left. \frac{c}{i+j-5}s^{i+j-5} \right| _{s=t_k} $$

上述公式需要满足条件: i, j必须都大于等于3

所以最终的积分系数矩阵Pk可以分别成两部分：

PPk = Qk(同kernel_third_order_derivative_) · Rk(同term_matrix)(注意这里是点乘，并非矩阵乘法)

上述中kernel_third_order_derivative_是积分三阶导系数矩阵，term_matrix是s的多项式矩阵。


// C.1 积分导数系数矩阵Q计算












8. //D. 如何优化求解系数

请参考开源软件qpOASES

求解完这个QP问题就可以得到系数矩阵，也变相得到了x和y的这n段平滑曲线，最后的工作就是包装秤一条ReferenceLine，如何包装？答案是采样，用更细的细度去采样离散保存这条基准线

// modules/planning/reference_line/qp_spline_reference_line_smoother.cc


bool QpSplineReferenceLineSmoother::Smooth(
    const ReferenceLine& raw_reference_line,
    ReferenceLine* const smoothed_reference_line) {
  Clear();
  const double kEpsilon = 1e-6;
  if (!Sampling()) {
    AERROR << "Fail to sample reference line smoother points!";
    return false;
  }

  spline_solver_->Reset(t_knots_, config_.qp_spline().spline_order());

  if (!AddConstraint()) {
    AERROR << "Add constraint for spline smoother failed";
    return false;
  }

  if (!AddKernel()) {
    AERROR << "Add kernel for spline smoother failed.";
    return false;
  }

  auto start = Clock::NowInSeconds();
  if (!Solve()) {
    AERROR << "Solve spline smoother problem failed";
  }
  auto end = Clock::NowInSeconds();
  ADEBUG << "QpSplineReferenceLineSmoother solve time is "
         << (end - start) * 1000 << " ms.";

  //   // Step A
  // mapping spline to reference line point
  const double start_t = t_knots_.front();  // 原始ReferenceLine的起点，第一个anchor point，也是平滑后的ReferenceLine起点
  const double end_t = t_knots_.back();	 // 原始ReferenceLine的终点，最后一个anchor point，也是平滑后的ReferenceLine终点

  const double resolution = (end_t - start_t) / (config_.num_of_total_points() - 1); // 采样精度，一共采样500个点
  double t = start_t;
  std::vector<ReferencePoint> ref_points;	 // 采样点保存
  const auto& spline = spline_solver_->spline();

  // Step B
  for (std::uint32_t i = 0; i < config_.num_of_total_points() && t < end_t; ++i, t += resolution) {
    const double heading =  std::atan2(spline.DerivativeY(t), spline.DerivativeX(t));  // 采样点速度大小
    const double kappa = CurveMath::ComputeCurvature(		// 采样点曲率，弧线越直曲率越小；弧线越弯，曲率越大
        spline.DerivativeX(t), spline.SecondDerivativeX(t),
        spline.DerivativeY(t), spline.SecondDerivativeY(t));
    const double dkappa = CurveMath::ComputeCurvatureDerivative(   // 曲率导数，描述曲率变化
        spline.DerivativeX(t), spline.SecondDerivativeX(t),
        spline.ThirdDerivativeX(t), spline.DerivativeY(t),
        spline.SecondDerivativeY(t), spline.ThirdDerivativeY(t));

    std::pair<double, double> xy = spline(t); // 求解累积距离为t时曲线的坐标，也是相对与第一个点的偏移坐标
    xy.first += ref_x_;		// 加上第一个anchor point的世界系坐标，变成世界系坐标
    xy.second += ref_y_;
    common::SLPoint ref_sl_point;
    if (!raw_reference_line.XYToSL({xy.first, xy.second}, &ref_sl_point)) {
      return false;
    }
    if (ref_sl_point.s() < -kEpsilon ||
        ref_sl_point.s() > raw_reference_line.Length()) {
      continue;
    }
    ref_sl_point.set_s(std::max(ref_sl_point.s(), 0.0));
       // 将点封装成ReferencePoint，并加入向量
    ReferencePoint rlp = raw_reference_line.GetReferencePoint(ref_sl_point.s());
    auto new_lane_waypoints = rlp.lane_waypoints();
    for (auto& lane_waypoint : new_lane_waypoints) {
      lane_waypoint.l = ref_sl_point.l();
    }
    ref_points.emplace_back(ReferencePoint(
        hdmap::MapPathPoint(common::math::Vec2d(xy.first, xy.second), heading,
                            new_lane_waypoints),
        kappa, dkappa));
  }
   // 去除过近的冗余点，最终将vector<ReferencePoint>封装成ReferenceLine
  ReferencePoint::RemoveDuplicates(&ref_points);
  if (ref_points.size() < 2) {
    AERROR << "Fail to generate smoothed reference line.";
    return false;
  }
  *smoothed_reference_line = ReferenceLine(ref_points);
  return true;
}


上述平滑ReferenceLine的生成过程其实也比较简单，分别采集每个点，计算坐标，速度，曲率，曲率导数等信息；然后修正采样点的累积距离便可以得到平滑后参考线的采样点。最后对所有的采样点封装变成平滑的参考线。

至此，knots分段与二次规划进行参考线平滑阶段已经全部完成，可以看到平滑参考线其实依旧是一个采样的过程，为什么不对原始的参考线直接进行更细粒度的采样(与anchor point和knots采样一样)？这里的理解是使用平滑参考线可以实时计算参考线上每个点的速度，曲率，曲率导数等信息，可以进一步描述车辆的运动状态信息。例如，车辆在平坦或者弯道部分(查询kappa)？车辆正要进入弯道还是脱离弯道(查询dkappa)？

在得到平滑参考线以后一定要做一个校验：对平滑参考线SmoothReferenceLine上的点(可以采样，比如累积距离每10m采样)，投影到原始参考线RawReferenceLine的距离为l，l不能太大，否则两条参考线有较大的偏移。这也是IsReferenceLineSmoothValid的工作。


modules/planning/reference_line/reference_line_provider.cc


bool ReferenceLineProvider::IsReferenceLineSmoothValid(
    const ReferenceLine &raw, const ReferenceLine &smoothed) const {
  constexpr double kReferenceLineDiffCheckStep = 10.0;
  for (double s = 0.0; s < smoothed.Length();
       s += kReferenceLineDiffCheckStep) {
    auto xy_new = smoothed.GetReferencePoint(s);

    common::SLPoint sl_new;
    if (!raw.XYToSL(xy_new, &sl_new)) {
      AERROR << "Fail to change xy point on smoothed reference line to sl "
                "point respect to raw reference line.";
      return false;
    }

    const double diff = std::fabs(sl_new.l());
    if (diff > FLAGS_smoothed_reference_line_max_diff) {
      AERROR << "Fail to provide reference line because too large diff "
                "between smoothed and raw reference lines. diff: "
             << diff;
      return false;
    }
  }
  return true;
}






9. //功能2：平滑参考线的拼接

平滑参考线拼接是针对不同时刻的RawReference，如果两条原始的RawReference是相连并且有覆盖的，那么可以不需要重新去进行平滑，只要直接使用上时刻的平滑参考线，或者仅仅平滑部分anchor point即可。

例如上时刻得到的平滑参考线reference_prev，这时刻由RouteSegments得到的原始费平滑参考线reference_current。由于RouteSegments生成有一个look_forward_distance的前向查询距离，所以这时候车辆的位置很可能还在前一时刻的平滑参考线reference_prev，这时候就可以复用上时刻的参考线信息，下面我们直接从代码来理解参考线拼接的流程和逻辑。

modules/planning/reference_line/reference_line_provider.cc


bool ReferenceLineProvider::CreateReferenceLine(std::list<ReferenceLine> *reference_lines,  std::list<hdmap::RouteSegments> *segments) {
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
	    if (pnc_map_->IsNewRouting(routing)) {
	      is_new_routing = true;
	      if (!pnc_map_->UpdateRoutingResponse(routing)) {
	        AERROR << "Failed to update routing in pnc map";
	        return false;
	      }
	    }
	  }

	  if (!CreateRouteSegments(vehicle_state, segments)) {
	    AERROR << "Failed to create reference line from routing";
	    return false;
	  }
	  // A. 参考线平滑，条件enable_reference_line_stitching设置为False，也就是不允许参考线拼接操作
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
	  } else {  // stitching reference line  // B. 允许参考线拼接
	    for (auto iter = segments->begin(); iter != segments->end();) {
	      reference_lines->emplace_back();
	      if (!ExtendReferenceLine(vehicle_state, &(*iter), &reference_lines->back())) {
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


// 下面我们将深入ExtendReferenceLine参考线扩展/拼接函数，查看逻辑，通过代码我们将逻辑整理如下：
	bool ReferenceLineProvider::ExtendReferenceLine(const VehicleState &state,  RouteSegments *segments,  ReferenceLine *reference_line) {

		// Case A
		// 1. 根据历史缓存信息，查询当前RouteSegments是否在某条(Smoothed)ReferenceLine上，如果不是就直接进行平滑参考线操作
  RouteSegments segment_properties;
  segment_properties.SetProperties(*segments);
  auto prev_segment = route_segments_.begin();
  auto prev_ref = reference_lines_.begin();
  while (prev_segment != route_segments_.end()) {
    if (prev_segment->IsConnectedSegment(*segments)) {  //查询路径段prev_segment是否连接到segments整个路径上的函数IsConnectedSegment其实很简单，无非以下四种情况：

																//segments整个路径的起点在prev_segment段内
																//segments整个路径的终点在prev_segment段内
																//prev_segment路径段的起点落在segments路径上
																//prev_segment路径段的终点落在segments路径上
      break;
    }
    ++prev_segment;
    ++prev_ref;
  }
  if (prev_segment == route_segments_.end()) {
    if (!route_segments_.empty() && segments->IsOnSegment()) {
      AWARN << "Current route segment is not connected with previous route "
               "segment";
    }
    return SmoothRouteSegment(*segments, reference_line);
  }


// Case B
  // 2. 如果在同一个平滑参考线(历史平滑参考线)上，计算车辆当前位置和历史平滑参考线终点的距离，如果距离超过了阈值，则可以复用这条历史参考线；否则长度不够需要拼接。
  common::SLPoint sl_point;
  Vec2d vec2d(state.x(), state.y());
  LaneWaypoint waypoint;
  if (!prev_segment->GetProjection(vec2d, &sl_point, &waypoint)) {   // 计算车辆当前位置在历史平滑参考线上的位置
    AWARN << "Vehicle current point: " << vec2d.DebugString()
          << " not on previous reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  const double prev_segment_length = RouteSegments::Length(*prev_segment);  // 历史平滑参考线总长度
  const double remain_s = prev_segment_length - sl_point.s();	 // 历史平滑参考线前方剩下的距离
  const double look_forward_required_distance = PncMap::LookForwardDistance(state.linear_velocity());  // 前向查询距离
  if (remain_s > look_forward_required_distance) {		// 如果剩下的距离足够长，那么直接复用这条历史平滑参考线
    *segments = *prev_segment;
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Reference line remain " << remain_s
           << ", which is more than required " << look_forward_required_distance
           << " and no need to extend";
    return true;
  }


  // Case C
  // 3 如果2种情况历史参考线遗留长度不够，那么久需要先对RouteSegments进行扩展，这部分在Pnc Map后接车道处理中有相关介绍。如果扩展失败直接进行平滑操作；如果扩展以后长度仍然不够，说明死路没有后继车道，只能复用历史平滑参考线。
  double future_start_s =
      std::max(sl_point.s(), prev_segment_length -
                                 FLAGS_reference_line_stitch_overlap_distance);
  double future_end_s =
      prev_segment_length + FLAGS_look_forward_extend_distance;  // 向后额外扩展look_forward_extend_distance的距离，默认50m
  RouteSegments shifted_segments;
  std::unique_lock<std::mutex> lock(pnc_map_mutex_);
  if (!pnc_map_->ExtendSegments(*prev_segment, future_start_s, future_end_s,
                                &shifted_segments)) {
    lock.unlock();
    AERROR << "Failed to shift route segments forward";
    return SmoothRouteSegment(*segments, reference_line);	 // C.1 扩展操作失败，直接对新的RouteSegments进行平滑得到平滑参考线
  }
  lock.unlock();
  if (prev_segment->IsWaypointOnSegment(shifted_segments.LastWaypoint())) {
    *segments = *prev_segment;	 // C.2 扩展操作成功，但是扩招以后长度没有太大变化，死路，直接使用历史平滑参考线
    segments->SetProperties(segment_properties);
    *reference_line = *prev_ref;
    ADEBUG << "Could not further extend reference line";
    return true;
  }

  // Case D 
  // 4如果3情况下扩展成功，并且额外增加了一定长度，得到了新的Path(也即新的RouteSegments)，接下来对新的路径进行平滑然后与历史平滑参考线进行拼接，就可以得到一条更长的平滑参考线。
  hdmap::Path path(shifted_segments);
  ReferenceLine new_ref(path);
  if (!SmoothPrefixedReferenceLine(*prev_ref, new_ref, reference_line)) {   // SmoothPrefixedReferenceLine过程和普通的拼接其实没多大差异
    AWARN << "Failed to smooth forward shifted reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!reference_line->Stitch(*prev_ref)) {  // 两条平滑车道线拼接
    AWARN << "Failed to stitch reference line";
    return SmoothRouteSegment(*segments, reference_line);
  }
  if (!shifted_segments.Stitch(*prev_segment)) {   // 两条平滑车道线对应的RouteSegments拼接
    AWARN << "Failed to stitch route segments";
    return SmoothRouteSegment(*segments, reference_line);
  }
  // Case E 
  // 5. 当在4完成参考线的拼接以后，就可以得到一条更长的参考线，前向查询距离经过限制不会超出要求，但是随着车辆的前进，车后参考线的长度会变得很大，所以最后一步就是对车后的参考线进行收缩，保证车辆前后都有合理长度的参考线
  *segments = shifted_segments;
  segments->SetProperties(segment_properties);
  common::SLPoint sl;
  if (!reference_line->XYToSL(vec2d, &sl)) {
    AWARN << "Failed to project point: " << vec2d.DebugString()
          << " to stitched reference line";
  }
  return Shrink(sl, reference_line, segments);
}



拼接过程也是非常简单，我们可以直接看Apollo给出的定义

Stitch current reference line with the other reference line
The stitching strategy is to use current reference points as much as
possible. The following two examples show two successful stitch cases.
Example 1
this: |--------A-----x-----B------|
other: |-----C------x--------D-------|
Result: |------A-----x-----B------x--------D-------|
In the above example, A-B is current reference line, and C-D is the other
reference line. If part B and part C matches, we update current reference
line to A-B-D.
Example 2
this: |-----A------x--------B-------|
other: |--------C-----x-----D------|
Result: |--------C-----x-----A------x--------B-------|
In the above example, A-B is current reference line, and C-D is the other
reference line. If part A and part D matches, we update current reference
line to C-A-B.
@return false if these two reference line cannot be stitched



收缩参考线ReferenceLine的操作很简单，就是删除查询距离1.5倍之后的ReferencePoint即可。

// modules/planning/reference_line/reference_line.cc
bool ReferenceLine::Shrink(const double s, double look_backward,
                           double look_forward) {
  const auto& accumulated_s = map_path_.accumulated_s();
  size_t start_index = 0;
  if (s > look_backward) {  // 查询收缩下界
    auto it_lower = std::lower_bound(accumulated_s.begin(), accumulated_s.end(),
                                     s - look_backward);
    start_index = std::distance(accumulated_s.begin(), it_lower);
  }
  size_t end_index = reference_points_.size();
  if (s + look_forward < Length()) {  // 查询收缩上界
    auto start_it = accumulated_s.begin();
    std::advance(start_it, start_index);
    auto it_higher =
        std::upper_bound(start_it, accumulated_s.end(), s + look_forward);
    end_index = std::distance(accumulated_s.begin(), it_higher);
  }

  //  // 删除上界和下界以外的点，完成收缩
  reference_points_.erase(reference_points_.begin() + end_index,
                          reference_points_.end());
  reference_points_.erase(reference_points_.begin(),
                          reference_points_.begin() + start_index);
  if (reference_points_.size() < 2) {
    AERROR << "Too few reference points after shrinking.";
    return false;
  }
  map_path_ = MapPath(std::move(std::vector<hdmap::MapPathPoint>(
      reference_points_.begin(), reference_points_.end())));
  return true;
}


注意在ReferenceLineProvider::ExtendReferenceLine中不论是ReferenceLine还是RouteSegments的Shrink函数，look_forward参数都设置为无穷大，所以只是后向收缩，前向不收缩