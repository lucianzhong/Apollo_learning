 
1. 
   提供当前无人车的各类状态信息，车辆状态提供器的输入包括定位数据Localization以及底盘信息Chassis
   // apollo/modules/common/vehicle_state/vehicle_state_provider.cc
   // apollo/modules/common/vehicle_state/vehicle_state_provider.h
    class VehicleStateProvider {
	  ...
	private:
	  common::VehicleState vehicle_state_;				// 车辆状态信息vehicle_state_
	  localization::LocalizationEstimate original_localization_;	// 定位信息original_localization_
	  // 这两个成员对应的类型都是以protobuf的形式定义，第二项original_localization_其实并不是主要成员，仅仅是记录上一次更新的Localization而已。第一项vehicle_state_才是完整保存车辆状态的载体
	}

2.  // modules/common/vehicle_state/proto/vehicle_state.proto

	message VehicleState {
	  optional double x = 1 [default =0.0];		// 车辆世界ENU坐标系x坐标
	  optional double y = 2 [default =0.0];		// 车辆世界ENU坐标系y坐标
	  optional double z = 3 [default =0.0];		// 车辆世界ENU坐标系z坐标
	  optional double timestamp = 4 [default =0.0];	// 时间戳信息
	  optional double roll = 5 [default =0.0];		// 车辆姿态相对于世界坐标系x轴旋转角度
	  optional double pitch = 6 [default =0.0];		// 车辆姿态相对于世界坐标系y轴旋转角度
	  optional double yaw = 7 [default =0.0];		 // 车辆姿态相对于世界坐标系z轴旋转角度
	  optional double heading = 8 [default =0.0];		// 车辆速度方向
	  optional double kappa = 9 [default =0.0];				 // 车辆半径倒数1/R
	  optional double linear_velocity = 10 [default =0.0];		// 车辆线速度
	  optional double angular_velocity = 11 [default =0.0];			// 车辆角速度
	  optional double linear_acceleration = 12 [default =0.0];		 // 车辆线加速度
	  optional apollo.canbus.Chassis.GearPosition gear = 13;		// 车辆齿轮状态，包含前进、倒车。停车、低速等状态
	  optional apollo.canbus.Chassis.DrivingMode driving_mode = 14;		// 驾驶状态，包含手动驾驶、自动驾驶、转向、刹车与油门等状态
	  optional apollo.localization.Pose pose = 15;			  // 车辆姿态，包含坐标，局部到世界坐标系变换矩阵，线速度(矢量)，线加速度(矢量)等信息。
	}



3.  apollo/modules/common/vehicle_state/vehicle_state_provider.cc
	
	Status VehicleStateProvider::Update( const localization::LocalizationEstimate &localization, const canbus::Chassis &chassis ) //车辆状态更新函数VehicleStateProvider::Update
	math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const		//根据当前车辆状态，给定一个时间t，来预测t时间后的车辆状态，注意，这个时间t必须是短时间内的预测，如果t过大，预测将不会准确(因为预测前提是假定这个时间段内，车辆的速度、角速度等指标不变)



4.  车辆状态更新函数( VehicleStateProvider::Update)
	在函数VehicleStateProvider::ConstructExceptLinearVelocity中实现，即将localization中的信息抽取出来填充到vehicle_state_中
	1.设置车辆姿态信息
	在函数VehicleStateProvider::ConstructExceptLinearVelocity中实现，即将localization中的信息抽取出来填充到vehicle_state_中，设置的内容包含:
	车辆坐标(x,y,z)
	车辆速度方向heading
	车辆xy平面角速度angular_velocity
	车辆半径倒数kappa
	车辆xyz方向旋转角roll, pitch, yaw
	
	2.设置车辆速度与时间戳信息
	车辆速度linear_velocity
	时间戳timestamp
	
	3.设置车辆底盘信息
	齿轮信息gear
	驾驶模式driving_mode

5. 车辆未来时刻预测函数( VehicleStateProvider::EstimateFuturePosition)

   预测未来短时间内车辆的位置与姿态信息，其实这是对车辆状态的一种修正。主要是在使用VehicleStateProvider::Update更新状态以后，当需要使用vehicle_state_时，经过了一定的时间差t，外加车辆运动就造成车辆状态的偏差，所以可以使用该函数来对车辆状态进行一次修正，
   但是修正必须要满足一个条件：当前时间与update时间(vehicle_state_.timestamp)与之间的时间差必须小于一个阈值，代码中设置为20ms，可以对这个时间差进行车辆姿态矫正。

	// 以下是一个矫正的例子：
	if (FLAGS_estimate_current_vehicle_state && start_timestamp - vehicle_state.timestamp() < 0.020) {
	    auto future_xy = VehicleStateProvider::instance()->EstimateFuturePosition( start_timestamp - vehicle_state.timestamp() );
	    vehicle_state.set_x( future_xy.x() );
	    vehicle_state.set_y( future_xy.y() );
	    vehicle_state.set_timestamp( start_timestamp );
	  }

	math::Vec2d VehicleStateProvider::EstimateFuturePosition(const double t) const {
		  Eigen::Vector3d vec_distance(0.0, 0.0, 0.0);

		   // Step 1. set vehicle velocity
		  double v = vehicle_state_.linear_velocity();
		  //查看齿轮状态，如果如果是倒车，速度就设置为负
		  if (vehicle_state_.gear() == canbus::Chassis::GEAR_REVERSE) {
		    v = -vehicle_state_.linear_velocity();
		  }

		  // Predict distance travel vector
		  if (std::fabs(vehicle_state_.angular_velocity()) < 0.0001) {	//vehicle_state_.angular_velocity()是带有正负性的，逆时针为正；顺时针为负，这样也就可以理解上述公式的符号
		    vec_distance[0] = 0.0;
		    vec_distance[1] = v * t;
		  } else {
		    vec_distance[0] = -v / vehicle_state_.angular_velocity() *  (1.0 - std::cos(vehicle_state_.angular_velocity() * t));   // x_new
		    vec_distance[1] = std::sin(vehicle_state_.angular_velocity() * t) * v / vehicle_state_.angular_velocity();			// y_new
		  }

		  // If we have rotation information, take it into consideration.
		  if (vehicle_state_.pose().has_orientation()) {
		    const auto &orientation = vehicle_state_.pose().orientation();

		    Eigen::Quaternion<double> quaternion(orientation.qw(), orientation.qx(), orientation.qy(), orientation.qz());

		    Eigen::Vector3d pos_vec(vehicle_state_.x(), vehicle_state_.y(), vehicle_state_.z());

		    auto future_pos_3d = quaternion.toRotationMatrix() * vec_distance + pos_vec;
		    return math::Vec2d(future_pos_3d[0], future_pos_3d[1]);
		  }

		  // If no valid rotation information provided from localization,
		  // return the estimated future position without rotation.
		  return math::Vec2d( vec_distance[0] + vehicle_state_.x(), vec_distance[1] + vehicle_state_.y() );  // (x', y')
		}
