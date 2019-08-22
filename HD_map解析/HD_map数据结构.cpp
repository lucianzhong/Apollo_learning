

1.	// Apollo地图格式对OpenDRIVE都有哪些改动，改动的原因或初衷是什么，改动有什么优势？

	百度的Apollo项目主要是依赖地图，配上各类传感器（加速度传感器，GPS，激光雷达，摄像头等）来实现定位，导航，决策，避障，遵守交规等行为。
	据我所知，高精地图对perception感知模块的用途是很大，比如说要用CNN分割周围车辆行人物体，hdmap能迅速给出需要关注的区域大小，可以删除不需要关注的绿化带和周围建筑等。
	另外2.0版本的信号灯识别，就依赖于地图里信号灯的位置和轮廓给定数据，然后投射到图像对应位置，能比所谓的纯图像识别要精确可靠。Apollo的地图是依照opendrive这个开源格式改出来的

	OpenDRIVE本身设计面向的应用是仿真器，自动驾驶需要更多的信息OpenDRIVE并没有完全提供，所以我们对OpenDRIVE的标准做了部分改动和扩展。
	主要改动和扩展了以下几个方面：
		一是地图元素形状的表述方式。以车道边界为例，标准OpenDRIVE采用基于Reference Line的曲线方程和偏移的方式来表达边界形状，而Apollo OpenDrive采用绝对坐标序列的方式描述边界形状；
		二是元素类型的扩展。例如新增了对于禁停区、人行横道、减速带等元素的独立描述；
		三是扩展了对于元素之间相互关系的描述。比如新增了junction与junction内元素的关联关系等；除此之外还有一些配合无人驾驶算法的扩展，比如增加了车道中心线到真实道路边界的距离、停止线与红绿灯的关联关系等。改动和扩展后的规格在实现上更加的简单，同时也兼顾了无人驾驶的应用需求。


	// 格式
	百度高精地图数据格式采用（XML）文件格式的数据组织方式，是基于国际通用的OpenDrive规范，并根据百度自动驾驶业务需求拓展修改而成

	// 坐标
	百度高精地图坐标采用WGS84经纬度坐标表示。WGS84为一种大地坐标系，也是目前广泛使用的GPS全球卫星定位系统使用的坐标系。

 	// 车道
 	道路的reference line 存储在ID为0的车道中，其他车道只存储当前车道的一个边界。例如，reference line右侧的车道只存储车道的右侧边界。

   车道总数目没有限制。Reference line 自身必须为 Lane 0。
 	

	// 路口区域（Juction）
   基本的原理比较简单，路口区域用Junction结构表达。在Junction内，incoming Road通过Connecting Roads与out-going道路相连。




2. // 一段道路的相关自动驾驶地图可以放置在如下结构的目录中：

	sunnyvale_big_loop
	├── background.jpg
	├── background.png
	├── base_map.bin
	├── base_map.lb1
	├── base_map.txt
	├── base_map.xml               # Defined by FLAGS_base_map_filename
	├── default_end_way_point.txt  # Defined by FLAGS_end_way_point_filename
	├── grid_map
	├── local_map
	├── map.json
	├── routing_map.bin            # Defined by FLAGS_routing_map_filename
	├── routing_map.txt
	├── sim_map.bin                # Defined by FLAGS_sim_map_filename
	├── sim_map.txt
	└── speed_control.pb.txt

	modules/common/configs/config_gflags.cc
	DEFINE_string(base_map_filename, "base_map.bin|base_map.xml|base_map.txt|base_map.xodr|base_map.odr", "Base map files in the map_dir, search in order."); //可以将可用地图文件名指定为备选列表,然后Apollo会找到第一个可用的文件加载


3.
	// 文件路径:
	/modules/map/hdmap/adapter/opendrive_adapter.*
	/modules/map/hdmap/adapter/proto_organizer.*
	/modules/map/proto/

	//流水线:
	1. 将地图文件作为XML文件载入
	2. 抽取头部信息（见下文头结构）
	3. 抽取路信息:
	     		查看是否是交叉口id
				抽取车道信息
				抽取物体信息（停止线，人行横道，禁停区，减速带）
				抽取信号标示（信号灯，停车标示，让车标示）
	4. 抽取交叉口信息（特别的是，Apollo格式和OpenDrive不一样）
														outline
														objectOverlapGroup: objectReference: id


	5. 通过分析路和交叉口，遍历所有道，抽取重叠元素
												找重叠的物体（4类）
												找重叠的信号标示（3类）
												找重叠的车道(起点，终点，是否合并)
	6. 本阶段地图准备就绪

		注意:
		通过地图东和西的最远位置来判断时区是否一致，不支持跨时区地图。
		通过对地图geoReference字段分析，转换成UTM投影(60等分，每个等分投射成二维直角坐标系)。




4. 
//系统内格式
	不管原始数据格式为什么，在Apollo内部的数据地图的格式为proto。以下为Apollo高精地图的对象定义：

	modules/map/proto/map.proto

	// This message defines how we project the ellipsoidal Earth surface to a plane.
	message Projection {
	  // PROJ.4 setting:
	  // "+proj=tmerc +lat_0={origin.lat} +lon_0={origin.lon} +k={scale_factor}
	  // +ellps=WGS84 +no_defs"
	  optional string proj = 1;
	}

	message Header {
	  optional bytes version = 1;		//地图数据库的版本
	  optional bytes date = 2;			  //地图数据库创建的时间
	  optional Projection projection = 3; //坐标系转换  //投射坐标系:"+proj=utm +zone=时区 +ellps=WGS84 +datum=WGS84 +units=m +no_defs";
	  optional bytes district = 4;			//地图数据库名称
	  optional bytes generation = 5;
	  optional bytes rev_major = 6;	//OpenDrive格式的主要版本号
	  optional bytes rev_minor = 7;	//OpenDrive格式的次要版本号
	  optional double left = 8;		//west，最小惯性参考系x值
	  optional double top = 9;		//north，最大惯性参考系y值
	  optional double right = 10;	//east，最大惯性参考系x值
	  optional double bottom = 11; 	//south，最小惯性参考系y值
	  optional bytes vendor = 12;	//地图厂商名称	
	}

	message Map {
	  optional Header header = 1;

	  repeated Crosswalk crosswalk = 2; // 人行道
	  repeated Junction junction = 3;   //路口区域
	  repeated Lane lane = 4;			//车道
	  repeated StopSign stop_sign = 5;  //停止线
	  repeated Signal signal = 6;		//信号灯
	  repeated YieldSign yield = 7;		//让路标志
	  repeated Overlap overlap = 8;		//重叠区域  //任何一对在地图上重合的东西，包括（车道，路口，人行横道）
	  repeated ClearArea clear_area = 9;  //禁停区域
	  repeated SpeedBump speed_bump = 10; //减速带
	  repeated Road road = 11;		//道路  //路，包含道路，物体等信息
	  repeated ParkingSpace parking_space = 12; //停车区域
	  repeated PNCJunction pnc_junction = 13;
	}

	其中有2个标志(StopSign，YieldSign)是美国才有的，后来查看了下知乎发现对应到国内是(停，让)，具体的含义都是一样，停车的意思是到路口先停止，看下有没有车，然后再开始启动，让车就是先让行，比如交汇路口，理应让直行的车辆先通过，然后再汇入道路。


	// 人行横道
	modules/map/proto/map_crosswalk.proto

	// Crosswalk is a place designated for pedestrians to cross a road.
	message Crosswalk {
	  optional Id id = 1;		 //编号
	  optional Polygon polygon = 2;  //多边形
	  repeated Id overlap_id = 3;    //重叠ID
	}


	modules/map/proto/map_junction.proto
	// A junction is the junction at-grade of two or more roads crossing.
	message Junction {
	  optional Id id = 1;		//编号
	  optional Polygon polygon = 2;  //多边形
	  repeated Id overlap_id = 3;	 //重叠id
	}



	// map.lane.proto 车道线
	modules/map/proto/map_lane.proto


	message LaneBoundaryType {
	  enum Type {
	    UNKNOWN = 0;
	    DOTTED_YELLOW = 1;
	    DOTTED_WHITE = 2;
	    SOLID_YELLOW = 3;
	    SOLID_WHITE = 4;
	    DOUBLE_YELLOW = 5;
	    CURB = 6;
	  };
	  // Offset relative to the starting point of boundary
	  optional double s = 1;
	  // support multiple types
	  repeated Type types = 2;
	}

	message LaneBoundary {
	  optional Curve curve = 1;

	  optional double length = 2;
	  // indicate whether the lane boundary exists in real world
	  optional bool virtual = 3;
	  // in ascending order of s
	  repeated LaneBoundaryType boundary_type = 4;
	}

	// Association between central point to closest boundary.
	message LaneSampleAssociation {
	  optional double s = 1;
	  optional double width = 2;
	}

	// A lane is part of a roadway, that is designated for use by a single line of
	// vehicles.
	// Most public roads (include highways) have more than two lanes.
	message Lane {
	  optional Id id = 1; //编号

	  // Central lane as reference trajectory, not necessary to be the geometry
	  // central.
	  optional Curve central_curve = 2;//中心曲线

	  // Lane boundary curve.
	  optional LaneBoundary left_boundary = 3; //左边界
	  optional LaneBoundary right_boundary = 4;//右边界

	  // in meters.
	  optional double length = 5;//长度

	  // Speed limit of the lane, in meters per second.
	  optional double speed_limit = 6;//速度限制

	  repeated Id overlap_id = 7;//重叠区域id

	  // All lanes can be driving into (or from).
	  repeated Id predecessor_id = 8;//前任id
	  repeated Id successor_id = 9; //继任者id

	  // Neighbor lanes on the same direction.
	  repeated Id left_neighbor_forward_lane_id = 10;//前面左边邻居id
	  repeated Id right_neighbor_forward_lane_id = 11;//前面右边邻居id

	  enum LaneType {//车道类型
	    NONE = 1;//无
	    CITY_DRIVING = 2;//城市道路
	    BIKING = 3; //自行车
	    SIDEWALK = 4;//人行道
	    PARKING = 5;//停车
	    SHOULDER = 6;
	  };
	  optional LaneType type = 12;//车道类型

	  enum LaneTurn {
	    NO_TURN = 1;//直行
	    LEFT_TURN = 2;//左转弯
	    RIGHT_TURN = 3;//右转弯
	    U_TURN = 4;//掉头
	  };
	  optional LaneTurn turn = 13; //转弯类型

	  repeated Id left_neighbor_reverse_lane_id = 14;//保留（后面？）左边邻居
	  repeated Id right_neighbor_reverse_lane_id = 15;//右边邻居

	  optional Id junction_id = 16;

	  // Association between central point to closest boundary.
	  repeated LaneSampleAssociation left_sample = 17;//中心点与最近左边界之间的关联
	  repeated LaneSampleAssociation right_sample = 18;//中心点与最近右边界之间的关联

	  enum LaneDirection {
	    FORWARD = 1;//前
	    BACKWARD = 2;//后，潮汐车道借用的情况？
	    BIDIRECTION = 3; //双向
	  }
	  optional LaneDirection direction = 19;//车道方向

	  // Association between central point to closest road boundary.
	  repeated LaneSampleAssociation left_road_sample = 20;//中心点与最近左路边界之间的关联
	  repeated LaneSampleAssociation right_road_sample = 21;//中心点与最近右路边界之间的关联

	  repeated Id self_reverse_lane_id = 22;
	}



// modules/map/proto/map_stop_sign.proto
/ A stop sign is a traffic sign to notify drivers that they must stop before
// proceeding.
message StopSign {

  optional Id id = 1; //编号

  repeated Curve stop_line = 2;  //停止线，Curve曲线应该是基础类型

  repeated Id overlap_id = 3;  //重叠id

  enum StopType {
    UNKNOWN = 0;   //未知
    ONE_WAY = 1;  //只有一车道可以停
    TWO_WAY = 2;
    THREE_WAY = 3;
    FOUR_WAY = 4;
    ALL_WAY = 5;
  };
  optional StopType type = 4;
}





	// 其中重点介绍车道与路口部分


	
	如前文所述，Apollo高精度地图OpenDrive采用绝对坐标序列的方式描述边界形状，依次为基础生成直线或类似直线的对象：

	modules/map/proto/map_geometry.proto

	// Polygon, not necessary convex.
	message Polygon {
	  repeated apollo.common.PointENU point = 1;
	}

	// Straight line segment.
	message LineSegment {
	  repeated apollo.common.PointENU point = 1;
	}

	// Generalization of a line.
	message CurveSegment {
	  oneof curve_type {
	    LineSegment line_segment = 1;
	  }
	  optional double s = 6;  // start position (s-coordinate)
	  optional apollo.common.PointENU start_position = 7;
	  optional double heading = 8;  // start orientation
	  optional double length = 9;
	}

	// An object similar to a line but that need not be straight.
	message Curve {
	  repeated CurveSegment segment = 1;
	}


	// 道路Road
	modules/map/proto/map_road.proto
	
	
// 车道Lane
modules/map/proto/map_lane.proto


// 路口Junction
modules/map/proto/map_pnc_junction.proto




// 信号灯
数据结构
message Subsignal {
  enum Type {
    UNKNOWN = 1;        //未知
    CIRCLE = 2;         //圆圈
    ARROW_LEFT = 3;     //向左箭头
    ARROW_FORWARD = 4;  //向前进箭头
    ARROW_RIGHT = 5;    //向右箭头
    ARROW_LEFT_AND_FORWARD = 6;      //向左前进箭头
    ARROW_RIGHT_AND_FORWARD = 7;     //向右前进箭头
    ARROW_U_TURN = 8;           //U型箭头
  };

  optional Id id = 1;       //编号
  optional Type type = 2;   //类型

  // Location of the center of the bulb. now no data support.
  // 中间灯泡的位置
  optional apollo.common.PointENU location = 3;
}
message Signal {
  enum Type {
    UNKNOWN = 1;    //未知
    MIX_2_HORIZONTAL = 2;   //水平2灯
    MIX_2_VERTICAL = 3;     //垂直2灯
    MIX_3_HORIZONTAL = 4;   //水平3灯
    MIX_3_VERTICAL = 5;     //垂直3灯
    SINGLE = 6;
  };

  optional Id id = 1;               //编号
  optional Polygon boundary = 2;    //轮廓
  repeated Subsignal subsignal = 3; //子信号信息
  // TODO: add orientation. now no data support.对应行驶方向，目前无数据
  repeated Id overlap_id = 4; //重叠id
  optional Type type = 5;   //信号灯外形类型，见Type
  // stop line
  repeated Curve stop_line = 6; //车辆停止线位置
}