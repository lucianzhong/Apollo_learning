
// Apollo 3.0
1.  apollo/modules/planning/tasks/traffic_decider/front_vehicle.cc

 if (enter_sidepass_mode) {
  sidepass_status->set_status(SidePassStatus::SIDEPASS);
  sidepass_status->set_pass_obstacle_id(passable_obstacle_id);
  sidepass_status->clear_wait_start_time();
  sidepass_status->set_pass_side(side);   // side知识左超车还是右超车。取值为ObjectSidePass::RIGHT或ObjectSidePass::LEFT
}



// 
TopoCreator的源码位于modules/routing/topo_creator/
配置文件（routing_config.pb.txt）中的值的调整将影响这里生成的Topo地图的计算代价
int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  apollo::routing::RoutingConfig routing_conf;

  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_routing_conf_file,
                                               &routing_conf))
      << "Unable to load routing conf file: " + FLAGS_routing_conf_file;

  AINFO << "Conf file: " << FLAGS_routing_conf_file << " is loaded.";

  const auto base_map = apollo::hdmap::BaseMapFile();
  const auto routing_map = apollo::hdmap::RoutingMapFile();

  apollo::routing::GraphCreator creator(base_map, routing_map, routing_conf);
  CHECK(creator.Create()) << "Create routing topo failed!";

  AINFO << "Create routing topo successfully from " << base_map << " to "
        << routing_map;
  return 0;
}

// apollo/modules/routing/proto/routing_config.proto

message RoutingConfig {
  optional double base_speed = 1;  // base speed for node creator [m/s]
  optional double left_turn_penalty =
      2;  // left turn penalty for node creater [m]
  optional double right_turn_penalty =
      3;                              // right turn penalty for node creater [m]
  optional double uturn_penalty = 4;  // left turn penalty for node creater [m]
  optional double change_penalty = 5;  // change penalty for edge creater [m]
  optional double base_changing_length =
      6;  // base change length penalty for edge creater [m]
}
