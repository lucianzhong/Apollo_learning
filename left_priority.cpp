
// Apollo 3.0
1.  apollo/modules/planning/tasks/traffic_decider/front_vehicle.cc

 if (enter_sidepass_mode) {
  sidepass_status->set_status(SidePassStatus::SIDEPASS);
  sidepass_status->set_pass_obstacle_id(passable_obstacle_id);
  sidepass_status->clear_wait_start_time();
  sidepass_status->set_pass_side(side);   // side知识左超车还是右超车。取值为ObjectSidePass::RIGHT或ObjectSidePass::LEFT
}



