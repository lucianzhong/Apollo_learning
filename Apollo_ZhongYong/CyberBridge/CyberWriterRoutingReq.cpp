#include "CyberWriterRoutingReq.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/routing/proto/routing.pb.h"
class Imp
{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::routing::RoutingRequest>> mWriter;
};

CyberWriterRoutingReq::CyberWriterRoutingReq():mImp(std::make_shared<Imp>())
{
    mWayPointSize = 0;
    memset(mWayPointList, 0, sizeof(WayPoint) * WAYPOINT_MAX_SIZE);
    
    apollo::cyber::Init("apollo_routing_request");
    auto node = apollo::cyber::CreateNode("writer");
    
    // configure QoS for routing request writer
    apollo::cyber::proto::RoleAttributes routing_request_attr;
    routing_request_attr.set_channel_name("/apollo/routing_request");
    auto qos = routing_request_attr.mutable_qos_profile();
    // only keeps the last message in history
    qos->set_history(apollo::cyber::proto::QosHistoryPolicy::HISTORY_KEEP_LAST);
    // reliable transfer
    qos->set_reliability(
        apollo::cyber::proto::QosReliabilityPolicy::RELIABILITY_RELIABLE);
    // when writer find new readers, send all its history messsage
    qos->set_durability(
        apollo::cyber::proto::QosDurabilityPolicy::DURABILITY_TRANSIENT_LOCAL);
    mImp->mWriter =
        node->CreateWriter<apollo::routing::RoutingRequest>(routing_request_attr);

}

void CyberWriterRoutingReq::publish(int sequence) const
{
    auto ts = apollo::cyber::Time::Now().ToSecond();
    auto routing_request = std::make_shared<apollo::routing::RoutingRequest>();

    for (size_t i = 0; i < mWayPointSize; i++)
    {
        apollo::routing::LaneWaypoint *point = routing_request->add_waypoint();
        apollo::common::PointENU *pose = new apollo::common::PointENU();
        pose->set_x(mWayPointList[i].x);
        pose->set_y(mWayPointList[i].y);
        point->set_allocated_pose(pose);
        apollo::routing::LaneWaypointL2Control *pointcontrol = routing_request->add_waypoint_control();
        pointcontrol->set_headingx(mWayPointList[i].headingx);
        pointcontrol->set_headingy(mWayPointList[i].headingy);
        pointcontrol->set_headingz(mWayPointList[i].headingz);
        pointcontrol->set_headingw(mWayPointList[i].headingw);
        pointcontrol->set_speed(mWayPointList[i].speed);
        pointcontrol->set_accel(mWayPointList[i].accel);
        pointcontrol->set_time_interval(mWayPointList[i].time_interval);
    }
    routing_request->set_switchcontrol_waypoint_id(switchControlIndex);
    routing_request->mutable_header()->set_module_name("cyber_bridge_routing");
    routing_request->mutable_header()->set_timestamp_sec(ts);
    routing_request->mutable_header()->set_sequence_num(sequence);
    //AINFO << "Constructed RoutingRequest to be sent:\n"
    //    << routing_request->DebugString();
    mImp->mWriter->Write(routing_request);
}
