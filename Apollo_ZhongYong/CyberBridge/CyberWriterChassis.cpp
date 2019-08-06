#include "CyberWriterChassis.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/canbus/proto/chassis.pb.h"
class Imp{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::canbus::Chassis>> mWriter;
};
CyberWriterChassis::CyberWriterChassis():mImp(std::make_shared<Imp>())
{
	apollo::cyber::Init("apollo_publish_chassis");
	auto node = apollo::cyber::CreateNode("writer");
	mImp->mWriter = node->CreateWriter<apollo::canbus::Chassis>("/apollo/canbus/chassis");
}

void CyberWriterChassis::publish(float speedMps, float engineRpm, float throttleP, float steeringP, float brakeP, int drivingMode) const
{
	auto chassis = std::make_shared<apollo::canbus::Chassis>();
	auto ts = apollo::cyber::Time::Now().ToSecond();
	chassis->mutable_header()->set_timestamp_sec(ts);
	chassis->set_speed_mps(speedMps);
	chassis->set_engine_rpm(engineRpm);
	chassis->set_throttle_percentage(throttleP);
	chassis->set_steering_percentage(steeringP);
	chassis->set_brake_percentage(brakeP);
	chassis->set_driving_mode(apollo::canbus::Chassis_DrivingMode(drivingMode));
	mImp->mWriter->Write(chassis);
}
