#include "CyberWriterImu.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/localization/proto/imu.pb.h"

class Imp
{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::localization::CorrectedImu>> mWriter;
};
CyberWriterImu::CyberWriterImu():mImp(std::make_shared<Imp>())
{
	apollo::cyber::Init("apollo_publish_imu");
    auto node = apollo::cyber::CreateNode("writer");
    mImp->mWriter = node->CreateWriter<apollo::localization::CorrectedImu>("/apollo/sensor/gnss/corrected_imu");
}

void CyberWriterImu::publish(
	float angVelX, float angVelY, float angVelZ,
	float linAccX, float linAccY, float linAccZ) const
{
	auto ts = apollo::cyber::Time::Now().ToSecond();
	auto imu = std::make_shared<apollo::localization::CorrectedImu>();
	imu->mutable_header()->set_module_name("gnss");
	imu->mutable_header()->set_timestamp_sec(ts);
	imu->mutable_imu()->mutable_angular_velocity()->set_x(angVelX);
	imu->mutable_imu()->mutable_angular_velocity()->set_y(angVelY);
	imu->mutable_imu()->mutable_angular_velocity()->set_z(angVelZ);
	imu->mutable_imu()->mutable_linear_acceleration()->set_x(linAccX);
	imu->mutable_imu()->mutable_linear_acceleration()->set_y(linAccY);
	imu->mutable_imu()->mutable_linear_acceleration()->set_z(linAccZ);
	mImp->mWriter->Write(imu);
}
