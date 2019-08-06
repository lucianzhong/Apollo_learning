#include "CyberWriterGps.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/localization/proto/gps.pb.h"

class Imp
{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::localization::Gps>> mWriter;
};

CyberWriterGps::CyberWriterGps():mImp(std::make_shared<Imp>())
{
	apollo::cyber::Init("apollo_publish_gps");
    auto node = apollo::cyber::CreateNode("writer");
    mImp->mWriter = node->CreateWriter<apollo::localization::Gps>("/apollo/sensor/gnss/odometry");
}

void CyberWriterGps::publish(
	float posX, float posY, float posZ, float mVelocityX, float mVelocityY, float mVelocityZ,
	float oriX, float oriY, float oriZ, float oriW) const
{
	auto ts = apollo::cyber::Time::Now().ToSecond();

	auto gps = std::make_shared<apollo::localization::Gps>();
	gps->mutable_header()->set_timestamp_sec(ts);
	gps->mutable_localization()->mutable_position()->set_x(posX);
	gps->mutable_localization()->mutable_position()->set_y(posY);
	gps->mutable_localization()->mutable_position()->set_z(posZ);
	gps->mutable_localization()->mutable_linear_velocity()->set_x(mVelocityX);
	gps->mutable_localization()->mutable_linear_velocity()->set_y(mVelocityY);
	gps->mutable_localization()->mutable_linear_velocity()->set_z(mVelocityZ);
	gps->mutable_localization()->mutable_orientation()->set_qx(oriX);
	gps->mutable_localization()->mutable_orientation()->set_qy(oriY);
	gps->mutable_localization()->mutable_orientation()->set_qz(oriZ);
	gps->mutable_localization()->mutable_orientation()->set_qw(oriW);
	mImp->mWriter->Write(gps);
}