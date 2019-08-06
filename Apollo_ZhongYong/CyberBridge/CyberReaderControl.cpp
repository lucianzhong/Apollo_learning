#include "CyberReaderControl.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/control/proto/control_cmd.pb.h"

class Imp
{
public:
    std::shared_ptr<apollo::cyber::Reader<apollo::control::ControlCommand>> mReader;
};

CyberReaderControl::CyberReaderControl():mImp(std::make_shared<Imp>())
{
	apollo::cyber::Init("apollo_subscribe_control");
	auto node = apollo::cyber::CreateNode("listener");
	mImp->mReader = node->CreateReader<apollo::control::ControlCommand>(
		"/apollo/control",
		[this](const std::shared_ptr<const apollo::control::ControlCommand> &msg) {
			mThrottle = msg->throttle();
			mSteering = msg->steering_target();
			mBrake = msg->brake();
		}
	);
}

void CyberReaderControl::spinOnce()
{
	mImp->mReader->Observe();
}