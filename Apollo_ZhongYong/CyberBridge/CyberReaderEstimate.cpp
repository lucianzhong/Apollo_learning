#include "CyberReaderEstimate.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/localization/proto/localization.pb.h"

class Imp
{
public:
    std::shared_ptr<apollo::cyber::Reader<apollo::localization::LocalizationEstimate2>> mReader;
};

CyberReaderEstimate::CyberReaderEstimate():mImp(std::make_shared<Imp>())
{
	apollo::cyber::Init("apollo_subscribe_trajectory");
  	auto node = apollo::cyber::CreateNode("listener");
	mImp->mReader = node->CreateReader<apollo::localization::LocalizationEstimate2>(
		"/apollo/localization2/pose",
		[this](const std::shared_ptr<const apollo::localization::LocalizationEstimate2> &msg) {
			for (int i = 0; i < msg->trajectory_point_size() && i < 100; i++)
			{
				const apollo::localization::Pose &pose = msg->pose();
				mPositionX = pose.position().x();
				mPositionY = pose.position().y();
				mPositionZ = pose.position().z();
				mOrientationX = pose.orientation().qx();
				mOrientationY = pose.orientation().qy();
				mOrientationZ = pose.orientation().qz();
				mOrientationW = pose.orientation().qw();
			}
		}
	);	
}

void CyberReaderEstimate::spinOnce()
{
	mImp->mReader->Observe();
}