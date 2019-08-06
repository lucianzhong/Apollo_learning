#include "CyberReaderTrajectory.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/planning/proto/planning.pb.h"
#include "modules/localization/proto/imu.pb.h"

class Imp
{
public:
    std::shared_ptr<apollo::cyber::Reader<apollo::planning::ADCTrajectory>> mReader;
};

CyberReaderTrajectory::CyberReaderTrajectory():mImp(std::make_shared<Imp>())
{
	mTrajectorySize = 0;

	apollo::cyber::Init("apollo_subscribe_trajectory");
  	auto node = apollo::cyber::CreateNode("listener");
  	mImp->mReader =
      node->CreateReader<apollo::planning::ADCTrajectory>(
          "/apollo/planning", 
		  [this](const std::shared_ptr<apollo::planning::ADCTrajectory>& msg) {
			mTrajectorySize = 0;
			for (int i = 0; i < msg->trajectory_point_size() && i < TRAJECTORY_MAX_SIZE; i++)
			{
				const apollo::common::TrajectoryPoint &traj_point = msg->trajectory_point(i);
				auto point = traj_point.path_point();
				mTrajectoryList[i].x = point.x();
				mTrajectoryList[i].y = point.y();
				mTrajectoryList[i].z = point.z();
				mTrajectoryList[i].t = point.theta();
				mTrajectoryList[i].k = point.kappa();
				mTrajectoryList[i].rt = traj_point.relative_time();
				mTrajectoryList[i].a = traj_point.a();
				mTrajectoryList[i].v = traj_point.v();
				mTrajectorySize++;
			}
			mTrajectoryTimestamp = msg->header().timestamp_sec();
		});


}

void CyberReaderTrajectory::spinOnce()
{
	mImp->mReader->Observe();
	mCurrentTimestamp = apollo::cyber::Time::Now().ToSecond();
}
