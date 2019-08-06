#pragma once
#include <memory>

#define TRAJECTORY_MAX_SIZE 150
class Imp;
class CyberReaderTrajectory
{
public:
	 	struct Trajectory
		{
			float x;
			float y;
			float z;
			float t;
			double rt;
			float k;
			float a;
			float v;
			float pad;
		};
		CyberReaderTrajectory();
		void spinOnce();
		Trajectory mTrajectoryList[TRAJECTORY_MAX_SIZE];
		int mTrajectorySize;
		double mTrajectoryTimestamp;
		double mCurrentTimestamp;

private:
		std::shared_ptr<Imp> mImp;
};
