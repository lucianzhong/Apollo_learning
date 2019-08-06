#pragma once
#include <memory>

#define WAYPOINT_MAX_SIZE 1024
class Imp;
class CyberWriterRoutingReq
{
public:
	struct WayPoint
  	{
    	double x;
    	double y;
    	double headingx;
    	double headingy;
    	double headingz;
    	double headingw;
    	double speed;
    	double accel;
    	double time_interval;
    	
	};
	
	
	CyberWriterRoutingReq();
	void publish(int sequence) const;
	unsigned mWayPointSize;
	unsigned switchControlIndex{0};
	WayPoint mWayPointList[WAYPOINT_MAX_SIZE];

private:
	std::shared_ptr<Imp> mImp;
};