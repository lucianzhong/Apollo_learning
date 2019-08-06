#pragma once
#include <memory>

class Imp;
class CyberWriterChassis
{
public:
	  CyberWriterChassis();
	  void publish(float speedMps, float engineRpm, float throttleP, float steeringP, float brakeP, int drivingMode) const;

private:
		std::shared_ptr<Imp> mImp;
};
