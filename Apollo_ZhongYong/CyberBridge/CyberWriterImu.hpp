#pragma once
#include <memory>

class Imp;
class CyberWriterImu
{
  public:
	 CyberWriterImu();
	 void publish(
		float angVelX, float angVelY, float angVelZ,
		float linAccX, float linAccY, float linAccZ) const;

  private:
	std::shared_ptr<Imp> mImp;
};