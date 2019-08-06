#pragma once
#include <memory>

class Imp;
class CyberReaderControl
{
public:
		CyberReaderControl();
		
		void spinOnce();
		float mThrottle{0.0};
		float mSteering{0.0};
		float mBrake{0.0};

private:
		std::shared_ptr<Imp> mImp;
};
