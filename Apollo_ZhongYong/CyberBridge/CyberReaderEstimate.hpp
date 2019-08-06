#pragma once
#include <memory>
class Imp;
class CyberReaderEstimate
{
public:
	 	CyberReaderEstimate();

		void spinOnce();
		float mPositionX{0.0};
		float mPositionY{0.0};
		float mPositionZ{0.0};
		float mOrientationX{0.0};
		float mOrientationY{0.0};
		float mOrientationZ{0.0};
		float mOrientationW{0.0};

private:
		std::shared_ptr<Imp> mImp;
};
