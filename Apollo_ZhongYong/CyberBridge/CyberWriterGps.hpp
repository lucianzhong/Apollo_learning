#pragma once
#include <memory>
class Imp;
class CyberWriterGps
{
public:
		CyberWriterGps();
		void publish(
		float posX, float posY, float posZ, float mVelocityX, float mVelocityY, float mVelocityZ,
		float oriX, float oriY, float oriZ, float oriW) const;

private:
	std::shared_ptr<Imp> mImp;
};