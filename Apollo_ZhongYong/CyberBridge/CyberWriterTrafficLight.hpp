//
// Created By: Huiyong.Men 2018/06/29
//

#pragma once
#include <memory>

class Imp;
class CyberWriterTrafficLight
{
public:
	  CyberWriterTrafficLight();
		void publish(int sequence, char *id[], float confidence[], int color[], int size) const;

private:
		std::shared_ptr<Imp> mImp;
};
