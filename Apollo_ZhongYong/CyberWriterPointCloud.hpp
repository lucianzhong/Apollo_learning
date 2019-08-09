//
// Created By: Huiyong.Men 2018/06/29
//

#pragma once
#include <memory>

class Imp;

class CyberWriterPointCloud
{
  public:
	 CyberWriterPointCloud();
	 ~CyberWriterPointCloud();
	 void publish(int sequence, int width, int height, int step, const char *data) const;

  private:
	std::shared_ptr<Imp> mImp;
};
