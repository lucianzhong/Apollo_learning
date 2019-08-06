#pragma once
#include <memory>
class Imp;
class CyberWriterInsStat
{
public:
		CyberWriterInsStat();
		void publish() const;

private:
	std::shared_ptr<Imp> mImp;
};