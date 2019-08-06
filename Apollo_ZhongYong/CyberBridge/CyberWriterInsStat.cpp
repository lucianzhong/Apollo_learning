#include "CyberWriterInsStat.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
class Imp
{
public:
	std::shared_ptr<apollo::cyber::Writer<apollo::drivers::gnss::InsStat>> mInsStatWriter;
};

CyberWriterInsStat::CyberWriterInsStat():mImp(std::make_shared<Imp>())
{
	apollo::cyber::Init("apollo_publish_gps");
    auto node = apollo::cyber::CreateNode("writerInsStat");
    mImp->mInsStatWriter = node->CreateWriter<apollo::drivers::gnss::InsStat>("/apollo/sensor/gnss/ins_stat");
}

void CyberWriterInsStat::publish() const
{
	if(mImp->mInsStatWriter)
	{
		auto ins_stat = std::make_shared<apollo::drivers::gnss::InsStat>();
		ins_stat->mutable_header()->set_module_name("gnss");
  		ins_stat->mutable_header()->set_timestamp_sec(apollo::cyber::Time::Now().ToSecond());
	 	ins_stat->set_ins_status(apollo::drivers::gnss::InsStatus::GOOD);
	 	ins_stat->set_pos_type(apollo::drivers::gnss::SolutionType::INS_RTKFIXED);//checked from rtk_localization.cc 
		mImp->mInsStatWriter->Write(ins_stat);
	}
}