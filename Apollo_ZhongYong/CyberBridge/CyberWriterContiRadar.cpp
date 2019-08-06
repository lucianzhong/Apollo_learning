#include "CyberWriterContiRadar.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/drivers/proto/conti_radar.pb.h"

class Imp{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::ContiRadar>> mWriter;
};
CyberWriterContiRadar::CyberWriterContiRadar():mImp(std::make_shared<Imp>())
{
    mObjectSize = 0;
    memset(mObjectList, 0, sizeof(Object) * OBJECT_MAX_SIZE);

    apollo::cyber::Init("apollo_publish_conti_radar");
    auto node = apollo::cyber::CreateNode("writer");
    mImp->mWriter = node->CreateWriter<apollo::drivers::ContiRadar>("/apollo/sensor/conti_radar");

}

void CyberWriterContiRadar::publish() const
{
    auto contiRadar = std::make_shared<apollo::drivers::ContiRadar>();
    auto ts = apollo::cyber::Time::Now().ToSecond();
    contiRadar->mutable_header()->set_timestamp_sec(ts);
    for (int i = 0; i < mObjectSize; i++)
    {
        auto ob = contiRadar->add_contiobs();
        ob->mutable_header()->set_timestamp_sec(ts);
        ob->set_clusterortrack(true);
        ob->set_obstacle_id(mObjectList[i].id);
        ob->set_longitude_dist(mObjectList[i].longitudeDist);
        ob->set_lateral_dist(mObjectList[i].lateralDist);
        ob->set_longitude_vel(mObjectList[i].longitudeVel);
        ob->set_lateral_vel(mObjectList[i].lateralVel);
        ob->set_rcs(mObjectList[i].rcs);
        ob->set_dynprop(mObjectList[i].dynprop);
    }
    mImp->mWriter->Write(contiRadar);
}
