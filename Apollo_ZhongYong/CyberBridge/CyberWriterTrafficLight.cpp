#include "CyberWriterTrafficLight.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"
class Imp
{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::perception::TrafficLightDetection>> mWriter;
};
CyberWriterTrafficLight::CyberWriterTrafficLight():mImp(std::make_shared<Imp>())
{
	apollo::cyber::Init("apollo_publish_traffic_light");
    auto node = apollo::cyber::CreateNode("writer");
    mImp->mWriter = node->CreateWriter<apollo::perception::TrafficLightDetection>("/apollo/perception/traffic_light");
}

void CyberWriterTrafficLight::publish(int sequence, char *id[], float confidence[], int color[], int size) const
{
	auto lights = std::make_shared<apollo::perception::TrafficLightDetection>();
	auto ts = apollo::cyber::Time::Now().ToSecond();
	lights->mutable_header()->set_module_name("perception_trafficLight");
	lights->mutable_header()->set_timestamp_sec(ts);
	lights->mutable_header()->set_sequence_num(sequence);
	for (int i = 0; i < size; i++)
	{
		apollo::perception::TrafficLight *light = lights->add_traffic_light();
		light->set_id(id[i]);
		light->set_confidence(confidence[i]);
		light->set_color(apollo::perception::TrafficLight_Color(color[i]));
	}
	mImp->mWriter->Write(lights);
}
