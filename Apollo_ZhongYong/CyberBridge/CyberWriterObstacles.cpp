#include "CyberWriterObstacles.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/perception/proto/perception_obstacle.pb.h"

class Imp
{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::perception::PerceptionObstacles>> mWriter;
};

CyberWriterObstacles::CyberWriterObstacles():mImp(std::make_shared<Imp>())
{
    mObstacleSize = 0;
    memset(mObstacleList, 0, sizeof(Obstacle) * OBSTACLE_MAX_SIZE);
    
    apollo::cyber::Init("apollo_publish_obstacles");
    auto node = apollo::cyber::CreateNode("writer");
    mImp->mWriter = node->CreateWriter<apollo::perception::PerceptionObstacles>("/apollo/perception/obstacles");
}

void CyberWriterObstacles::publish(int sequence) const
{
    auto obstacles = std::make_shared<apollo::perception::PerceptionObstacles>();
    obstacles->mutable_header()->set_module_name("perception_obstacle");
    auto ts = apollo::cyber::Time::Now().ToSecond();
    obstacles->mutable_header()->set_timestamp_sec(ts);
    obstacles->mutable_header()->set_sequence_num(sequence);
    for (int i = 0; i < mObstacleSize; i++)
    {
        apollo::perception::PerceptionObstacle *obstacle = obstacles->add_perception_obstacle();
        obstacle->set_timestamp(ts);
        obstacle->set_id(mObstacleList[i].id);
        obstacle->set_type((apollo::perception::PerceptionObstacle_Type)mObstacleList[i].type);
        obstacle->set_theta(mObstacleList[i].theta);
        obstacle->mutable_position()->set_x(mObstacleList[i].posX);
        obstacle->mutable_position()->set_y(mObstacleList[i].posY);
        obstacle->mutable_position()->set_z(mObstacleList[i].posZ);
        obstacle->mutable_velocity()->set_x(mObstacleList[i].velX);
        obstacle->mutable_velocity()->set_y(mObstacleList[i].velY);
        obstacle->mutable_velocity()->set_z(mObstacleList[i].velZ);
        obstacle->set_length(mObstacleList[i].length);
        obstacle->set_width(mObstacleList[i].width);
        obstacle->set_height(mObstacleList[i].height);
        for (int j = 0; j < mObstacleList[i].polygonPointSize; j++)
        {
            auto *point = obstacle->add_polygon_point();
            point->set_x(mObstacleList[i].polygonPointList[j].x);
            point->set_y(mObstacleList[i].polygonPointList[j].y);
            point->set_z(mObstacleList[i].polygonPointList[j].z);
        }
        obstacle->set_confidence(mObstacleList[i].confidence);
        obstacle->set_confidence_type((apollo::perception::PerceptionObstacle_ConfidenceType)mObstacleList[i].confidenceType);
    }
    mImp->mWriter->Write(obstacles);
}
