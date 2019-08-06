#pragma once
#include <memory>

#define OBSTACLE_MAX_SIZE 512

class Imp;
class CyberWriterObstacles
{
public:
    struct PolygonPoint
    {
        float x;
        float y;
        float z;
    };
    struct Obstacle
    {
        int id;
        int type;
        float theta;
        float posX;
        float posY;
        float posZ;
        float velX;
        float velY;
        float velZ;
        float length;
        float width;
        float height;
        int polygonPointSize;
        PolygonPoint polygonPointList[128];
        float confidence;
        int confidenceType;
    };
    CyberWriterObstacles();
    void publish(int sequence) const;
    int mObstacleSize;
    Obstacle mObstacleList[OBSTACLE_MAX_SIZE];

private:
    std::shared_ptr<Imp> mImp;
};
