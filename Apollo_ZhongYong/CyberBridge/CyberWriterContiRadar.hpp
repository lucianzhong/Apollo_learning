#pragma once
#include <memory>

#define OBJECT_MAX_SIZE 100
class Imp;
class CyberWriterContiRadar
{
public:
    struct Object
    {
        int id;
        float longitudeDist;
        float lateralDist;
        float longitudeVel;
        float lateralVel;
        float rcs;
        int dynprop;
    };
    CyberWriterContiRadar();
    void publish() const;
    int mObjectSize;
    Object mObjectList[OBJECT_MAX_SIZE];
private:
    std::shared_ptr<Imp> mImp;
};
