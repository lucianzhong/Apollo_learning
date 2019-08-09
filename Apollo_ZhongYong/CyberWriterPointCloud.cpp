#include "CyberWriterPointCloud.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include <iostream>
#include <vector>


class Imp
{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::PointCloud>> mWriter;
};


CyberWriterPointCloud::CyberWriterPointCloud():mImp(std::make_shared<Imp>())
{
    apollo::cyber::Init("apollo_publish_PointCloud");
    auto node = apollo::cyber::CreateNode("writer");
    mImp->mWriter = node->CreateWriter<apollo::drivers::PointCloud>("/apollo/sensor/lidar128/compensator/PointCloud2");
}




std::string names[] = {"x", "y", "z", "intensity", "ring"};
int offsets[] = {0, 4, 8, 16, 20};
int datatypes[] = {7, 7, 7, 7, 4}; //   uint8 FLOAT32 = 7， uint8 UINT16  = 4



void CyberWriterPointCloud::publish(int sequence, int width, int height, int step, const char *data) const{
    
    auto ts = apollo::cyber::Time::Now().ToSecond();

    auto PointCloud = std::make_shared<apollo::drivers::PointCloud>();

    apollo::drivers::PointXYZIT *PointXYZIT=PointCloud->add_point();


    // message

    // optional apollo.common.Header header = 1;
    PointCloud->mutable_header()->set_module_name("PointCloud");
    PointCloud->mutable_header()->set_timestamp_sec(ts);

    // optional string frame_id = 2;
    PointCloud->set_frame_id("velodyne64");


    // optional bool is_dense = 3;
    PointCloud->set_is_dense (true);  //is_dense若是true，代表点云数据中不包含无效点（nan点）；若是false，代表点云中包含无效点

   
    std::vector<float> PointXYZIT_x;
    std::vector<float> PointXYZIT_y;
    std::vector< float> PointXYZIT_z;
    std::vector< float> PointXYZIT_intensity;
    //std::vector< int> PointXYZIT_timestamp;


    /*
    for (int i=0;i<width;i++){
    PointXYZIT_x.push_back(*(float*)&data[i]);
    PointXYZIT_y.push_back(*(float*)&data[i+4]);
    PointXYZIT_z.push_back(*(float*)&data[i+8]);
    PointXYZIT_intensity.push_back(data[i+12]);
    PointXYZIT_timestamp.push_back(0);
    }
    */

    float x;
    float y;
    float z;
    float intensity;
    

   for (int i=0;i<=width;i++){
       // x
       memcpy(&x, &data[i],4 * sizeof(char));
       i += 4;
    
       PointXYZIT->set_x(x);

       // y
       memcpy(&y, &data[i], 4 * sizeof(char));
       i += 4;
     
         PointXYZIT->set_y(y);
       // z
       memcpy(&z, &data[i], 4 * sizeof(char));
       i += 4;
       PointXYZIT->set_z(z);
       //intensity
       memcpy(&intensity,&data[i], 4 * sizeof(char));
        PointXYZIT->set_intensity(intensity);
       i += 4;
       // ring
       i += 2;
   }
    


  

    PointXYZIT->set_timestamp(ts);

    
    // optional double measurement_time = 5;
    PointCloud->set_measurement_time( 1 );



    //optional uint32 width = 6;
     
    PointCloud->set_width(width);


    //optional uint32 height = 7;
    PointCloud->set_height(height);



    mImp->mWriter->Write(PointCloud);

}
