

#include "CyberWriterImage.hpp"
//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>
// bazel build //CyberBridge:libApolloCyber.so

#include "CyberWriterImage.hpp"
#include "cyber/cyber.h"
#include "cyber/time/time.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include <iostream>
#include <vector>



class Imp
{
public:
    std::shared_ptr<apollo::cyber::Writer<apollo::drivers::Image>> mWriter;
};


CyberWriterImage::CyberWriterImage(EImageType imageType):mImp(std::make_shared<Imp>())
{
    apollo::cyber::Init("apollo_publish_image");
    auto node = apollo::cyber::CreateNode("writer");
    mImp->mWriter = node->CreateWriter<apollo::drivers::Image>(mImageType == EImageType_Long ? "/apollo/sensor/camera/traffic/image_long" : "/apollo/sensor/camera/traffic/image_short");
}



 void CyberWriterImage::publish(int sequence, int width, int height, char* data) const{
        std::cout<<"publish Image"<<std::endl;
            
        auto ts = apollo::cyber::Time::Now().ToSecond();

        auto Image = std::make_shared<apollo::drivers::Image>();


        Image->mutable_header()->set_module_name("Image");
        Image->mutable_header()->set_timestamp_sec(ts);
        Image->mutable_header()->set_sequence_num(sequence);

        Image->set_frame_id(mImageType == EImageType_Long ? "long_camera" : "short_camera");

        Image->set_width(width);
        Image->set_height(height);
        Image->set_encoding("bgr8");

        Image->set_step( width*3);
        

       Image->set_data(data);
        mImp->mWriter->Write(Image);

 }








