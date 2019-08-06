//
// Created By: Huiyong.Men 2018/07/24
//

#include "CyberWriterImage.hpp"
//#include <ros/ros.h>
//#include <sensor_msgs/Image.h>

CyberWriterImage::CyberWriterImage(EImageType imageType) : mImageType(imageType)
{
    // int argc = 0;
    // ros::init(argc, nullptr, mImageType == EImageType_Long ? "apollo_publish_image_long" : "apollo_publish_image_short");

    // ros::NodeHandle handler;
    // ros::Publisher *pPublisher = new ros::Publisher();
    // *pPublisher = handler.advertise<sensor_msgs::Image>(mImageType == EImageType_Long ? "/apollo/sensor/camera/traffic/image_long" : "/apollo/sensor/camera/traffic/image_short", 10);
    // mpPublisher = (void *)pPublisher;
}

CyberWriterImage::~CyberWriterImage()
{
    // if (mpPublisher)
    //     delete (ros::Publisher *)mpPublisher;
}

 void CyberWriterImage::publish(int sequence, int width, int height, char* data) const
{
    // sensor_msgs::Image img;
    // img.header.seq = sequence;
    // img.header.stamp = ros::Time::now();
    // img.header.frame_id = mImageType == EImageType_Long ? "long_camera" : "short_camera";
    // img.width = width;
    // img.height = height;
    // img.encoding = "bgr8";
    // img.is_bigendian = false;
    // img.step = img.width * 3;
    // img.data.resize(img.step*height);
    // memcpy(img.data.data(), data, img.step*height);
    // ros::Publisher *pPublisher = (ros::Publisher *)mpPublisher;
    // pPublisher->publish(img);
}