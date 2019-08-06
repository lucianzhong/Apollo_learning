//
// Created By: Huiyong.Men 2018/07/24
//

#pragma once

class CyberWriterImage
{
public:
   enum EImageType {
    EImageType_Long,
    EImageType_Short
  };
   CyberWriterImage(EImageType);
   ~CyberWriterImage();
   void publish(int sequence, int width, int height, char* data) const;

private:
  EImageType mImageType;
//  void *mpPublisher;
};