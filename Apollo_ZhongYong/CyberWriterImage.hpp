//
// Created By: Huiyong.Men 2018/07/24
//

#pragma once
#include <memory>

class Imp;

class CyberWriterImage
{
public:
   enum EImageType {
    EImageType_Long,
    EImageType_Short
  };
   CyberWriterImage(EImageType);
   ~CyberWriterImage() = default;;
   void publish(int sequence, int width, int height, char* data) const;

private:
  EImageType mImageType;
  std::shared_ptr<Imp> mImp;
};