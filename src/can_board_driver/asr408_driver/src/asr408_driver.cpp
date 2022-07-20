#include "asr408_driver/asr408_driver.h"
uint16_t ASR408Driver::ChtoID(uint8_t ch, uint8_t c)
{
  uint16_t ida = 0X06 << 8 | ch << 4 | c;
  return ida;
}
uint32_t ASR408Driver::Objects_number() { return objects_num_; }
asr408_msgs::ObjectList ASR408Driver::Object_list() { return obj_list_; }

void ASR408Driver::Decode(unsigned char *tbuff, uint16_t id, uint8_t channel, uint16_t index)
{
  // std::cout << "can id： " << std::hex << id << std::endl;
  if (id == ChtoID(channel, 0x0a))
  {
    Decode60a(tbuff, index);
  }
  else if (id == ChtoID(channel, 0x0b))
  {
    Decode60b(tbuff, index);
  }
  else if (id == ChtoID(channel, 0x0c))
  {
    Decode60c(tbuff, index);
  }
  else if (id == ChtoID(channel, 0x0d))
  {
    Decode60d(tbuff, index);
  }
  else if (id == ChtoID(channel, 0x0e))
  {
    Decode60e(tbuff, index);
  }
  else
  {
  }
}
inline void ASR408Driver::Decode60a(unsigned char *tbuff, uint16_t index)
{
  objects_num_ = *(tbuff + index + 18);
  uint64_t timestamp_sec = (*(tbuff + index + 4)) << 40 | (*(tbuff + index + 5)) << 32 |
                     (*(tbuff + index + 6)) << 24 | (*(tbuff + index + 7)) << 16 |
                     (*(tbuff + index + 8)) << 8 | (*(tbuff + index + 9));
  uint32_t timestamp_nsec = (*(tbuff + index + 10)) << 24 | (*(tbuff + index + 11)) << 16 |
                          (*(tbuff + index + 12)) << 8 | (*(tbuff + index + 13));
  obj_list_.header.stamp = ros::Time(static_cast<double>(timestamp_sec + timestamp_nsec * 1e-9));
  
  obj_list_.header.seq = *(tbuff + index + 19) << 8 | *(tbuff + index + 20);
  obj_list_.data.clear();
  // std::cout << "objects_num_ " << objects_num_ << " index: " << index << std::endl;
  // CanData.RecData60a.Channel=tbuff[index+14];
  // CanData.RecData60a.NofObjects=tbuff[index+18];
  // CanData.RecData60a.MeasCounter=tbuff[index+19]<<8|tbuff[index+20];
  // CanData.RecData60a.InterfaceVersion=(tbuff[index+21]>>4)&0x0f;

  // printf("NofObjects1:%d\n\r",CanData[1].RecData60a.NofObjects);
}
inline void ASR408Driver::Decode60b(unsigned char *tbuff, uint16_t index)
{
  asr408_msgs::Object object;

  object.ID = *(tbuff + index + 18);
  // std::cout << "6XB objects_id： " << (int)object.ID << std::endl;
  object.DistLong = ((*(tbuff + index + 19)) << 5 | (*(tbuff + index + 20) & 0xf8) >> 3) * 0.2 - 500;
  object.DistLat = (((*(tbuff + index + 20)) & 0x07) << 8 | (*(tbuff + index + 21))) * 0.2 - 204.6;
  object.VrelLong = (*(tbuff + index + 22) << 2 | (*(tbuff + index + 23) & 0xc0) >> 6) * 0.25 - 128.00;
  object.VrelLat = ((*(tbuff + index + 23) & 0x3f) << 3 | (*(tbuff + index + 24) & 0xe0) >> 5) * 0.25 - 64.00;
  object.DynProp = *(tbuff + index + 24) & 0x07;
  object.RCS = *(tbuff + index + 25) * 0.5 - 64.0;

  // if (object.DistLong < 5 && object.DistLat < 5) {
  //   std::cout << "6XB : " << std::hex << (int)*(tbuff+index+19) << " " << (int)*(tbuff+index+20) << std::endl;
  //   std::cout << std::hex << ((*(tbuff + index + 19)) << 5 | (*(tbuff+ index + 20) & 0xf8) >> 3) << std::endl;
  //   std::cout << "6XB : " << std::hex << (int)*(tbuff+index+20) << " " << (int)*(tbuff+index+21) << std::endl;
  //   std::cout << std::hex << (((*(tbuff+index+20))&0x07)<<8|(*(tbuff+index+21))) << std::endl;
  //   std::cout << "6XB DistLong: " << object.DistLong << " DistLat: " << object.DistLat << std::endl;
  // }

  obj_list_.data.emplace_back(std::move(object));
}
inline void ASR408Driver::Decode60c(unsigned char *tbuff, uint16_t index)
{
  for (auto &object : obj_list_.data)
  {
    if (object.ID == *(tbuff + index + 18))
    {
      // std::cout << "6XC objects_id： " << (int)object.ID << " " << (int)*(tbuff+index+18) << std::endl;
      object.DistLong_rms = (*(tbuff + index + 19) & 0xf8) >> 3;
      object.DistLat_rms = (*(tbuff + index + 19) & 0x07) << 2 | (*(tbuff + index + 20) >> 6);
      object.VrelLong_rms = (*(tbuff + index + 20) & 0x3e) >> 1;
      object.VrelLat_rms = (*(tbuff + index + 20) & 0x01) << 4 | (*(tbuff + index + 21) & 0xf0) >> 4;
      object.ArelLong_rms = (*(tbuff + index + 21) & 0x0f) << 1 | (*(tbuff + index + 22) & 0x80) >> 7;
      object.ArelLat_rms = (*(tbuff + index + 22) & 0x7c) >> 2;
      object.Orientation_rms = (*(tbuff + index + 22) & 0x03) << 3 | (*(tbuff + index + 23) & 0xe0) >> 5;
      object.MeasState = (*(tbuff + index + 24) & 0x1c) >> 2;
      object.ProbOfExist = (*(tbuff + index + 24) & 0xe0) >> 5;
    }
  }
}
inline void ASR408Driver::Decode60d(unsigned char *tbuff, uint16_t index)
{
  for (auto &object : obj_list_.data)
  {
    if (object.ID == *(tbuff + index + 18))
    {
      // std::cout << "6XD objects_id： " << (int)object.ID << std::endl;
      object.ArelLong = (*(tbuff + index + 19) << 3 | (*(tbuff + index + 20) & 0xe0) >> 5) * 0.01 - 10.00;
      object.ArelLat = ((*(tbuff + index + 20) & 0x1f) << 4 | (*(tbuff + index + 21) & 0xf0) >> 4) * 0.01 - 2.50;
      object.Class = *(tbuff + index + 21) & 0x07;
      object.OrientationAngel = (*(tbuff + index + 22) << 2 | (*(tbuff + index + 23) & 0xc0) >> 6) * 0.4 - 180.00;
      object.Length = *(tbuff + index + 24) * 0.2;
      object.Width = *(tbuff + index + 25) * 0.2;
    }
  }
}
inline void ASR408Driver::Decode60e(unsigned char *tbuff, uint16_t index)
{
  for (auto &object : obj_list_.data)
  {
    if (object.ID == *(tbuff + index + 18))
    {
      // std::cout << "6XE objects_id： " << (int)object.ID << std::endl;
      object.CollDetRegionBitfield = *(tbuff + index + 19);
    }
  }
}