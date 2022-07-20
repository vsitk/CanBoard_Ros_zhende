#include "h9_driver/h9_driver.h"
h9_msgs::H9 H9_Driver::H9Message(unsigned char* tbuff,uint16_t comp){
  uint64_t timestamp_sec = (*(tbuff + comp + 4)) << 40 | (*(tbuff + comp + 5)) << 32 |
                           (*(tbuff + comp + 6)) << 24 | (*(tbuff + comp + 7)) << 16 |
                           (*(tbuff + comp + 8)) << 8 | (*(tbuff + comp + 9));
  uint32_t timestamp_nsec = (*(tbuff + comp + 10)) << 24 | (*(tbuff + comp + 11)) << 16 |
                            (*(tbuff + comp + 12)) << 8 | (*(tbuff + comp + 13));
  h9_msg_.header.stamp = ros::Time(static_cast<double>(timestamp_sec + timestamp_nsec * 1e-9));
  return h9_msg_;}
bool H9_Driver::GetState()
{
  if(b_abs1_ && b_abs2_ && b_ems2_ && b_ems3_ && b_sas1_ && b_tcu3_)
  {
    return true;
  }else
  {
    return false;
  }
  
}
void H9_Driver::ClearState(){b_abs1_ = false;
                             b_abs2_ = false;
                             b_ems2_ = false;
                             b_ems3_ = false;
                             b_sas1_ = false;
                             b_tcu3_ = false;
}

void H9_Driver::H9_EnCode(unsigned char* tbuff,uint16_t comp)
{
  uint16_t id_n=*(tbuff+comp+15)<<8|*(tbuff+comp+16);
  switch (id_n)
  {
  case 0x0c0:
    h9_msg_.WheelSpeed_FL=((*(tbuff+comp+20)&0x7f)<<8|*(tbuff+comp+19))*0.01;
    h9_msg_.WheelSpeed_FR=((*(tbuff+comp+22)&0x7f)<<8|*(tbuff+comp+21))*0.01;
    h9_msg_.VehicleSpeed=((*(tbuff+comp+24)&0x07f)<<8|*(tbuff+comp+23))*0.01;
    b_abs1_=true;
    break;
  case 0x0c2:
    h9_msg_.WheelSpeed_RL=((*(tbuff+comp+20)&0x7f)<<8|*(tbuff+comp+19))*0.01;
    h9_msg_.WheelSpeed_RR=((*(tbuff+comp+22)&0x7f)<<8|*(tbuff+comp+21))*0.01;
    b_abs2_=true;
    break;
  case 0x092:
    h9_msg_.ThrottlePosition=(*(tbuff+comp+22))*0.4;
    b_ems2_=true;
    break;
  case 0x094:
    h9_msg_.BrakePedalSt=(*(tbuff+comp+22)&0x18)>>3;
    b_ems3_=true;
    break;
  case 0x0d0:
  {
    uint8_t nsign = (tbuff[comp+20]&0x80)>>7;
    h9_msg_.SteeringWheelAngle=((*(tbuff+comp+20)&0x7f)<<8|*(tbuff+comp+19))*0.04375;
    if(nsign==1)
    {
      h9_msg_.SteeringWheelAngle *= -1;
    }
    nsign=(tbuff[comp+22]&0x80)>>7;
    h9_msg_.SteeringWheelSpeed=((*(tbuff+comp+22)&0x7f)<<8|*(tbuff+comp+21))*0.04375;
    if(nsign==1)
    {
      h9_msg_.SteeringWheelSpeed *= -1;
    }
    b_sas1_=true;
    break;
  }
  case 0x0b4:
    h9_msg_.ActualGear=(*(tbuff+comp+21)&0xf0)>>4;
    b_tcu3_=true;
    break;
  default:
    break;
  }
}