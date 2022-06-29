#include "h9_driver/h9_driver.h"
void H9_Driver::H9_EnCode(unsigned char tbuff[1500],int comp)
{
  short ID_N=tbuff[comp+15]<<8|tbuff[comp+16];
  switch (ID_N)
  {
  case 0x0c0:
    msgabs1.Checksum=tbuff[comp+18];
    msgabs1.WheelSpeed_FL=((tbuff[comp+20]&0x7f)<<8|tbuff[comp+19])*0.01;
    msgabs1.WheelSpeed_FR=((tbuff[comp+22]&0x7f)<<8|tbuff[comp+21])*0.01;
    msgabs1.VehicleSpeed=((tbuff[comp+24]&0x07f)<<8|tbuff[comp+23])*0.01;
    msgabs1.DrivingDirection=tbuff[comp+25]&0x03;
    msgabs1.EPBSwitch=tbuff[comp+25]&0x0c;
    msgabs1.LiveCounter=(tbuff[comp+25]&0xf0)>>4;
    pubabs1.publish(msgabs1);
    break;
  case 0x0c2:
    msgabs2.Checksum=tbuff[comp+18];
    msgabs2.WheelSpeed_RL=((tbuff[comp+20]&0x7f)<<8|tbuff[comp+19])*0.01;
    msgabs2.WheelSpeed_RR=((tbuff[comp+22]&0x7f)<<8|tbuff[comp+21])*0.01;
    msgabs2.MasterCylinderPressure=((tbuff[comp+24]&0x07)<<8|tbuff[comp+23])*0.1;
    msgabs2.LiveCounter=(tbuff[comp+25]&0xf0)>>4;
    pubabs2.publish(msgabs2);
    break;
  case 0x130:
    msgabs3.Checksum=tbuff[comp+18];
    msgabs3.WheelPulse_FL=tbuff[comp+19];
    msgabs3.WheelPulse_FR=tbuff[comp+20];
    msgabs3.WheelPulse_RL=tbuff[comp+21];
    msgabs3.WheelPulse_RR=tbuff[comp+22];
    msgabs3.WheelSpeedDirection_FL=tbuff[comp+23]&0x03;
    msgabs3.WheelSpeedDirection_FR=(tbuff[comp+23]&0x0c)>>2;
    msgabs3.WheelSpeedDirection_RL=(tbuff[comp+23]&0x30)>>4;
    msgabs3.WheelSpeedDirection_RR=(tbuff[comp+23]&0xc0)>>6;
    msgabs3.BrakeTorqueOnWheels=((tbuff[comp+25]&0x0f)<<8|tbuff[comp+24])*2;
    msgabs3.LiveCounter=(tbuff[comp+25]&0xf0)>>4;
    pubabs3.publish(msgabs3);
    break;
  case 0x092:
    msgems2.Checksum=tbuff[comp+18];
    msgems2.IndicatedDriverTorq=((tbuff[comp+20]&0x0f)<<8|tbuff[comp+19])*0.25;
    msgems2.ThrottlePosition=(tbuff[comp+22])*0.4;
    msgems2.AccelerationPedalPosition=(tbuff[comp+23])*0.4;
    msgems2.CalcAccelerationPedalPositi=(tbuff[comp+24])*0.4;
    msgems2.KickdownReq=(tbuff[comp+25]&0x02)>>1;
    msgems2.LiveCounter=(tbuff[comp+25]&0xf0)>>4;
    pubems2.publish(msgems2);
    break;
  case 0x094:
    msgems3.Checksum=tbuff[comp+18];
    msgems3.EngineSpeed=((tbuff[comp+20]<<8)|tbuff[comp+19])*0.25;
    msgems3.EngineSt=tbuff[comp+21]&0x07;
    msgems3.BrakePedalSt=(tbuff[comp+21]&0x18)>>3;
    msgems3.LiveCounter=(tbuff[comp+25]&0xf0)>>4;
    pubems3.publish(msgems3);
    break;
  case 0x2a6:
    msggps1.GPSTime_Hour=tbuff[comp+18]&0x1f;
    msggps1.GPSTime_Minute=tbuff[comp+19]&0x3f;
    msggps1.GPSTime_Second=tbuff[comp+20]&0x3f;
    msggps1.GPSTime_Year=(int)(tbuff[comp+21]&0x7f)+2011;
    msggps1.GPSTime_Month=(tbuff[comp+22]&0x0f)+1;
    msggps1.GPSTime_Day=(tbuff[comp+23]&0x7f)+1;
    pubgps1.publish(msggps1);
    break;
  case 0x0d0:
    msgsas1.Checksum=tbuff[comp+18];
    msgsas1.SteeringWheelAngle=((tbuff[comp+20]&0x7f)<<8|tbuff[comp+19])*0.04375;
    msgsas1.SteeringWheelAngleSign=(tbuff[comp+20]&0x80)>>7;
    msgsas1.SteeringWheelSpeed=((tbuff[comp+22]&0x7f)<<8|tbuff[comp+21])*0.04375;
    msgsas1.SteeringWheelSpeedSign=(tbuff[comp+22]&0x80)>>7;
    msgsas1.SAS1_St=tbuff[comp+25]&0x03;
    msgsas1.LiveCounter=(tbuff[comp+25]&0xf0)>>4;
    pubasa1.publish(msgsas1);
    break;
  case 0x0b4:
    msgtcu3.Checksum=tbuff[comp+18];
    msgtcu3.ActualGear=(tbuff[comp+21]&0xf0)>>4;
    msgtcu3.TargetGear=tbuff[comp+22]&0x0f;
    msgtcu3.ShiftingInProgress=(tbuff[comp+22]&0x10)>>4;
    msgtcu3.LeverInfo=tbuff[comp+25]&0x0f;
    msgtcu3.LiveCounter=(tbuff[comp+25]&0xf0)>>4;
    pubtcu1.publish(msgtcu3);
    break;
  default:
    break;
  }
}