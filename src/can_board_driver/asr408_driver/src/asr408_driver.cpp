#include "asr408_driver/asr408_driver.h"

short Driver::ID_H(int ch,short c)
{
  short ida=0X06<<8|ch<<4|c;
  return ida;
}
void Driver::Decode(unsigned char tbuff[1500],short id, int ch,int &po)
{
    if(id==ID_H(ch,0x0a))
    {
        Driver::Decode60a(tbuff,ch);
        po+=22;
    }else if(id==ID_H(ch,0x0b))
    {
        Driver::Decode60b(tbuff,dep_60b[ch],ch,po);
        dep_60b[ch]++;
        po+=26;
    }else if(id==ID_H(ch,0x0c))
    {
        Driver::Decode60c(tbuff,dep_60c[ch],ch,po);
        dep_60c[ch]++;
        po+=25;
    }else if(id==ID_H(ch,0x0d))
    {
        Driver::Decode60d(tbuff,dep_60d[ch],ch,po);
        dep_60d[ch]++;
        po+=26;
    }else if(id==ID_H(ch,0x0e))
    {
        Driver::Decode60e(tbuff,dep_60e[ch],ch,po);
        dep_60e[ch]++;
        po+=20;
    }else
    {

    }
}
inline void Driver::Decode60a(unsigned char tbuff[1500],int ch)
{
    CanData[ch].RecData60a.Channel=tbuff[14];
    CanData[ch].RecData60a.NofObjects=tbuff[18];
    CanData[ch].RecData60a.MeasCounter=tbuff[19]<<8|tbuff[20];
    CanData[ch].RecData60a.InterfaceVersion=(tbuff[21]>>4)&0x0f;
}
inline void Driver::Decode60b(unsigned char tbuff[1500],int Nof,int ch,int comp)
{
  CanData[ch].RecData60b[Nof].DistLong =(tbuff[comp+19]<<5|(tbuff[comp+20]&0xf8)>>3)*0.2-500;
  CanData[ch].RecData60b[Nof].DistLat=((tbuff[comp+20]&0x03)<<8|(tbuff[comp+21]))*0.2-204.6;
  CanData[ch].RecData60b[Nof].VrelLong=(tbuff[comp+22]<<2|(tbuff[comp+23]&0xc0)>>6)*0.25-128.00;
  CanData[ch].RecData60b[Nof].VrelLat=((tbuff[comp+23]&0x3f)<<3|(tbuff[comp+24]&0xe0)>>5)*0.25-64.00;
  CanData[ch].RecData60b[Nof].DynProp=tbuff[comp+24]&0x07;
  CanData[ch].RecData60b[Nof].RCS=tbuff[comp+25]*0.5-64.0;
}
inline void Driver::Decode60c(unsigned char tbuff[1500],int Nof,int ch,int comp)
{
  CanData[ch].RecData60c[Nof].DistLong_rms=(tbuff[comp+19]&0xf8)>>3;
  CanData[ch].RecData60c[Nof].DistLat_rms=(tbuff[comp+19]&0x07)<<2|(tbuff[comp+20]>>6);
  CanData[ch].RecData60c[Nof].VrelLong_rms=(tbuff[comp+20]&0x3e)>>1;
  CanData[ch].RecData60c[Nof].VrelLat_rms=(tbuff[comp+20]&0x01)<<4|(tbuff[comp+21]&0xf0)>>4;
  CanData[ch].RecData60c[Nof].ArelLong_rms=(tbuff[comp+21]&0x0f)<<1|(tbuff[comp+22]&0x80)>>7;
  CanData[ch].RecData60c[Nof].ArelLat_rms=(tbuff[comp+22]&0x7c)>>2;
  CanData[ch].RecData60c[Nof].Orientation_rms=(tbuff[comp+22]&0x03)<<3|(tbuff[comp+23]&0xe0)>>5;
  CanData[ch].RecData60c[Nof].MeasState=(tbuff[comp+24]&0x1c)>>2;
  CanData[ch].RecData60c[Nof].ProbOfExist=(tbuff[comp+24]&0xe0)>>5;
}
inline void Driver::Decode60d(unsigned char tbuff[1500],int Nof,int ch,int comp)
{
  CanData[ch].RecData60d[Nof].ArelLong=(tbuff[comp+19]<<3|(tbuff[comp+20]&0xe0)>>5)*0.01-10.00;
  CanData[ch].RecData60d[Nof].ArelLat=((tbuff[comp+20]&0x1f)<<4|(tbuff[comp+21]&0xf0)>>4)*0.01-2.50;
  CanData[ch].RecData60d[Nof].Class=tbuff[comp+21]&0x07;
  CanData[ch].RecData60d[Nof].OrientationAngel=((tbuff[comp+22])<<2|(tbuff[comp+23]&0xc0)>>6)*0.4-180.00;
  CanData[ch].RecData60d[Nof].Length=(tbuff[comp+24])*0.2;
  CanData[ch].RecData60d[Nof].Width=tbuff[comp+25]*0.2;
}
inline void Driver::Decode60e(unsigned char tbuff[1500],int Nof,int ch,int comp)
{
	CanData[ch].RecData60e[Nof].CollDetRegionBitfield=tbuff[comp+19];
}  
void Driver::EnCode(int i,int ch)
{
  msg[ch].header.stamp = ros::Time::now();
  msg[ch].header.frame_id="";
  msg[ch].data[i].Channel=CanData[ch].RecData60a.Channel;
  msg[ch].data[i].NofObjects=CanData[ch].RecData60a.NofObjects;
  msg[ch].data[i].MeasCounter=CanData[ch].RecData60a.MeasCounter;
  msg[ch].data[i].InterfaceVersion=CanData[ch].RecData60a.InterfaceVersion;
  msg[ch].data[i].DistLong=CanData[ch].RecData60b[i].DistLong;
  msg[ch].data[i].DistLat=CanData[ch].RecData60b[i].DistLat;
  msg[ch].data[i].VrelLong=CanData[ch].RecData60b[i].VrelLong;
  msg[ch].data[i].VrelLat=CanData[ch].RecData60b[i].VrelLat;
  msg[ch].data[i].DynProp=CanData[ch].RecData60b[i].DynProp;
  msg[ch].data[i].RCS=CanData[ch].RecData60b[i].RCS;
  msg[ch].data[i].DistLong_rms=CanData[ch].RecData60c[i].DistLong_rms;
  msg[ch].data[i].DistLat_rms=CanData[ch].RecData60c[i].DistLong_rms;
  msg[ch].data[i].VrelLong_rms=CanData[ch].RecData60c[i].VrelLong_rms;
  msg[ch].data[i].VrelLat_rms=CanData[ch].RecData60c[i].VrelLat_rms;
  msg[ch].data[i].ArelLong_rms=CanData[ch].RecData60c[i].ArelLong_rms;
  msg[ch].data[i].ArelLat_rms=CanData[ch].RecData60c[i].ArelLat_rms;
  msg[ch].data[i].Orientation_rms=CanData[ch].RecData60c[i].Orientation_rms;
  msg[ch].data[i].MeasState=CanData[ch].RecData60c[i].MeasState;
  msg[ch].data[i].ProbOfExist=CanData[ch].RecData60c[i].ProbOfExist;
  msg[ch].data[i].ArelLong=CanData[ch].RecData60d[i].ArelLong;
  msg[ch].data[i].ArelLat=CanData[ch].RecData60d[i].ArelLat;
  msg[ch].data[i].Class=CanData[ch].RecData60d[i].Class;
  msg[ch].data[i].OrientationAngel=CanData[ch].RecData60d[i].OrientationAngel;
  msg[ch].data[i].Length=CanData[ch].RecData60d[i].Length;
  msg[ch].data[i].Width=CanData[ch].RecData60d[i].Width;
  msg[ch].data[i].CollDetRegionBitfield=CanData[ch].RecData60e[i].CollDetRegionBitfield;
}
