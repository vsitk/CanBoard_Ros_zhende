
#include "asr408_driver/asr408_driver.h"
#include "socket.h"
#include "h9_driver/h9_driver.h"
using namespace std;
ros::Publisher pub0;
ros::Publisher pub1;
Socket sock;
Driver driver;
H9_Driver h9_driver;
bool isaa55=false;
int point=0;
bool isaa55_h9=false;
void Channel_h9(unsigned char tbuff[1500],int ch)
{
  int pot_h9=0;
  isaa55_h9=false;
  while(!isaa55_h9)
  {
  uint32_t CanType=tbuff[pot_h9+0]<<24|tbuff[pot_h9+1]<<16|tbuff[pot_h9+2]<<8
          |tbuff[pot_h9+3];
    short ID_h9=tbuff[pot_h9+15]<<8|tbuff[pot_h9+16];
    if(CanType==0xaa55aa55)
    {
      if(ID_h9==0x0c0|ID_h9==0x0c2|ID_h9==0x130|ID_h9==0x092|ID_h9==0x094
          |ID_h9==0X2a6|ID_h9==0x0d0|ID_h9==0x0b4)
      {
        h9_driver.H9_EnCode(tbuff,pot_h9);
      }
      pot_h9+=26;
    }else 
    {
      isaa55_h9=true;
      break;
    }
    
  }
}
void Channel(unsigned char tbuff[1500],int ch)
{
  short ID_N=tbuff[15]<<8|tbuff[16];
  if(ID_N==driver.ID_H(ch,0x0a))
  {
    driver.msg[ch].data.resize(driver.dep_60b[ch]);
    for(int i=0;i<driver.dep_60b[ch];i++)
    {
      driver.EnCode(i,ch);
      switch (ch)
      {
      case 0:
        pub0.publish(driver.msg[0]);
        break;
      case 1:
        pub1.publish(driver.msg[1]);
        break;
      default:
        break;
      }
    }
    driver.dep_60b[ch]=0;
    driver.dep_60c[ch]=0;
    driver.dep_60d[ch]=0;
    driver.dep_60e[ch]=0;
  }
  while(!isaa55)
  {
    uint32_t CanType=tbuff[point+0]<<24|tbuff[point+1]<<16|tbuff[point+2]<<8
          |tbuff[point+3];
    ID_N=tbuff[point+15]<<8|tbuff[point+16];
    if(CanType==0xaa55aa55&&(ID_N==driver.ID_H(ch,0x0a)|ID_N==driver.ID_H(ch,0x0b)
        |ID_N==driver.ID_H(ch,0x0c)|ID_N==driver.ID_H(ch,0x0d)|ID_N==driver.ID_H(ch,0x0e)))
    {
      isaa55=false;
      driver.Decode(tbuff,ID_N,ch,point);
    }else 
    {
      isaa55=true;
      break;
    }
  }
}
//////////////////////////////////////////////main/////////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_node");
  ros::NodeHandle n;
  unsigned char recbuff[1500];
  int channel;
  int len;

  sock.Initi();
  pub0= n.advertise<asr408_msgs::ObjectList>("radar/front", 10);
  pub1= n.advertise<asr408_msgs::ObjectList>("radar/rear", 10);
  h9_driver.pubabs1= n.advertise<h9_msgs::ABS1>("h9/abs1", 10);
  h9_driver.pubabs2= n.advertise<h9_msgs::ABS2>("h9/abs2", 10);
  h9_driver.pubabs3= n.advertise<h9_msgs::ABS3>("h9/abs3", 10);
  h9_driver.pubems2= n.advertise<h9_msgs::EMS2>("h9/ems2", 10);
  h9_driver.pubems3= n.advertise<h9_msgs::EMS3>("h9/ems3", 10);
  h9_driver.pubgps1= n.advertise<h9_msgs::GPS1>("h9/gps1", 10);
  h9_driver.pubasa1= n.advertise<h9_msgs::SAS1>("h9/sas1", 10);
  h9_driver.pubtcu1= n.advertise<h9_msgs::TCU3>("h9/tcu3", 10);
  sockaddr_in sender_address;
  socklen_t sender_address_len = sizeof(sender_address);
   while(ros::ok())
  {
    len=recvfrom(sock.sockfd_, recbuff, sizeof(recbuff), 0, (sockaddr*)&sender_address, &sender_address_len);
    uint32_t CanType=recbuff[0]<<24|recbuff[1]<<16|recbuff[2]<<8
          |recbuff[3];
    short ID_h9=recbuff[15]<<8|recbuff[16];
    channel=recbuff[14];
    switch (channel)
    {
    case 0:
      Channel(recbuff,channel);
      break;
    case 1:
      Channel(recbuff,channel);
      break;
    case 2:
      Channel_h9(recbuff,channel);
      break;
    case 3:
      //Channel_h9(recbuff,channel);
      break;
    case 4:
      Channel(recbuff,channel);
      break;
    case 5:
      Channel(recbuff,channel);
      break;
    case 6:
      Channel(recbuff,channel);
      break;
    case 7:
      Channel(recbuff,channel);
      break;
    default:
      break;
    }
    isaa55=false; 
    point=0;
  }
  ros::spin();
  close(sock.sockfd_);
  return 0;
}
