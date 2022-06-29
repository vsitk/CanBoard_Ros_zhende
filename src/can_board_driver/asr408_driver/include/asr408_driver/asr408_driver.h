#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include <asr408_msgs/ObjectList.h>
#include <asr408_msgs/Object.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <boost/thread.hpp>
class Driver
{
public:
    short ID_H(int ch,short c);
    void pub_poll(void);
    void Decode(unsigned char tbuff[1500],short id, int ch,int &po);
    void Decode60a(unsigned char tbuff[1500],int ch);
    void Decode60b(unsigned char tbuff[1500],int Nof,int ch,int comp);
    void Decode60c(unsigned char tbuff[1500],int Nof,int ch,int comp);
    void Decode60d(unsigned char tbuff[1500],int Nof,int ch,int comp);
    void Decode60e(unsigned char tbuff[1500],int Nof,int ch,int comp);
    void EnCode(int i,int ch);
    
    asr408_msgs::ObjectList msg[8];
    int dep_60b[8];
    int dep_60c[8];
    int dep_60d[8];
    int dep_60e[8];
struct Can_Data{
	struct Rec_Data60a{
        short    Channel;
        short    NofObjects;
        short    MeasCounter;
        short    InterfaceVersion;
	}RecData60a;
	struct Rec_Data60b{
        float    DistLong;
        float    DistLat;
        float    VrelLong;
        float    VrelLat;
        short    DynProp;
        float    RCS;
	}RecData60b[100];
	struct Rec_Data60c{
		short    DistLong_rms;
		short    DistLat_rms;
		short    VrelLong_rms;
		short    VrelLat_rms;
		short    ArelLong_rms;
		short    ArelLat_rms;
		short    Orientation_rms;
		short    MeasState;
		short    ProbOfExist;
	}RecData60c[100];
	struct Rec_Data60d{
		float    ArelLong;
		float    ArelLat;
		short    Class;
		float    OrientationAngel ;
		float    Length;
		float    Width;
	}RecData60d[100];
	struct Rec_Data60e{
	short    CollDetRegionBitfield;
	}RecData60e[100];
}CanData[8];
private:

};
