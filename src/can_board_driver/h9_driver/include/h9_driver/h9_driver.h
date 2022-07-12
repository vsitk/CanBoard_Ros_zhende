#include "ros/ros.h"
#include <ros/console.h>
 #include "h9_msgs/H9.h"
class H9_Driver
{
public:
    void H9_EnCode(unsigned char* tbuff,uint16_t comp);
    h9_msgs::H9 H9Message();
    bool GetState();
    void ClearState();

private:
    h9_msgs::H9 h9_msg_;
    bool b_abs1_;
    bool b_abs2_;
    bool b_ems2_;
    bool b_ems3_;
    bool b_sas1_;
    bool b_tcu3_;
};
