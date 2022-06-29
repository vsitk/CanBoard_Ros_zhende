#include "ros/ros.h"
#include <ros/console.h>
#include "h9_msgs/ABS1.h"
#include "h9_msgs/ABS2.h"
#include "h9_msgs/ABS3.h"
#include "h9_msgs/EMS2.h"
#include "h9_msgs/EMS3.h"
#include "h9_msgs/GPS1.h"
#include "h9_msgs/SAS1.h"
#include "h9_msgs/TCU3.h"
class H9_Driver
{
private:
public:
    void H9_EnCode(unsigned char tbuff[1500],int comp);
    ros::Publisher pubabs1;
	ros::Publisher pubabs2;
	ros::Publisher pubabs3;
	ros::Publisher pubems2;
	ros::Publisher pubems3;
	ros::Publisher pubgps1;
	ros::Publisher pubasa1;
	ros::Publisher pubtcu1;
    h9_msgs::ABS1 msgabs1;
    h9_msgs::ABS2 msgabs2;
    h9_msgs::ABS3 msgabs3;
    h9_msgs::EMS2 msgems2;
    h9_msgs::EMS3 msgems3;
    h9_msgs::GPS1 msggps1;
    h9_msgs::SAS1 msgsas1;
    h9_msgs::TCU3 msgtcu3;
};
