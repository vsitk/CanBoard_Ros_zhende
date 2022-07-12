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
class ASR408Driver
{
public:
    uint16_t ChtoID(uint8_t channel,uint8_t c);
    void Decode(unsigned char* tbuff,uint16_t id, uint8_t channel, uint16_t index);
    void Decode60a(unsigned char* tbuff, uint16_t index);
    void Decode60b(unsigned char* tbuff, uint16_t index);
    void Decode60c(unsigned char* tbuff, uint16_t index);
    void Decode60d(unsigned char* tbuff, uint16_t index);
    void Decode60e(unsigned char* tbuff, uint16_t index);
    
    uint32_t Objects_number();
    asr408_msgs::ObjectList Object_list();
private:
	uint32_t objects_num_;
    asr408_msgs::ObjectList obj_list_;
};
