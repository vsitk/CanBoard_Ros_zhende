#include "ros/ros.h"
#include <ros/console.h>
#include "std_msgs/String.h"
#include "asr408_msgs/ObjectList.h"
#include "asr408_msgs/Object.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <boost/thread.hpp>
class Socket{
public:
    void Init();
    int sockfd_;

};