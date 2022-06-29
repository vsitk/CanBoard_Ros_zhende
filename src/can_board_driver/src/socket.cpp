#include "socket.h"
void Socket::Initi()
{
    sockfd_ = -1;
    sockfd_ = socket(PF_INET, SOCK_DGRAM, 0);
    if (sockfd_ == -1)
    {
        perror("socket");  // TODO: ROS_ERROR errno
        return;
    }
    int opt = 1;
    if (setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (const void*)&opt, sizeof(opt)))
    {
        perror("setsockopt error!\n");
        return;
    }
    sockaddr_in my_addr;                   // my address information
    memset(&my_addr, 0, sizeof(my_addr));  // initialize to zeros
    my_addr.sin_family = AF_INET;          // host byte order
    my_addr.sin_port = htons(23400);        // port in network byte order
    my_addr.sin_addr.s_addr = INADDR_ANY;  // automatically fill in my IP

    if (bind(sockfd_, (sockaddr*)&my_addr, sizeof(sockaddr)) == -1)
    {
        perror("bind");  // TODO: ROS_ERROR errno
        return;
    }else
    {
        printf("连接成功............\n"); 
    }
    
}