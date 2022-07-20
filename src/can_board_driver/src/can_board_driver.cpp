#include "can_board_driver.h"

using namespace std;

CanBoardDriver::CanBoardDriver(ros::NodeHandle& node, ros::NodeHandle& private_nh)
{
    asr408_front_pub_ = node.advertise<asr408_msgs::ObjectList>("radar/front", 10);
    asr408_rear_pub_ = node.advertise<asr408_msgs::ObjectList>("radar/rear", 10);
    h9_pub_ = node.advertise<h9_msgs::H9>("h9", 10);
    front_marker_pub_ = node.advertise<visualization_msgs::Marker>("radar/front_marker", 10);
    rear_marker_pub_ = node.advertise<visualization_msgs::Marker>("radar/rear_marker", 10);

    sock_.Init();
}

CanBoardDriver::~CanBoardDriver()
{
    close(sock_.sockfd_);
}

void CanBoardDriver::ProcessCan()
{
    unsigned char recbuff[1500];
    uint8_t channel;
    uint32_t len;
    
    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);

    while (ros::ok())
    {
        memset(recbuff, 0, 1500);
        index_ = 0;

        len = recvfrom(sock_.sockfd_, recbuff, sizeof(recbuff), 0, (sockaddr *)&sender_address, &sender_address_len);
        // if (len > 0)
        //     packet_nums_++;
        
        while (index_ < 1500 && ros::ok())
        {
            uint32_t CanType = recbuff[index_ + 0] << 24 | recbuff[index_ + 1] << 16 | recbuff[index_ + 2] << 8 | recbuff[index_ + 3];
            // cout << "receive data. CanType: " << hex << CanType << " index_: " << dec << index_ << endl;
            if (CanType == 0xaa55aa55)
            {
                channel = recbuff[index_ + 14];
                // cout << "channel： " << hex << channel << endl;

                switch (channel)
                {
                case 0:
                    Channel_408(recbuff, channel);
                    break;
                case 1:
                    Channel_408(recbuff, channel);
                    break;
                case 2:
                    Channel_h9(recbuff);
                    break;
                default:
                    break;
                }

                auto offset = recbuff[index_ + 17];
                index_ += offset + 18;
            }
            else
            {
                break;
            }
        }
    }
    // cout << "packet nums: " << packet_nums_ << endl;
}

void CanBoardDriver::Channel_h9(unsigned char *tbuff)
{
    uint16_t can_id = *(tbuff + index_ + 15) << 8 | *(tbuff + index_ + 16);
    // cout << hex <<  "can_id: " << can_id << " index_: " << dec << index_ << endl;
    if (can_id == 0x0c0 | can_id == 0x0c2 | can_id == 0x092 | can_id == 0x094 |
        can_id == 0x0d0 | can_id == 0x0b4)
    {
        h9_driver_.H9_EnCode(tbuff, index_);
        if (h9_driver_.GetState())
        {
            h9_pub_.publish(h9_driver_.H9Message(tbuff, index_));
            h9_driver_.ClearState();
        }
    }
}

void CanBoardDriver::Channel_408(unsigned char *tbuff, uint8_t channel)
{
    uint16_t can_id = (*(tbuff + index_ + 15) << 8 | *(tbuff + (index_ + 16)));
    // if (can_id == 0x60A && ch == 0)
    // {
    //     asr_60a_nums_++;
    //     cout << hex <<  "can_id: " << can_id << " nums: " << dec << asr_60a_nums_ << " index_: " << dec << index_ << " packet nums: " << packet_nums_ << endl;
    // }

    switch (channel)
    {
    case 0:
        if (can_id == 0x60A && front_408_driver_.Objects_number() > 0 && front_408_driver_.Object_list().data.size() == front_408_driver_.Objects_number())
        {
            // visualization_msgs::Marker marker_msg;
            // marker_msg.header.frame_id = "map";
            // marker_msg.header.seq = front_408_driver_.Object_list().header.seq;
            // marker_msg.header.stamp = front_408_driver_.Object_list().header.stamp;
            // marker_msg.type = visualization_msgs::Marker::POINTS;
            // marker_msg.scale.x = 0.2;
            // marker_msg.scale.y = 0.2;
            // marker_msg.color.r = 1.0;
            // marker_msg.color.g = 0.0;
            // marker_msg.color.b = 0.0;
            // marker_msg.color.a = 1.0;
            // marker_msg.lifetime = ros::Duration(0.08 * 1e9);

            // for (auto& obj : front_408_driver_.Object_list().data) {
            //   geometry_msgs::Point point_msg;
            //   point_msg.x = obj.DistLong;
            //   point_msg.y = obj.DistLat;
            //   point_msg.z = 0;
            //   marker_msg.points.emplace_back(std::move(point_msg));
            // }
            // front_marker_pub_.publish(marker_msg);

            // cout << "front objects_num： " << front_408_driver_.objects_num << " front size of object list: " << front_408_driver_.Object_list().data.size() << endl;
            asr408_front_pub_.publish(front_408_driver_.Object_list());
        }

        front_408_driver_.Decode(tbuff, can_id, channel, index_);

        break;
    case 1:
        if (can_id == 0x61A && rear_408_driver_.Objects_number() > 0 && rear_408_driver_.Object_list().data.size() == rear_408_driver_.Objects_number())
        {
            // visualization_msgs::Marker marker_msg;
            // marker_msg.header.frame_id = "map";
            // marker_msg.header.seq = rear_408_driver_.Object_list().header.seq;
            // marker_msg.header.stamp = rear_408_driver_.Object_list().header.stamp;
            // marker_msg.type = visualization_msgs::Marker::POINTS;
            // marker_msg.scale.x = 0.2;
            // marker_msg.scale.y = 0.2;
            // marker_msg.color.r = 1.0;
            // marker_msg.color.g = 0.0;
            // marker_msg.color.b = 0.0;
            // marker_msg.color.a = 1.0;
            // marker_msg.lifetime = ros::Duration(0.08 * 1e9);

            // for (auto& obj : rear_408_driver_.Object_list().data) {
            //   geometry_msgs::Point point_msg;
            //   point_msg.x = obj.DistLong;
            //   point_msg.y = obj.DistLat;
            //   point_msg.z = 0;
            //   marker_msg.points.emplace_back(std::move(point_msg));
            // }
            // rear_marker_pub_.publish(marker_msg);

            asr408_rear_pub_.publish(rear_408_driver_.Object_list());
        }

        rear_408_driver_.Decode(tbuff, can_id, channel, index_);

        break;
    default:
        break;
    }
}