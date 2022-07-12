#include "asr408_driver/asr408_driver.h"
#include "socket.h"
#include "h9_driver/h9_driver.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "geometry_msgs/Point.h"
#include "visualization_msgs/MarkerArray.h"

class CanBoardDriver {
  public:
    CanBoardDriver() = delete;
    CanBoardDriver(ros::NodeHandle& node, ros::NodeHandle& private_nh);
    virtual ~CanBoardDriver();
    void ProcessCan();

  private:
    void Channel_h9(unsigned char* tbuff);
    void Channel_408(unsigned char* tbuff, uint8_t channel);
    
  private:
    ros::Publisher asr408_front_pub_;
    ros::Publisher asr408_rear_pub_;
    ros::Publisher h9_pub_;
    ros::Publisher front_marker_pub_;
    ros::Publisher rear_marker_pub_;

    Socket sock_;

    ASR408Driver front_408_driver_;
    ASR408Driver rear_408_driver_;
    H9_Driver h9_driver_;

    uint32_t index_;
    uint32_t packet_nums_ = 0;
    uint32_t asr_60a_nums_ = 0;
};
