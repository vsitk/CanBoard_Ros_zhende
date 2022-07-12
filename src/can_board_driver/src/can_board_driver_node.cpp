#include "can_board_driver.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "can_board_driver");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    CanBoardDriver can_board_driver(node, private_nh);
    can_board_driver.ProcessCan();

    ros::spin();
}