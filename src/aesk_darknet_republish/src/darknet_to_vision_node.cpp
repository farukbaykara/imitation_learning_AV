#include "DarknetToVision.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "darknet_republish");
    ros::NodeHandle node_handle("~");

    aesk_darknet_republish::DarknetToVision darknet_to_vision(node_handle);

    ros::spin();
    return 0;
}