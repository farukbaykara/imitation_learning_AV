#include <ros/ros.h>
#include "aesk_object_localization/DarknetRepublish.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "darknet_republish");
    ros::NodeHandle node_handle("~");

    aesk_darknet_republish::DarknetRepublish darknet_republish(node_handle);

    ros::spin();
    return 0;
}