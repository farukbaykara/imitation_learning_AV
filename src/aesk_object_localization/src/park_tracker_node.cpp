#include <ros/ros.h>
#include "aesk_object_localization/ParkTracker.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "park_tracker");
    ros::NodeHandle node_handle("~");

    aesk_object_localization::ParkTracker park_tracker(node_handle);

    ros::spin();
    return 0;
}