#include <ros/ros.h>
#include "aesk_object_localization/lidar_tracker/LidarTracker.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_tracker");
    ros::NodeHandle node_handle("~");

    aesk_object_localization::LidarTracker lidar_tracker{ node_handle };

    ros::spin();
    return 0;
}