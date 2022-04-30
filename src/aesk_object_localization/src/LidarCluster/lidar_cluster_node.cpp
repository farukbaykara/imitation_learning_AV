#include <ros/ros.h>
#include "aesk_object_localization/LidarCluster/LidarCluster.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_cluster");
    ros::NodeHandle node_handle("~");

    aesk_object_localization::LidarCluster cluster{ node_handle };

    ros::spin();
    return 0;
}