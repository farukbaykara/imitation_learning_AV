#include <ros/ros.h>
#include "aesk_object_localization/CloudFilter.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_painter");
    ros::NodeHandle node_handle("~");

    aesk_object_localization::CloudFilter cloud_filter(node_handle);

    ros::spin();
    return 0;
}