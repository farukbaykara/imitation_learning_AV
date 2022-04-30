#include <ros/ros.h>
#include "aesk_object_localization/ROIFilter/ROIFilter.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "roi_filter");
    ros::NodeHandle node_handle("~");

    aesk_object_localization::ROIFilter roi_filter{ node_handle };

    ros::spin();
    return 0;
}