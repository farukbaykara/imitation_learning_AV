#include <ros/ros.h>
#include "aesk_object_localization/CloudPainter.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_painter");
    ros::NodeHandle node_handle_cloud("~");
    // ros::NodeHandle node_handle_yolo("~");

    // aesk_object_localization::CloudPainter cloud_painter(node_handle_cloud, node_handle_yolo);
    aesk_object_localization::CloudPainter cloud_painter(node_handle_cloud);

    return 0;
}