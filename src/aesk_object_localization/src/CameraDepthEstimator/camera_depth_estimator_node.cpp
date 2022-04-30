#include <ros/ros.h>
#include "aesk_object_localization/CameraDepthEstimator/CameraDepthEstimator.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "viz_obj_localization");
    ros::NodeHandle node_handle("~");

    aesk_object_localization::CameraDepthEstimator depth(node_handle);

    return 0;
}