#include <ros/ros.h>
#include "aesk_object_localization/lane3d_combiner/Lane3DCombiner.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane3d_combiner");
    ros::NodeHandle node_handle("~");

    aesk_object_localization::Lane3DCombiner lane3d_combiner{ node_handle };

    ros::spin();
    return 0;
}