#include <ros/ros.h>
#include "aesk_visualization/detection_3d_viz/Detection3DViz.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_3d_viz");
    ros::NodeHandle node_handle("~");

    aesk_visualization::Detection3DViz viz{ node_handle };

    ros::spin();
    return 0;
}