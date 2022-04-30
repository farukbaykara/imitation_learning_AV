#include <ros/ros.h>
#include "aesk_visualization/lane_2d_viz/Lane2DViz.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lane_2d_viz");
    ros::NodeHandle node_handle("~");

    aesk_visualization::Lane2DViz viz(node_handle);

    return 0;
}