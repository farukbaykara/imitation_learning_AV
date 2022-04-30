#include <ros/ros.h>
#include "aesk_visualization/projected_cloud_viz/ProjectedCloudViz.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "projected_cloud_viz");
    ros::NodeHandle node_handle("~");

    aesk_visualization::ProjectedCloudViz viz(node_handle);

    return 0;
}