#include <ros/ros.h>
#include <iostream>

#include "bezier_swerve.h"

int main(int argc , char **argv) 
{
    ros::init(argc, argv, "bezier_swerve");
    ros::NodeHandle node_handle("~");

    aesk_bezier_swerve::Swerve Swerve(node_handle);

    ros::spin();
    return 0;
}

