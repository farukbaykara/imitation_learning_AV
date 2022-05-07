#include <ros/ros.h>
#include <iostream>

#include "obstacle_localizer/obstacle_localizer.h"

int main(int argc , char **argv) 
{
    ros::init(argc, argv, "obstacle_localizer");
    ros::NodeHandle node_handle("~");

    obstacle_localizer::Car Car(node_handle);


    ros::spin();
    return 0;
}