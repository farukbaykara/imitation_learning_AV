#include <iostream>
#include <ros/ros.h>

#include "vehicle2ros/vehicle2ros.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "vehicle2ros");
    ros::NodeHandle node_handle("~");

    

    car::Car car(node_handle);

    ros::spin();

    return 0;
}