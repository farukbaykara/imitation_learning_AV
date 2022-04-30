#include <ros/ros.h>
#include "aesk_object_localization/sign_finder/SignFinder.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sign_finder");
    ros::NodeHandle node_handle("~");

    aesk_object_localization::SignFinder sign_finder{ node_handle };

    ros::spin();
    return 0;
}