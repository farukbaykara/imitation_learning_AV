#include <ros/ros.h>
#include <sensor_msgs/Image.h>

/**
 * Subscriber callback
 */
void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {

    // Get a pointer to the depth values casting the data
    // pointer to floating point
    float* depths = (float*)(&msg->data[0]);

    // Image coordinates of the center pixel
    int u = msg->width / 2;
    int v = msg->height / 2;

    // Linear index of the center pixel
    int centerIdx = u + msg->width * v;

    // Output the measure
    ROS_INFO("Center distance : %g m", depths[centerIdx]);
}

/**
 * Node main function
 */
int main(int argc, char** argv) {
    // Node initialization
    ros::init(argc, argv, "zed_video_subscriber");
    ros::NodeHandle n;

    // Depth topic subscriber
    ros::Subscriber subDepth = n.subscribe("/zed/zed_node/depth/depth_registered", 10,
                                           depthCallback);

    // Node execution
    ros::spin();

    return 0;
}
