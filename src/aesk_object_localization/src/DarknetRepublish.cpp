#include "aesk_object_localization/DarknetRepublish.hpp"

namespace aesk_darknet_republish
{
DarknetRepublish::DarknetRepublish(ros::NodeHandle& t_node_handle) : m_node_handle(t_node_handle)
{
    m_boxes_subscriber = m_node_handle.subscribe<darknet_ros_msgs::BoundingBoxes>(
        "/darknet_ros/bounding_boxes", 1, &DarknetRepublish::topicCallback, this);

    m_boxes_publisher = m_node_handle.advertise<darknet_ros_msgs::BoundingBoxes>("darknet_republished", 1);

    ROS_INFO("Successfully launched node.");
}

void DarknetRepublish::topicCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& t_boxes)
{
    darknet_ros_msgs::BoundingBoxes my_box;
    my_box.header = t_boxes->image_header;
    my_box.image_header = t_boxes->image_header;
    my_box.bounding_boxes = t_boxes->bounding_boxes;

    std::cout << "------------------------\n";
    std::cout << "Normal header:\n" << my_box.header << '\n';
    std::cout << "Image header:\n" << my_box.image_header << '\n';

    m_boxes_publisher.publish(my_box);

    std::cout << "***********************************\n";
    std::cout << "Normal header:\n" << my_box.header << '\n';
    std::cout << "Image header:\n" << my_box.image_header << '\n';
}
}  // namespace aesk_darknet_republish