#pragma once

#include <ros/ros.h>

#include <darknet_ros_msgs/BoundingBoxes.h>

namespace aesk_darknet_republish
{
class DarknetRepublish
{
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    DarknetRepublish(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * ROS message filter callback method.
     * @param t_boxes the received Image message.
     */
    void topicCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& t_boxes);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    ros::Subscriber m_boxes_subscriber;

    //! Projected point cloud publisher.
    ros::Publisher m_boxes_publisher;

    //! Message filter subscriber for Image.
    // ImageSubscriber m_image_subscriber;
};

}  // namespace aesk_darknet_republish
