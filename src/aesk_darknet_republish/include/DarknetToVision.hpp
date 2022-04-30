#pragma once

#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection2D.h>
#include <vision_msgs/BoundingBox2D.h>

namespace aesk_darknet_republish
{
class DarknetToVision
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    DarknetToVision(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * YOLO BoundingBoxes callback method.
     * @param t_yolo_boxes the received PointCloud2 message.
     */
    void yoloTopicCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& t_yolo_boxes);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! YOLO boundind boxes topic name.
    std::string m_yolo_topic;

    //! Subscriber for YOLO BoundingBoxes topic.
    ros::Subscriber m_yolo_subscriber;

    //! Publisher for Vision Message
    ros::Publisher m_vision_publisher;
};

}  // namespace aesk_darknet_republish