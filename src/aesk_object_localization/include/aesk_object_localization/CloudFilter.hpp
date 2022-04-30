#pragma once

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <aesk_object_localization/ProjectedCloudAction.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

namespace aesk_object_localization
{
class CloudFilter
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    CloudFilter(ros::NodeHandle& t_node_handle);

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

    //! Action client to get the most recent projected point cloud.
    actionlib::SimpleActionClient<aesk_object_localization::ProjectedCloudAction> m_action_client;

    //! Lidar topic name.
    std::string m_lidar_topic;

    //! YOLO boundind boxes topic name.
    std::string m_yolo_topic;

    //! Subscriber for YOLO BoundingBoxes topic.
    ros::Subscriber m_yolo_subscriber;

    //! Publisher for ArrayOfPointCloud2s.
    ros::Publisher m_clouds_publisher;

    //! Extended bounding box x_min value.
    int m_extended_x_min;

    //! Extended bounding box x_max value.
    int m_extended_x_max;

    //! Extended bounding box y_min value.
    int m_extended_y_min;
    
    //! Extended bounding box y_max value.
    int m_extended_y_max;
};

}  // namespace aesk_object_localization
