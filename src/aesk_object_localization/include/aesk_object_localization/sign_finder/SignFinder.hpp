#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection2DArray.h>
#include <vision_msgs/Detection3DArray.h>
#include "aesk_object_localization/PointCloudOpen3D.hpp"
#include "aesk_object_localization/lidar_tracker/Object.hpp"

namespace aesk_object_localization
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class SignFinder
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit SignFinder(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * ROS lidar topic callback method.
     * @param t_msg the received PointCloud2 message.
     */
    void projectedCloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_msg);

    /*!
     * ROS lidar topic callback method.
     * @param t_msg the received PointCloud2 message.
     */
    void detection2dCallback(const vision_msgs::Detection2DArray& t_msg);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! ROS lidar topic subscriber.
    ros::Subscriber m_lidar_sub;

    //! ROS lidar topic name to subscribe to.
    std::string m_lidar_topic;

    //! Detection 2D topic subscriber.
    ros::Subscriber m_detection2d_sub;

    //! Detection 2D topic name to subscribe to.
    std::string m_detection2d_topic;

    //! ROS filtered cloud publisher.
    ros::Publisher m_detection3d_pub;
    ros::Publisher m_pc2_pub;

    //! Stores detected objects coming from m_detection2d_topic.
    vision_msgs::Detection2DArray m_detection2d_array;

    //! Stores projected point cloud.
    sensor_msgs::PointCloud2 m_projected_cloud;
};
}  // namespace aesk_object_localization
