#pragma once

#include <ros/ros.h>
#include <random>
#include <vision_msgs/Detection3DArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include "aesk_object_localization/PointCloudOpen3D.hpp"

namespace aesk_visualization
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class Detection3DViz
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit Detection3DViz(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Detection 3D array topic callback method.
     * @param t_msg the received Detection3DArray message.
     */
    void detectionCallback(const vision_msgs::Detection3DArray& t_msg);

    /*!
     * Create random RGB color as Eigen Vector3D.
     */
    Eigen::Vector3d createRandomColor();

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! ROS detection topic subscriber.
    ros::Subscriber m_detection_sub;

    //! ROS detection topic name to subscribe to.
    std::string m_detection_topic;

    //! Marker publisher.
    ros::Publisher m_marker_pub;

    //! PointCloudOpen3D publisher.
    ros::Publisher m_cloud_pub;

    //! Random number stuff.
    std::random_device m_dev;
    std::mt19937 m_rng;
    std::uniform_real_distribution<double> m_dist255;
};
}  // namespace aesk_visualization
