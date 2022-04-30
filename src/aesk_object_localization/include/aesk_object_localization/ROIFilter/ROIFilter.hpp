#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "aesk_object_localization/PointCloudOpen3D.hpp"

namespace aesk_object_localization
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ROIFilter
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit ROIFilter(ros::NodeHandle& t_node_handle);

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
    void lidarCallback(const sensor_msgs::PointCloud2& t_msg);

    /*!
     * Logs boolean.
     * @param t_bool the bool.
     */
    void logBoolean(const std::string& t_field, const bool& t_bool);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! ROS lidar topic subscriber.
    ros::Subscriber m_lidar_sub;

    //! ROS lidar topic name to subscribe to.
    std::string m_lidar_topic;

    //! ROS filtered cloud publisher.
    ros::Publisher m_filtered_pub;

    //! Boundaries of the region of interest.
    std::map<std::string, double> m_roi_bounds;

    //! Whether use voxel down sampling or not.
    bool m_use_voxel;

    //! Voxel size for down sampling.
    double m_voxel_size;
};
}  // namespace aesk_object_localization
