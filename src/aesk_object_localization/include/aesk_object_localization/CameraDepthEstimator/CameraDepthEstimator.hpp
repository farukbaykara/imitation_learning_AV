#pragma once

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "open3d/Open3D.h"

namespace aesk_object_localization
{
class CameraDepthEstimator
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit CameraDepthEstimator(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Projected cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void projectedCloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud);

    /*!
     * Lane 2D cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void laneCloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud);

    /*!
     * Lane 2D cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void estimateDepth();

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! Projected cloud topic name.
    std::string m_point_cloud_topic;

    //! Drivable area topic name.
    std::string m_lane_cloud_topic;

    //! Projected point cloud subscriber.
    ros::Subscriber m_point_cloud_sub;

    //! Drivable area point cloud subscriber.
    ros::Subscriber m_lane_cloud_sub;

    //! Lane 3D cloud publisher.
    ros::Publisher m_point_cloud_pub;

    //! Stores Open3D cloud.
    std::shared_ptr<open3d::geometry::PointCloud> m_o3d_cloud;

    //! Stores 2D Lane cloud.
    sensor_msgs::PointCloud2ConstPtr m_lane_cloud;

    //! Flag for receiving projected point cloud.
    bool m_is_cloud_received{};

    //! Flag for receiving lane point cloud.
    bool m_is_lane_received{};

    //! Nearest neighbour search radius.
    double m_search_radius;
};

}  // namespace aesk_object_localization