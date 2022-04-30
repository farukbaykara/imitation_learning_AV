#pragma once
#include <random>  // for std::mt19937
#include <ctime>   // for std::time
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include "aesk_object_localization/PointCloudOpen3D.hpp"

namespace aesk_object_localization
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class LidarCluster
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit LidarCluster(ros::NodeHandle& t_node_handle);

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
     * Creates ROS Header with "velodyne" fixed frame and ros::Time now.
     */
    static std_msgs::Header createHeader();

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! ROS lidar topic subscriber.
    ros::Subscriber m_lidar_sub;

    //! ROS lidar topic name to subscribe to.
    std::string m_lidar_topic;

    //! Clustered cloud publisher.
    ros::Publisher m_detections_pub;

    //! DBSCAN epsilon parameter.
    double m_eps{};

    //! DBSCAN min_points parameter.
    int m_min_points{};

    //! For random number generation.
    std::mt19937 m_mersenne;
    std::uniform_int_distribution<> m_die;
};
}  // namespace aesk_object_localization
