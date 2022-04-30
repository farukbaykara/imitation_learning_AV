#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace aesk_object_localization
{

using policy_t = message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                                 sensor_msgs::PointCloud2>;

class Lane3DCombiner
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit Lane3DCombiner(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * The callback method for lanes.
     * @param t_left the left PointCloud2 message.
     * @param t_right the right PointCloud2 message.
     */
    void lane3DCallback(const sensor_msgs::PointCloud2ConstPtr& t_left,
                        const sensor_msgs::PointCloud2ConstPtr& t_right);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! ROS left lane topic subscriber.
    message_filters::Subscriber<sensor_msgs::PointCloud2> m_left_sub;

    //! ROS right lane topic subscriber.
    message_filters::Subscriber<sensor_msgs::PointCloud2> m_right_sub;

    // Message Filter synchronizer.
    message_filters::Synchronizer<policy_t> m_synchronizer;

    //! ROS left lane topic name to subscribe to.
    std::string m_left_topic;

    //! ROS right lane topic name to subscribe to.
    std::string m_right_topic;

    //! ROS filtered cloud publisher.
    ros::Publisher m_combined_pub;
};
}  // namespace aesk_object_localization
