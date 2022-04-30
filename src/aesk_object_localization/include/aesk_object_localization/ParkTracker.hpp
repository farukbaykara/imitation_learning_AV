#pragma once

#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <aesk_object_localization/ParkAction.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

namespace aesk_object_localization
{
class ParkTracker
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    ParkTracker(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Finish line check callback method.
     * @param t_is_finished the received finish bool message.
     */
    void finishCallback(const std_msgs::BoolConstPtr& t_is_finished);

    /*!
     * Sign markers callback method.
     * @param t_marker_array the received marker array message.
     */
    void signCallback(const visualization_msgs::MarkerArrayConstPtr& t_marker_array);

    /*!
     * Projected point cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud);

    /*!
     * Calculate the distance between lidar and traffic sign.
     * @param t_x_coordinate the x coordinate of the sign.
     * @param t_y_coordinate the y coordinate of the sign.
     * @param t_z_coordinate the z coordinate of the sign.
     */
    double calculateDistance(const double& t_x_coordinate, const double& t_y_coordinate,
                             const double& t_z_coordinate);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! Action client for clustering
    actionlib::SimpleActionClient<aesk_object_localization::ParkAction> m_action_client;

    //! Finish line subscriber.
    ros::Subscriber m_finish_line_subscriber;

    //! Sign markers subscriber.
    ros::Subscriber m_sign_markers_subscriber;

    //! Projected point cloud subscriber.
    ros::Subscriber m_projected_cloud_subscriber;

    //! Park sign marker publisher.
    ros::Publisher m_park_sign_publisher;

    //! The area which park sign will be search publisher.
    ros::Publisher m_area_publisher;

    //! Publish the point cloud inside the box.
    ros::Publisher m_point_cloud_publisher;

    //! Projected cloud topic name.
    std::string m_cloud_topic;

    //! Finish line topic name.
    std::string m_finish_topic;

    //! Finish line topic name.
    std::string m_sign_markers_topic;

    //! Park sign YOLO id.
    int m_park_id;

    //! Object count
    int m_object_count;

    //! Tracker box.
    std::map<std::string, float> m_tracker_box;

    //! Finish flag.
    bool m_is_finished;

    //! Object counter.
    int m_object_counter;

    //! X coordinate of last park sign.
    double m_last_x;

    //! Y coordinate of last park sign.
    double m_last_y;

    //! Z coordinate of last park sign.
    double m_last_z;
};

}  // namespace aesk_object_localization
