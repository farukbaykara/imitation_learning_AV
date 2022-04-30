#pragma once

#include <algorithm>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/PointCloud2.h>
#include <vision_msgs/Detection3DArray.h>
#include "open3d/Open3D.h"
#include "aesk_object_localization/PointCloudOpen3D.hpp"
#include "Object.hpp"
#include "ObjectTracker.hpp"

namespace aesk_object_localization
{

class LidarTracker
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit LidarTracker(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Point cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud);

    /*!
     * Sign detections from SignFinder callback method.
     * @param t_detections the received marker array message.
     */
    void signDetectionsCallback(const vision_msgs::Detection3DArrayConstPtr& t_detections);

    /*!
     * Print currently tracking objects.
     */
    void printActiveObjects();

    /*!
     * Print currently tracking objects.
     */
    void printStagingArea();

    /*!
     * Delete an element from a vector by index.
     * https://stackoverflow.com/a/3487736/11495016
     * @param t_vector the vector.
     * @param t_index the index of the element to be deleted.
     */
    static void quickDelete(std::vector<Object>& t_vector, const int& t_index);

    /*!
     * Search for the sign in the calculated area.
     * @param t_x_coordinate the x coordinate of the sign.
     * @param t_y_coordinate the y coordinate of the sign.
     * @param t_z_coordinate the z coordinate of the sign.
     */
    void trackObject();

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! Point cloud subscriber.
    ros::Subscriber m_cloud_sub;

    //! Projected cloud topic name parameter.
    std::string m_cloud_topic;

    //! Sign detections subscriber.
    ros::Subscriber m_sign_detections_sub;

    //! Detected signs topic name parameter. From SignFinder. (a.k.a. old green markers.)
    std::string m_sign_detections_topic;

    //! Tracked signs publisher.
    ros::Publisher m_tracked_detections_pub;

    //! Gate publisher.
    ros::Publisher m_gate_publisher;

    //! Object count parameter.
    int m_object_count{};

    //! Boundaries of the tracker area. Objects will not be tracked if they leave this area.
    std::map<std::string, double> m_tracker_ares_bounds;

    //! The size of the area which will be used to find objects inside.
    std::map<std::string, double> m_gate_extent;

    //! The center position of the gate with respect to object.
    std::map<std::string, double> m_gate_margin;

    //! The histogram that will store object counter for each sign.
    std::array<long, 24> m_object_histogram{};

    //! Stores objects which are tracking right now.
    std::vector<Object> m_active_objects;

    //! Stores the objects which their counter isn't satisfied.
    std::vector<std::pair<Object, long>> m_staging_area;
};

}  // namespace aesk_object_localization
