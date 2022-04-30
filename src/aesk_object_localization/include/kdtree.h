#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "std_msgs/Bool.h"
#include <geometry_msgs/Pose.h>
#include <pcl/segmentation/extract_clusters.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>

#include <aesk_object_localization/ArrayOfPointCloud2s.h>
#include <aesk_object_localization/PointCloud2WithId.h>

#include <aesk_object_localization/ProjectedCloudAction.h>
#include <aesk_object_localization/ArrayOfPointCloud2s.h>

#include <aesk_object_localization/SignTrackerAction.h>

#include <actionlib/server/simple_action_server.h>

class Detection
{
  private:
    ros::NodeHandle& n;

    ros::Subscriber lidarSubscriber;

    ros::Publisher cloud_filtered_pub;
    ros::Publisher pose_array_pub;
    ros::Publisher marker_array_pub;

    sensor_msgs::PointCloud2 point_cloud;
    sensor_msgs::PointCloud2 ros_pc2_out;

    geometry_msgs::Pose pose;

    std::string sensor_model;

    double z_axis_min;
    double z_axis_max;
    int cluster_size_min;
    int cluster_size_max;

    int regions[100];

    float tolerance = 0.0;

    //! ActionServer.
    actionlib::SimpleActionServer<aesk_object_localization::SignTrackerAction> m_action_server;

    /*!
     * Action server callback method for transmitting projected point cloud to CloudFilter.
     * @param goal action goal.
     */
    void actionServerCallback(const aesk_object_localization::SignTrackerGoalConstPtr& goal);

  public:
    Detection(int argc, char** argv, ros::NodeHandle& t_node_handle);

    void pointCloudCallback(
        const aesk_object_localization::ArrayOfPointCloud2sConstPtr& cloud_sign_array);
};
