#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

#include <tf/tf.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <aesk_object_localization/ProjectedCloudAction.h>
#include <aesk_object_localization/ArrayOfPointCloud2s.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

namespace aesk_object_localization
{
class CloudPainter
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit CloudPainter(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Raw point cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void topicCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud);

    /*!
     * Action server callback method for transmitting projected point cloud to CloudFilter.
     * @param goal action goal.
     */
    void actionServerCallback(const aesk_object_localization::ProjectedCloudGoalConstPtr& goal);

    /*!
     * Wait for camera to lidar transform.
     * @param trans_velo_to_cam reference for TransformStamped message.
     */
    bool waitForCameraLidarTransform(geometry_msgs::TransformStamped& trans_velo_to_cam);

    /*!
     * Creates projection matrix.
     */
    void createMatrix();

    /*!
     * Finish line callback method.
     * @param t_is_finish the received Bool message.
     */
    void finishCallback(const std_msgs::BoolConstPtr& t_is_finish);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! ActionServer.
    actionlib::SimpleActionServer<aesk_object_localization::ProjectedCloudAction> m_action_server;

    //! ActionServer result message.
    aesk_object_localization::ProjectedCloudResult m_result;

    //! The finish line flag.
    bool m_is_finish;

    //! Lidar topic name.
    std::string m_lidar_topic;

    //! Camera topic name.
    std::string m_camera_topic;

    //! Finish line topic name.
    std::string m_finish_topic;

    //! Intrinsic parameters.
    std::vector<double> m_intrinsic_parameters;

    //! Intrinsic matrix.
    cv::Mat m_intrinsic_matrix;

    //! Projected point cloud publisher.
    ros::Publisher m_point_cloud_pub;

    //! Raw lidar point cloud subscriber.
    ros::Subscriber m_point_cloud_sub;

    //! Raw lidar point cloud subscriber.
    ros::Subscriber m_finish_sub;

    //! Tf2 stuff for extrinsic matrix.
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;

    //! Region of interest.
    std::map<std::string, float> m_region_of_interest;

    //! Region of interest after finish line.
    std::map<std::string, float> m_region_of_interest_finish;

    //! Camera matrix to project 3D points to 2D image plane.
    Eigen::Matrix4d m_mat_point_transformer;

    //! Image width.
    int m_image_width;

    //! Image height.
    int m_image_height;

    //! This is the name of camera frame to get extrinsic params.
    std::string m_camera_frame_id;
};

}  // namespace aesk_object_localization
