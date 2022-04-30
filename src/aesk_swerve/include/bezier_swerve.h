#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/tf.h>
#include <math.h>
#include <tf/LinearMath/Scalar.h>

#include "lgsvl_msgs/Detection3DArray.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include "aesk_swerve/BezierGoal.h"
#include "aesk_swerve/BezierAction.h"

namespace aesk_bezier_swerve
{
class Swerve
{
  public:
    /*!
     * Constructor.
     * @param nodeHandle the ROS node handle.
     */
    Swerve(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Visualizes control points and waypoints.
     * @param t_visualization_method to choose what to visualize.
     */
    void visualize(const std::string& t_visualization_method,
                   const std::vector<geometry_msgs::Point>& t_bezier_control_points,
                   const std::vector<geometry_msgs::Point>& t_bezier_curve_points);

    /*!
     * Gets the distance between two pints.
     * @return float distance.
     */
    float get_distance(float x1, float y1, float x2, float y2);

    /*!
     * Obstacle callback method.
     * @param t_Obstacle the received message.
     */
    void obstacleCallback(const lgsvl_msgs::Detection3DArray& t_Obstacle);

    /*!
     * Odometry callback method.
     * @param t_Odom the received message.
     */
    void odomCallback(const nav_msgs::Odometry& t_Odom);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! Obstacle topic name.
    std::string m_obstacle_topic;

    //! Output topic name.
    std::string m_output_topic;

    //! Path marker topic name.
    std::string m_path_marker_topic;

    //! Control points marker topic name.
    std::string m_points_marker_topic;

    //! Odometry topic name.
    std::string m_odom_topic;

    //! Obstacle subscriber.
    ros::Subscriber m_obstacle_sub;

    //! Odometry subscriber.
    ros::Subscriber m_odom_sub;

    //! Waypoint publisher.
    ros::Publisher m_waypoint_pub;

    //! Path marker publisher.
    ros::Publisher m_path_marker_pub;

    //! Control points marker publisher.
    ros::Publisher m_points_marker_pub;

    //! Distance to detect the obstacle.
    float m_obstacle_dist;

    //! Distance to offset.
    float m_offset;

    //! Distance to offset.
    std::string m_visualization_method;

    //! Initilization parameter.
    bool m_init = true;

    //! Current Pose of the obstacle.
    geometry_msgs::Pose m_current_pose;

    //! Action client for park sign clustering
    actionlib::SimpleActionClient<aesk_swerve::BezierAction> m_action_client_bezier;

    //! Stored yaw of the car.
    tfScalar m_old_car_yaw;

    //! Stored roll of the car.
    tfScalar m_old_car_roll;

    //! Stored pitch of the car.
    tfScalar m_old_car_pitch;

    //! Yaw of the car.
    tfScalar m_car_yaw;

    //! Roll of the car.
    tfScalar m_car_roll;

    //! Pitch of the car.
    tfScalar m_car_pitch;
};

}  // namespace aesk_bezier_swerve