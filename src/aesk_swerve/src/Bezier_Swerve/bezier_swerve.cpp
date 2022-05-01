#include <ros/ros.h>

#include "bezier_swerve.h"

namespace aesk_bezier_swerve
{
Swerve::Swerve(ros::NodeHandle& t_node_handle)
  : m_node_handle(t_node_handle), m_action_client_bezier("bezier_action_server", true)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    if (m_visualization_method != "all" && m_visualization_method != "points" &&
        m_visualization_method != "path")
    {
        m_visualization_method = "path";
    }

    ROS_INFO("Obstacle topic: : %s", m_obstacle_topic.c_str());

    ROS_INFO("Output topic: : %s", m_output_topic.c_str());

    ROS_INFO("Waiting for action server to start.");
    m_action_client_bezier.waitForServer();
    ROS_INFO("Action server started.");

    m_obstacle_sub = m_node_handle.subscribe(m_obstacle_topic, 10, &Swerve::obstacleCallback, this);

    m_odom_sub = m_node_handle.subscribe(m_odom_topic, 10, &Swerve::odomCallback, this);

    m_waypoint_pub = m_node_handle.advertise<nav_msgs::Path>(m_output_topic, 1000);

    m_path_marker_pub =
        m_node_handle.advertise<visualization_msgs::MarkerArray>(m_path_marker_topic, 1000);

    m_points_marker_pub =
        m_node_handle.advertise<visualization_msgs::MarkerArray>(m_points_marker_topic, 1000);

    ROS_INFO("Successfully launched node.");
}

bool Swerve::readParameters()
{
    if (!m_node_handle.getParam("obstacle_topic", m_obstacle_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("output_topic", m_output_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("path_marker_topic", m_path_marker_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("point_marker_topic", m_points_marker_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("odom_topic", m_odom_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("obstacle_dist", m_obstacle_dist))
    {
        return false;
    }
    if (!m_node_handle.getParam("offset", m_offset))
    {
        return false;
    }
    if (!m_node_handle.getParam("visualization_method", m_visualization_method))
    {
        return false;
    }
    return true;
}

void Swerve::odomCallback(const nav_msgs::Odometry& t_Odom)
{
    tf::Quaternion q(t_Odom.pose.pose.orientation.x, t_Odom.pose.pose.orientation.y,
                     t_Odom.pose.pose.orientation.z, t_Odom.pose.pose.orientation.w);
    tf::Matrix3x3 t_car_quaterninon(q);

    t_car_quaterninon.getRPY(m_car_roll, m_car_pitch,
                             m_car_yaw);  // gets yaw,pitch ,roll from quaternions.
}

void Swerve::obstacleCallback(const lgsvl_msgs::Detection3DArray& t_Obstacle)
{
    if (!t_Obstacle.detections.empty())
    {
        if (m_init && t_Obstacle.detections[0].bbox.position.position.x < m_obstacle_dist)
        {
            if (t_Obstacle.detections[0].label == "12")
            {
                m_old_car_yaw = m_car_yaw;
                m_old_car_roll = m_car_roll;
                m_old_car_pitch = m_car_pitch;

                m_init = false;
                ROS_INFO("Saved the first pose");
            }
        }
    }

    if (!m_init)
    {
        m_current_pose = t_Obstacle.detections[0].bbox.position; //kutunun x y si 

        tfScalar t_roll_change, t_pitch_change, t_yaw_change;
        t_yaw_change = (m_car_yaw - m_old_car_yaw);
        t_roll_change = (m_car_roll - m_old_car_roll);
        t_pitch_change = (m_car_pitch - m_old_car_pitch);

        std::cout << t_yaw_change << std::endl;

        m_old_car_yaw = m_car_yaw;
        m_old_car_roll = m_car_roll;
        m_old_car_pitch = m_car_pitch;

        tf::Matrix3x3 t_rotation_matrix;
        t_rotation_matrix.setEulerYPR(t_yaw_change, t_pitch_change, t_roll_change);

        tf::Point t_position_matrix, t_fixed_position;
        t_position_matrix.setX(m_current_pose.position.x);
        t_position_matrix.setY(m_current_pose.position.y + m_offset);
        t_position_matrix.setZ(m_current_pose.position.z);

        t_fixed_position = t_rotation_matrix * t_position_matrix;

        if (m_current_pose.position.x < m_obstacle_dist)
        {
            //! Bezier control points vector.
            std::vector<geometry_msgs::Point> t_bezier_control_points;

            //! Bezier curve vector.
            std::vector<geometry_msgs::Point> t_bezier_curve_points;

            geometry_msgs::Point t_first_point;
            t_first_point.x = 0.0;
            t_first_point.y = 0.0;
            t_bezier_control_points.push_back(t_first_point);

            geometry_msgs::Point t_second_point;
            t_second_point.x = m_current_pose.position.x / 2;
            t_second_point.y = 0.0;
            t_bezier_control_points.push_back(t_second_point);

            geometry_msgs::Point t_third_point;
            t_third_point.x = m_current_pose.position.x / 2;
            t_third_point.y = m_current_pose.position.y + m_offset;
            t_bezier_control_points.push_back(t_third_point);

            geometry_msgs::Point t_final_point;
            t_final_point.x = t_fixed_position[0];
            t_final_point.y = t_fixed_position[1];
            t_bezier_control_points.push_back(t_final_point);

            aesk_swerve::BezierGoal t_bezier_goal;
            t_bezier_goal.control_points = t_bezier_control_points;
            ROS_INFO("Created bezier control points.");

            m_action_client_bezier.sendGoal(t_bezier_goal);

            bool finished_before_timeout = m_action_client_bezier.waitForResult(ros::Duration(0.2));

            if (!finished_before_timeout)
            {
                ROS_INFO("Action did not finish before the time out.");
            }
            else
            {
                t_bezier_curve_points = m_action_client_bezier.getResult()->path;
                ROS_INFO("Got Bezier curve from action server.");
            }

            visualize(m_visualization_method, t_bezier_control_points, t_bezier_curve_points);

            nav_msgs::Path t_pose_array;

            for (auto point : t_bezier_curve_points)
            {
                geometry_msgs::PoseStamped t_temp_point;

                t_temp_point.pose.position.x = point.x;

                t_temp_point.pose.position.y = point.y;

                t_temp_point.pose.position.z = point.z;

                t_pose_array.poses.push_back(t_temp_point);
            }

            m_waypoint_pub.publish(t_pose_array);
        }
    }
}

void Swerve::visualize(const std::string& t_visualization_method,
                       const std::vector<geometry_msgs::Point>& t_bezier_control_points,
                       const std::vector<geometry_msgs::Point>& t_bezier_curve_points)
{
    if (t_visualization_method == "all" || t_visualization_method == "path")
    {
        visualization_msgs::MarkerArray t_path_marker_array;
        int t_index = 0;
        for (auto point : t_bezier_curve_points)
        {
            visualization_msgs::Marker point_marker;
            point_marker.header.frame_id = "velodyne";
            point_marker.header.stamp = ros::Time::now();
            point_marker.id = t_index++;
            point_marker.type = point_marker.SPHERE;
            point_marker.action = point_marker.ADD;
            point_marker.pose.position.x = point.x;
            point_marker.pose.position.y = point.y;
            point_marker.pose.position.z = point.z;
            point_marker.pose.orientation.x = 0;
            point_marker.pose.orientation.y = 0;
            point_marker.pose.orientation.z = 0;
            point_marker.pose.orientation.w = 1;
            point_marker.scale.x = 0.8;
            point_marker.scale.y = 0.8;
            point_marker.scale.z = 0.8;
            point_marker.color.a = 1;
            point_marker.color.r = 1;
            point_marker.color.g = 0;
            point_marker.color.b = 0;
            point_marker.lifetime = ros::Duration(0.2);

            t_path_marker_array.markers.push_back(point_marker);
        }
        m_path_marker_pub.publish(t_path_marker_array);

        t_index = 0;
    }

    if (t_visualization_method == "all" || t_visualization_method == "points")
    {
        visualization_msgs::MarkerArray t_points_marker_array;
        int t_index = 0;
        for (auto point : t_bezier_control_points)
        {
            visualization_msgs::Marker point_marker;
            point_marker.header.frame_id = "velodyne";
            point_marker.header.stamp = ros::Time::now();
            point_marker.id = t_index++;
            point_marker.type = point_marker.SPHERE;
            point_marker.action = point_marker.ADD;
            point_marker.pose.position.x = point.x;
            point_marker.pose.position.y = point.y;
            point_marker.pose.position.z = point.z;
            point_marker.pose.orientation.x = 0;
            point_marker.pose.orientation.y = 0;
            point_marker.pose.orientation.z = 0;
            point_marker.pose.orientation.w = 1;
            point_marker.scale.x = 0.8;
            point_marker.scale.y = 0.8;
            point_marker.scale.z = 0.8;
            point_marker.color.a = 1;
            point_marker.color.r = 0;
            point_marker.color.g = 1;
            point_marker.color.b = 1;
            point_marker.lifetime = ros::Duration(0.2);

            t_points_marker_array.markers.push_back(point_marker);
        }
        m_points_marker_pub.publish(t_points_marker_array);

        t_index = 0;
    }
}

float Swerve::get_distance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}

}  // namespace aesk_bezier_swerve