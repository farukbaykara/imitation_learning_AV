#pragma once

#include <ros/ros.h>

#include <iostream>
#include <tf/tf.h>
#include <math.h>
#include <tf/LinearMath/Scalar.h>
#include <vector>

#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/PointStamped.h"

namespace obstacle_localizer
{
    class Car
    {

    public:
        /*!
         * Constructor.
         * @param nodeHandle the ROS node handle.
         */
        Car(ros::NodeHandle &t_node_handle);

    private:
        /*!
         * Obstacle callback method.
         * @param t_Obstacle the received message.
         */
        void obstacleMarkerCallback(const visualization_msgs::MarkerArray::ConstPtr &t_obstacleMarker_msg);

        /*!
         * Localized Obstacle publisher method.
         * @param t_Obstacle the received message.
         */
        void obstaclePublisher(float t_pose_x ,float t_pose_y);

        //! ROS node handle.
        ros::NodeHandle &m_node_handle;

        //! ROS topic subscriber.
        ros::Subscriber m_obstacleMarker_sub;

        //! ROS topic publisher.
        ros::Publisher m_obstacle_pub;

        //! Incoming marker array points.
        std::vector<visualization_msgs::MarkerArray> m_incoming_obstacleArray;
        
        //! Incoming marker array size.
        geometry_msgs::PoseStamped m_obstaclePose;
    };

} /* obstacle_localizer namespace */