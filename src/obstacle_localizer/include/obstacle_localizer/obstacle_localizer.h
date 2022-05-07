#pragma once

#include <ros/ros.h>


#include <tf/tf.h>
#include <math.h>
#include <tf/LinearMath/Scalar.h>


#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Bool.h"

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
        void obstacleMarkerCallback(const visualization_msgs::MarkerArray &t_obstacleMarker_msg);

        /*!
         * Obstacle callback method.
         * @param t_Obstacle the received message.
         */
        void obstaclePublisher();

        //! ROS node handle.
        ros::NodeHandle &m_node_handle;

        //! ROS topic subscriber.
        ros::Subscriber m_obstacleMarker_sub;

        //! ROS topic publisher.
        ros::Publisher m_obstacle_pub;
    };

} /* obstacle_localizer namespace */