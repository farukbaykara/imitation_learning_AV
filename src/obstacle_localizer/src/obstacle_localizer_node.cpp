
#include <ros/ros.h>

#include "obstacle_localizer/obstacle_localizer.h"

namespace obstacle_localizer
{

    Car::Car(ros::NodeHandle &t_node_handle)
        : m_node_handle(t_node_handle)
    {

        m_obstacleMarker_sub = m_node_handle.subscribe("/obstacles_markers", 1, &Car::obstacleMarkerCallback, this);

        m_obstacle_pub = m_node_handle.advertise<geometry_msgs::PoseStamped>("/obstacle_pose", 1000);
    }

    void Car::obstacleMarkerCallback(const visualization_msgs::MarkerArray::ConstPtr &t_obstacleMarker_msg)
    {

        float t_poseSum_x;
        float t_poseSum_y;
        float t_markerPoint_size;
        int t_markerArray_size = t_obstacleMarker_msg->markers.size();

        std::vector<visualization_msgs::Marker> t_marker_point;
        std::cout<< t_obstacleMarker_msg->markers[0].points[0]<<std::endl;

        //t_poseSum_y = t_obstacleMarker_msg->markers[0].points[0].y;

        // std::cout<<t_poseSum_x<<std::endl;
        //  std::cout<<t_poseSum_y<<std::endl;
        //  std::cout<<t_obstacleMarker_msg->markers.size()<<std::endl;
        float t_poseAvg_x = t_poseSum_x;

        float t_poseAvg_y = t_poseSum_y;

        obstaclePublisher(t_poseAvg_x, t_poseAvg_y);
    }

    void Car::obstaclePublisher(float t_pose_x, float t_pose_y)
    {

        m_obstaclePose.header.frame_id = "base_scan";
        m_obstaclePose.pose.position.x = t_pose_x;
        m_obstaclePose.pose.position.y = t_pose_y;

        m_obstacle_pub.publish(m_obstaclePose);
    }

}