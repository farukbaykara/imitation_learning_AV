
#include <ros/ros.h>

#include "obstacle_localizer/obstacle_localizer.h"


namespace obstacle_localizer
{



    Car::Car(ros::NodeHandle& t_node_handle)
        : m_node_handle(t_node_handle)
    {

        m_obstacleMarker_sub = m_node_handle.subscribe("/obstacles_markers",1,&Car::obstacleMarkerCallback,this);


        m_obstacle_pub = m_node_handle.advertise<std_msgs::Bool>("new_topic", 1000);




    }


    void Car::obstacleMarkerCallback(const visualization_msgs::MarkerArray &t_obstacleMarker_msg){


        
        int a = 2 +2 ;

        obstaclePublisher();
        
    }


    void Car::obstaclePublisher(){

        std_msgs::Bool boolData;

        boolData.data = 1;

        
        m_obstacle_pub.publish(boolData);




    }




}