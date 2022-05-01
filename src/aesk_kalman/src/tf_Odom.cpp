#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>

rosgraph_msgs::Clock time_;

bool init = false;
nav_msgs::Odometry::ConstPtr first_odom;


void odomCallBack(const nav_msgs::Odometry::ConstPtr& msg)
{

    if (!init)
    {
        first_odom = msg;
        init = true;
    }

    ros::Time clck = ros::Time::now();
    static tf::TransformBroadcaster br_base;
    static tf::TransformBroadcaster br_gps;
    static tf::TransformBroadcaster br_odom;
    static tf::TransformBroadcaster br_imu;
    tf::Transform transform;
    tf::Transform transform_map;
    tf::Transform transform_gps;
    tf::Transform transform_imu;

    const geometry_msgs::Point pos{first_odom->pose.pose.position};
    const geometry_msgs::Quaternion rot{first_odom->pose.pose.orientation};
    const geometry_msgs::Point pos_msg{msg->pose.pose.position};
    const geometry_msgs::Quaternion rot_msg{msg->pose.pose.orientation};

    // transform_map.setOrigin( tf::Vector3((pos.x), (pos.y), (pos.z)) );
    // transform_map.setRotation( tf::Quaternion(0, 0,0, 1) );
    // br_odom.sendTransform(tf::StampedTransform(transform_map, clck, "map", "odom"));

    // transform.setOrigin(tf::Vector3(pos_msg.x-pos.x,
    //                                 pos_msg.y-pos.y,
    //                                 pos_msg.z-pos.z));

    // transform.setRotation(tf::Quaternion(0, 0,0, 1));
    // br_base.sendTransform(tf::StampedTransform(transform, clck, "odom", "base_link"));
    

    


    // transform_gps.setOrigin( tf::Vector3(0, 0, 0) );
    // transform_gps.setRotation( tf::Quaternion(0,0,0,1) );
    // br_gps.sendTransform(tf::StampedTransform(transform_gps, clck, "base_link", "gps"));

    // transform_imu.setOrigin( tf::Vector3(0, 0, 0) );
    // transform_imu.setRotation( tf::Quaternion(0,0,0,1) );
    // br_imu.sendTransform(tf::StampedTransform(transform_imu, clck, "base_link", "imu"));

}


void clockCallBack(const rosgraph_msgs::Clock::ConstPtr& msg)
{
  time_=*msg;

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf2Odom");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/odom", 1, odomCallBack);
  ros::Subscriber sub2 = n.subscribe("/clock", 1, clockCallBack);
  ros::Rate r(10);
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }
   
  return 0;
};