#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <math.h>
#include <iterator>
#include <random>
#include <stdlib.h>
#include <time.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf/transform_broadcaster.h>
#include <stdio.h>

#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>




#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

class odomTranslator
{

private:
    double noise;
    std_msgs::Float64 msg;
    nav_msgs::Odometry odom_trans;
    nav_msgs::Odometry odom;
    sensor_msgs::Imu imu;
    sensor_msgs::Imu imu_trans;
    nav_msgs::Odometry gps;
    nav_msgs::Odometry gps_trans;

    ros::Subscriber odom_trans_sub;
    ros::Subscriber imu_sub;
    ros::Subscriber gps_sub;

    ros::Publisher odom_trans_pub;
    ros::Publisher imu_trans_pub;
    ros::Publisher gps_trans_pub;
   
   

    ros::NodeHandle *nh;

    nav_msgs::Odometry roll, pitch, yaw;
    tfScalar roll_tf, pitch_tf, yaw_tf;
    bool init = false;
    nav_msgs::Odometry::ConstPtr first_odom;


    tf::Transform transform;
    

public:
    ros::Time time = ros::Time::now();
    
    odomTranslator(ros::NodeHandle *nh)
    {

        odom_trans_pub = nh->advertise<nav_msgs::Odometry>("/odom_trans", 30);
        imu_trans_pub = nh->advertise<sensor_msgs::Imu>("/imu_trans", 30);
        gps_trans_pub = nh->advertise<nav_msgs::Odometry>("/gps_trans", 30);
        odom_trans_sub = nh->subscribe("/odom", 30, &odomTranslator::odomCallback, this);
        imu_sub = nh->subscribe("/imu_raw", 10, &odomTranslator::imuCallback,this);
        gps_sub = nh->subscribe("/odometry/gps", 10, &odomTranslator::gpsCallback,this);
        
    
    }
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        
        imu.header.seq = msg->header.seq;
        imu.header.stamp.sec = msg->header.stamp.sec;
        imu.header.stamp.nsec = msg->header.stamp.nsec;
        imu.header.frame_id = msg->header.frame_id;
        imu.orientation.x = msg->orientation.x;
        imu.orientation.y = msg->orientation.y;
        imu.orientation.z = msg->orientation.z;
        imu.orientation.w = msg->orientation.w;
        imu.orientation_covariance = msg->orientation_covariance;
        imu.angular_velocity.x = msg->angular_velocity.x;
        imu.angular_velocity.y = msg->angular_velocity.y;
        imu.angular_velocity.z = msg->angular_velocity.z;
        imu.angular_velocity_covariance = msg->angular_velocity_covariance;
        imu.linear_acceleration_covariance = msg->linear_acceleration_covariance;
        imu.linear_acceleration.x = msg->linear_acceleration.x;
        imu.linear_acceleration.y = msg->linear_acceleration.y;
        imu.linear_acceleration.z = msg->linear_acceleration.z;


        imuMsgPublisher();


    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {

        tf::Quaternion q(
            odomMsg->pose.pose.orientation.x,
            odomMsg->pose.pose.orientation.y,
            odomMsg->pose.pose.orientation.z,
            odomMsg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);

        tf::Matrix3x3 m2;
        //m2.setEulerYPR(3.14, 0, 0);
        //m = m2 * m;

        m.getRPY(roll_tf, pitch_tf, yaw_tf); //gets yaw,pitch ,roll from quaternions.



        odom.header.stamp = odomMsg->header.stamp;
        odom.header.frame_id = odomMsg->header.frame_id;
        odom.child_frame_id = odomMsg->child_frame_id;


        odom.pose.pose.position.x = odomMsg->pose.pose.position.x;
        odom.pose.pose.position.y = odomMsg->pose.pose.position.y;
        odom.pose.pose.position.z = odomMsg->pose.pose.position.z;

        odom.pose.pose.orientation.x = odomMsg->pose.pose.orientation.x;
        odom.pose.pose.orientation.y = odomMsg->pose.pose.orientation.y;
        odom.pose.pose.orientation.z = odomMsg->pose.pose.orientation.z;
        odom.pose.pose.orientation.w = odomMsg->pose.pose.orientation.w;




        odom.twist.twist.linear.x = odomMsg->twist.twist.linear.x;
        odom.twist.twist.linear.y = odomMsg->twist.twist.linear.y;
        odom.twist.twist.linear.z = odomMsg->twist.twist.linear.z;

        odom.twist.twist.angular.x = odomMsg->twist.twist.angular.x;
        odom.twist.twist.angular.y = odomMsg->twist.twist.angular.y;
        odom.twist.twist.angular.z = odomMsg->twist.twist.angular.z;


        

        odomMsgPublisher();
    }
    void gpsCallback(const nav_msgs::Odometry::ConstPtr &gpsMsg)
    {

        gps.header.stamp = gpsMsg->header.stamp;
        gps.header.frame_id = gpsMsg->header.frame_id;
        gps.child_frame_id = gpsMsg->child_frame_id;


        gps.pose.pose.position.x = gpsMsg->pose.pose.position.x;
        gps.pose.pose.position.y = gpsMsg->pose.pose.position.y;
        gps.pose.pose.position.z = gpsMsg->pose.pose.position.z;
        gps.pose.covariance = gpsMsg->pose.covariance;

        gps.pose.pose.orientation.x = gpsMsg->pose.pose.orientation.x;
        gps.pose.pose.orientation.y = gpsMsg->pose.pose.orientation.y;
        gps.pose.pose.orientation.z = gpsMsg->pose.pose.orientation.z;
        gps.pose.pose.orientation.w = gpsMsg->pose.pose.orientation.w;




        gps.twist.twist.linear.x = gpsMsg->twist.twist.linear.x;
        gps.twist.twist.linear.y = gpsMsg->twist.twist.linear.y;
        gps.twist.twist.linear.z = gpsMsg->twist.twist.linear.z;

        gps.twist.twist.angular.x = gpsMsg->twist.twist.angular.x;
        gps.twist.twist.angular.y = gpsMsg->twist.twist.angular.y;
        gps.twist.twist.angular.z = gpsMsg->twist.twist.angular.z;
        gps.twist.covariance = gpsMsg->twist.covariance;

        

        gpsMsgPublisher();
    }
    void gpsMsgCreator()
    {

        gps_trans.header.stamp = gps.header.stamp;
        gps_trans.header.frame_id = gps.header.frame_id;
        gps_trans.child_frame_id = gps.child_frame_id;

        gps_trans.pose.pose.position.x = gps.pose.pose.position.x + gausRandom(0,0.3);
        gps_trans.pose.pose.position.y = gps.pose.pose.position.y + gausRandom(0,0.3);
        gps_trans.pose.pose.position.z = gps.pose.pose.position.z;

        gps_trans.pose.pose.orientation.x = gps.pose.pose.orientation.x ;
        gps_trans.pose.pose.orientation.y = gps.pose.pose.orientation.y ;
        gps_trans.pose.pose.orientation.z = gps.pose.pose.orientation.z ;
        gps_trans.pose.pose.orientation.w = gps.pose.pose.orientation.w ;
        gps_trans.pose.covariance= gps.pose.covariance;


        gps_trans.twist.twist.linear.x = gps.twist.twist.linear.x + gausRandom(0,0.3);
        gps_trans.twist.twist.linear.y = odom.twist.twist.linear.y+ gausRandom(0,0.3);
        gps_trans.twist.twist.linear.z = odom.twist.twist.linear.z+ gausRandom(0,0.3);

        gps_trans.twist.twist.angular.x = gps.twist.twist.angular.x;
        gps_trans.twist.twist.angular.y = gps.twist.twist.angular.y;
        gps_trans.twist.twist.angular.z = gps.twist.twist.angular.z;
        gps_trans.twist.covariance = gps.twist.covariance;
    



    }

    void imuMsgCreator(){


        imu_trans.header.seq =  imu.header.seq;
        imu_trans.header.stamp.sec = imu.header.stamp.sec;
        imu_trans.header.stamp.nsec = imu.header.stamp.nsec;
        imu_trans.header.frame_id =  imu.header.frame_id;
        imu_trans.orientation.x = imu.orientation.x ;
        imu_trans.orientation.y = imu.orientation.y ;
        imu_trans.orientation.z = imu.orientation.z ;
        imu_trans.orientation.w =  imu.orientation.w;
        imu_trans.angular_velocity.x = imu.angular_velocity.x + gausRandom(0,0.3);
        imu_trans.angular_velocity.y = imu.angular_velocity.y + gausRandom(0,0.3);
        imu_trans.angular_velocity.z = imu.angular_velocity.z + gausRandom(0,0.3);
        imu_trans.linear_acceleration.x = imu.linear_acceleration.x + gausRandom(0,0.3);
        imu_trans.linear_acceleration.y = imu.linear_acceleration.y + gausRandom(0,0.3);
        imu_trans.linear_acceleration.z = imu.linear_acceleration.z + gausRandom(0,0.3);

        imu_trans.orientation_covariance = imu.orientation_covariance;
        imu_trans.angular_velocity_covariance = imu.angular_velocity_covariance;
        imu_trans.linear_acceleration_covariance = imu.linear_acceleration_covariance;



    }
    void odomMsgCreator()
    {

        odom_trans.header.stamp = odom.header.stamp;
        odom_trans.header.frame_id = odom.header.frame_id;
        odom_trans.child_frame_id = odom.child_frame_id;

        odom_trans.pose.pose.position.x = odom.pose.pose.position.x;
        odom_trans.pose.pose.position.y = odom.pose.pose.position.y;
        odom_trans.pose.pose.position.z = odom.pose.pose.position.z;

        odom_trans.pose.pose.orientation.x = odom.pose.pose.orientation.x ;
        odom_trans.pose.pose.orientation.y = odom.pose.pose.orientation.y ;
        odom_trans.pose.pose.orientation.z = odom.pose.pose.orientation.z ;
        odom_trans.pose.pose.orientation.w = odom.pose.pose.orientation.w ;




        odom_trans.twist.twist.linear.x = odom.twist.twist.linear.x + gausRandom(0,0.1);
        odom_trans.twist.twist.linear.y = odom.twist.twist.linear.y+ gausRandom(0,0.1);
        odom_trans.twist.twist.linear.z = odom.twist.twist.linear.z+ gausRandom(0,0.1);

        odom_trans.twist.twist.angular.x = odom.twist.twist.angular.x;
        odom_trans.twist.twist.angular.y = odom.twist.twist.angular.y;
        odom_trans.twist.twist.angular.z = odom.twist.twist.angular.z;
    
    



    }

    //Generates random value between 0-1
    double rand_gen()
    {
        // return a uniformly distributed random value
        return ((double)(rand()) + 1.) / ((double)(RAND_MAX) + 1.);
    }


    //Returns random noise between -n and n
    double gausRandom(double mean,double stddev){

        auto dist = std::bind(std::normal_distribution<double>{mean, stddev},
        std::mt19937(std::random_device{}()));

        double val = dist();


        return val;

        


    }
    void gpsMsgPublisher(){

        gpsMsgCreator();

        gps_trans_pub.publish(gps_trans);
    }
    void imuMsgPublisher()
    {

        imuMsgCreator();



        imu_trans_pub.publish(imu_trans);
        
    
    }
    void odomMsgPublisher()
    {

        odomMsgCreator();



        odom_trans_pub.publish(odom_trans);
        
    
    }
};

int main(int argc, char **argv)
{

    ros::init(argc, argv, "odom_trans_node");
    ros::NodeHandle nh;

    odomTranslator odom_t(&nh);

    ros::Rate rate(10);
    ros::spin();
}