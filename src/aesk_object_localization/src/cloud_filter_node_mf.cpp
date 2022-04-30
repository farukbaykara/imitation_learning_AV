#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <aesk_object_localization/ProjectedCloudAction.h>

#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include "aesk_object_localization/PointXYZpXpY.hpp"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <aesk_object_localization/PointCloud2WithId.h>
#include <aesk_object_localization/ArrayOfPointCloud2s.h>

//! Lidar topic name.
std::string m_lidar_topic;

//! YOLO boundind boxes topic name.
std::string m_yolo_topic;

//! Extended bounding box x_min value.
int m_extended_x_min;

//! Extended bounding box x_max value.
int m_extended_x_max;

//! Extended bounding box y_min value.
int m_extended_y_min;

//! Extended bounding box y_max value.
int m_extended_y_max;

//! Publisher for ArrayOfPointCloud2s.
ros::Publisher m_clouds_publisher;

/*!
 * Message Filter callback function.
 * @param t_cloud the received PointCloud2 message.
 * @param t_yolo_boxes the received BoundingBoxes message.
 */
void filterCallback(const sensor_msgs::PointCloud2ConstPtr& t_cloud,
                    const darknet_ros_msgs::BoundingBoxesConstPtr& t_yolo_boxes)
{
    ROS_INFO("Started MessageFilter callback.");
    
    // ros_tkdnn publishes an empty array even it doesn't detect any objects.
    if (t_yolo_boxes->bounding_boxes.size() == 0)
    {
        return;
    }

    // Convert ROS PointCloud2 message to PCL type.
    pcl::PointCloud<pcl::PointXYZpXpY>::Ptr point_cloud_raw(new pcl::PointCloud<pcl::PointXYZpXpY>);
    pcl::fromROSMsg(*t_cloud, *point_cloud_raw);

    aesk_object_localization::ArrayOfPointCloud2s cloud_array;
    cloud_array.header.stamp = ros::Time::now();

    for (auto box : t_yolo_boxes->bounding_boxes)
    {
        pcl::PointCloud<pcl::PointXYZpXpY>::Ptr point_cloud_filtered(
            new pcl::PointCloud<pcl::PointXYZpXpY>);

        pcl::PassThrough<pcl::PointXYZpXpY> passthrough_by_px;
        passthrough_by_px.setInputCloud(point_cloud_raw);
        passthrough_by_px.setFilterFieldName("pX");
        passthrough_by_px.setFilterLimits(box.xmin - m_extended_x_min, box.xmax + m_extended_x_max);
        passthrough_by_px.filter(*point_cloud_filtered);

        pcl::PassThrough<pcl::PointXYZpXpY> passthrough_by_py;
        passthrough_by_py.setInputCloud(point_cloud_filtered);
        passthrough_by_py.setFilterFieldName("pY");
        passthrough_by_py.setFilterLimits(box.ymin - m_extended_y_min, box.ymax + m_extended_y_max);
        passthrough_by_py.filter(*point_cloud_filtered);

        sensor_msgs::PointCloud2 point_cloud_filtered_msg;
        pcl::toROSMsg(*point_cloud_filtered, point_cloud_filtered_msg);
        point_cloud_filtered_msg.header.frame_id = "velodyne";
        point_cloud_filtered_msg.header.stamp = ros::Time::now();

        aesk_object_localization::PointCloud2WithId cloud_with_id;
        cloud_with_id.header.stamp = ros::Time::now();
        cloud_with_id.id = box.id;
        cloud_with_id.cloud = point_cloud_filtered_msg;

        cloud_array.clouds.push_back(cloud_with_id);

        cloud_array.header.stamp = ros::Time::now();
        cloud_array.clouds.push_back(cloud_with_id);
    }

    m_clouds_publisher.publish(cloud_array);
}

bool readParameters(ros::NodeHandle& m_node_handle)
{
    if (!m_node_handle.getParam("lidar_topic", m_lidar_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("yolo_topic", m_yolo_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("extended_x_min", m_extended_x_min))
    {
        return false;
    }
    if (!m_node_handle.getParam("extended_x_max", m_extended_x_max))
    {
        return false;
    }
    if (!m_node_handle.getParam("extended_y_min", m_extended_y_min))
    {
        return false;
    }
    if (!m_node_handle.getParam("extended_y_max", m_extended_y_max))
    {
        return false;
    }
    return true;
}

/*!
 * Main function.
 * @param t_yolo_boxes the received PointCloud2 message.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_painter_mf");
    ros::NodeHandle node_handle("~");

    readParameters(node_handle);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_subscriber(node_handle,
                                                                           m_lidar_topic, 1);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> yolo_subscriber(node_handle,
                                                                                 m_yolo_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2,
                                                            darknet_ros_msgs::BoundingBoxes>
        MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), cloud_subscriber,
                                                     yolo_subscriber);
    sync.registerCallback(boost::bind(&filterCallback, _1, _2));

    //! Publisher for ArrayOfPointCloud2s.
    m_clouds_publisher =
        node_handle.advertise<aesk_object_localization::ArrayOfPointCloud2s>("filtered_clouds", 1);

    ROS_INFO("Successfully launched node.");

    ros::spin();
    return 0;
}