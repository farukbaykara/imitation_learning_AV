#include "aesk_object_localization/CloudFilter.hpp"
#include "aesk_object_localization/PointXYZpXpY.hpp"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

#include <aesk_object_localization/PointCloud2WithId.h>
#include <aesk_object_localization/ArrayOfPointCloud2s.h>

namespace aesk_object_localization
{
CloudFilter::CloudFilter(ros::NodeHandle& t_node_handle)
  : m_node_handle(t_node_handle), m_action_client("cloud_painter/cloud_projector", true)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Lidar topic: : %s", m_lidar_topic.c_str());
    ROS_INFO("YOLO topic: : %s", m_yolo_topic.c_str());

    ROS_INFO("Extended x_min: %d", m_extended_x_min);
    ROS_INFO("Extended x_max: %d", m_extended_x_max);
    ROS_INFO("Extended y_min %d", m_extended_y_min);
    ROS_INFO("Extended y_max %d", m_extended_y_min);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    m_action_client.waitForServer();  // will wait for infinite time
    ROS_INFO("Action server started.");

    m_yolo_subscriber =
        m_node_handle.subscribe(m_yolo_topic.c_str(), 1, &CloudFilter::yoloTopicCallback, this);

    m_clouds_publisher = m_node_handle.advertise<aesk_object_localization::ArrayOfPointCloud2s>(
        "filtered_clouds", 1);

    ROS_INFO("Successfully launched node.");
}

bool CloudFilter::readParameters()
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

void CloudFilter::yoloTopicCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& t_yolo_boxes)
{
    // ros_tkdnn publishes an empty array even it doesn't detect any objects.
    if (t_yolo_boxes->bounding_boxes.size() == 0)
    {
        return;
    }

    // "x_max" and "y_max" values do not matter.
    // All we do here is to get the most recent projected cloud.
    aesk_object_localization::ProjectedCloudGoal goal;
    goal.x_max = 0;
    goal.y_max = 0;
    m_action_client.sendGoal(goal);

    // Wait for the action to return.
    bool finished_before_timeout = m_action_client.waitForResult(ros::Duration(0.2));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = m_action_client.getState();
        // ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
        ROS_INFO("Action did not finish before the time out.");

    // Convert ROS PointCloud2 message to PCL type.
    pcl::PointCloud<pcl::PointXYZpXpY>::Ptr point_cloud_raw(new pcl::PointCloud<pcl::PointXYZpXpY>);
    pcl::fromROSMsg(m_action_client.getResult()->cloud, *point_cloud_raw);

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

        cloud_array.header.stamp = ros::Time::now();
        cloud_array.clouds.push_back(cloud_with_id);
    }

    m_clouds_publisher.publish(cloud_array);
}
}  // namespace aesk_object_localization