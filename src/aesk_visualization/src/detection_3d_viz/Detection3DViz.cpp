#include "aesk_visualization/detection_3d_viz/Detection3DViz.hpp"

namespace aesk_visualization
{
Detection3DViz::Detection3DViz(ros::NodeHandle& t_node_handle)
  : m_node_handle{ t_node_handle }, m_rng(m_dev()), m_dist255(0.0, 1.0)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Detections 3D topic: %s", m_detection_topic.c_str());

    m_detection_sub =
        m_node_handle.subscribe(m_detection_topic, 1, &Detection3DViz::detectionCallback, this);
    m_marker_pub = m_node_handle.advertise<visualization_msgs::MarkerArray>("detections", 1);
    m_cloud_pub = m_node_handle.advertise<sensor_msgs::PointCloud2>("cloud", 1);

    ROS_INFO("Successfully launched detection_3d_viz node.");
}
Eigen::Vector3d Detection3DViz::createRandomColor()
{
    return Eigen::Vector3d{ m_dist255(m_rng), m_dist255(m_rng), m_dist255(m_rng) };
}
bool Detection3DViz::readParameters()
{
    if (!m_node_handle.getParam("detection_3d_topic", m_detection_topic))
    {
        return false;
    }
    return true;
}
void Detection3DViz::detectionCallback(const vision_msgs::Detection3DArray& t_msg)
{
    visualization_msgs::MarkerArray markers;
//    utils::PointCloudOpen3D cluster_clouds;

    // Iterate through detected objects.
    for (size_t i = 0; i < t_msg.detections.size(); ++i)
    {
        // Create marker for detected object.
        visualization_msgs::Marker marker;
        marker.header.frame_id = "velodyne";
        marker.header.stamp = ros::Time::now();
        marker.id = static_cast<int>(i);
        marker.type = marker.CUBE;
        marker.action = marker.ADD;
        marker.pose.position.x = t_msg.detections[i].bbox.center.position.x;
        marker.pose.position.y = t_msg.detections[i].bbox.center.position.y;
        marker.pose.position.z = t_msg.detections[i].bbox.center.position.z;
        marker.pose.orientation.x = t_msg.detections[i].bbox.center.orientation.x;
        marker.pose.orientation.y = t_msg.detections[i].bbox.center.orientation.y;
        marker.pose.orientation.z = t_msg.detections[i].bbox.center.orientation.z;
        marker.pose.orientation.w = t_msg.detections[i].bbox.center.orientation.w;
        marker.scale.x = t_msg.detections[i].bbox.size.x;
        marker.scale.y = t_msg.detections[i].bbox.size.y;
        marker.scale.z = t_msg.detections[i].bbox.size.z;
        marker.color.a = 1;
        const Eigen::Vector3d color = createRandomColor();
        marker.color.r = static_cast<float>(color.x());
        marker.color.g = static_cast<float>(color.y());
        marker.color.b = static_cast<float>(color.z());
        marker.lifetime = ros::Duration(0.1);
        markers.markers.push_back(marker);

//        const std::shared_ptr<utils::PointCloudOpen3D> cloud =
//            utils::PointCloudOpen3D::CreateFromPointCloud2(t_msg.detections[i].source_cloud);
//
//        cluster_clouds.points_.insert(cluster_clouds.points_.end(), cloud->points_.begin(),
//                                      cloud->points_.end());

//        for (size_t j = 0; j < cloud->points_.size(); ++j)
//        {
//            cluster_clouds.colors_.emplace_back(color);
//        }
    }
    m_marker_pub.publish(markers);
//    cluster_clouds.Publish(m_cloud_pub);
}

}  // namespace aesk_visualization