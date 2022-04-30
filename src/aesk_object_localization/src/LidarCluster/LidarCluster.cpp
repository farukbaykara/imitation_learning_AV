#include "aesk_object_localization/LidarCluster/LidarCluster.hpp"

namespace aesk_object_localization
{
LidarCluster::LidarCluster(ros::NodeHandle& t_node_handle)
  : m_node_handle{ t_node_handle }
  , m_mersenne{ static_cast<std::mt19937::result_type>(std::time(nullptr)) }
  , m_die{ 0, 255 }
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Lidar topic: %s", m_lidar_topic.c_str());

    m_lidar_sub = m_node_handle.subscribe(m_lidar_topic, 1, &LidarCluster::lidarCallback, this);
    m_detections_pub = m_node_handle.advertise<vision_msgs::Detection3DArray>("detections", 1);

    // Create a random number for nothing. >> /dev/null
    m_die(m_mersenne);

    ROS_INFO("Successfully launched lidar_cluster node.");
}

bool LidarCluster::readParameters()
{
    if (!m_node_handle.getParam("lidar_topic", m_lidar_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("dbscan/eps", m_eps))
    {
        return false;
    }
    if (!m_node_handle.getParam("dbscan/min_points", m_min_points))
    {
        return false;
    }
    return true;
}

void LidarCluster::lidarCallback(const sensor_msgs::PointCloud2& t_msg)
{
    // Create wrapped o3d cloud from receiving PointCloud2 msg.
    const std::shared_ptr<utils::PointCloudOpen3D> cloud_raw =
        utils::PointCloudOpen3D::CreateFromPointCloud2(t_msg);

    // Do the clustering.
    const std::vector<int> indices = cloud_raw->ClusterDBSCAN(m_eps, m_min_points);

    // Get cluster number.
    const std::set<int> unique_indices{ indices.begin(), indices.end() };
    size_t cluster_num{ unique_indices.size() };
    if (unique_indices.count(-1) != 0)
    {
        // Remove outliers.
        cluster_num -= 1;
    }
    ROS_INFO("Cluster number: %zu", cluster_num);

    std::vector<utils::PointCloudOpen3D> clusters;
    for (size_t i = 0; i < cluster_num; ++i)
    {
        // Create a point cloud for each cluster.
        utils::PointCloudOpen3D detection;
        clusters.push_back(detection);
    }

    // Separate clusters.
    for (size_t i = 0; i < indices.size(); ++i)
    {
        // If the point is an outlier, continue.
        if (indices[i] == -1)
        {
            continue;
        }
        const int cluster_id = indices[i];
        clusters[cluster_id].points_.push_back(cloud_raw->points_[i]);
    }

    vision_msgs::Detection3DArray detections;
    detections.header = createHeader();
    for (const auto& cluster : clusters)
    {
        const open3d::geometry::OrientedBoundingBox bbox = cluster.GetOrientedBoundingBox();
        const Eigen::Vector3d center = bbox.GetCenter();
        const Eigen::Quaterniond rotation{ bbox.R_ };
        const Eigen::Vector3d extent = bbox.extent_;

        vision_msgs::Detection3D detection;
        detection.header = createHeader();
        detection.bbox.center.position.x = center.x();
        detection.bbox.center.position.y = center.y();
        detection.bbox.center.position.z = center.z();
        detection.bbox.center.orientation.x = rotation.x();
        detection.bbox.center.orientation.y = rotation.y();
        detection.bbox.center.orientation.z = rotation.z();
        detection.bbox.center.orientation.w = rotation.w();
        detection.bbox.size.x = extent.x();
        detection.bbox.size.y = extent.y();
        detection.bbox.size.z = extent.z();
        detection.source_cloud.header = createHeader();
        cluster.ConvertToPointCloud2(detection.source_cloud);
        detections.detections.push_back(detection);
    }
    m_detections_pub.publish(detections);
}
std_msgs::Header LidarCluster::createHeader()
{
    std_msgs::Header header;
    header.frame_id = "velodyne";
    header.stamp = ros::Time::now();
    return header;
}
}  // namespace aesk_object_localization