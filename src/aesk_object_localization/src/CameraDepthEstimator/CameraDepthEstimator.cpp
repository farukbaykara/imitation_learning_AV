#include "aesk_object_localization/CameraDepthEstimator/CameraDepthEstimator.hpp"

namespace aesk_object_localization
{
CameraDepthEstimator::CameraDepthEstimator(ros::NodeHandle& t_node_handle_cloud)
  : m_node_handle(t_node_handle_cloud), m_is_cloud_received{ false }, m_is_lane_received{ false }
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Projected point cloud topic: : %s", m_point_cloud_topic.c_str());
    ROS_INFO("Lane point cloud topic: : %s", m_lane_cloud_topic.c_str());

    // TODO: USE MESSAGE FILTER!

    m_point_cloud_sub = m_node_handle.subscribe<sensor_msgs::PointCloud2>(
        m_point_cloud_topic, 1, &CameraDepthEstimator::projectedCloudCallback, this);

    m_lane_cloud_sub = m_node_handle.subscribe<sensor_msgs::PointCloud2>(
        m_lane_cloud_topic, 1, &CameraDepthEstimator::laneCloudCallback, this);

    m_point_cloud_pub = m_node_handle.advertise<sensor_msgs::PointCloud2>("lane_3d", 1);

    m_o3d_cloud = std::make_shared<open3d::geometry::PointCloud>();

    ROS_INFO("Successfully launched CameraDepthEstimator node.");

    ros::Rate rate(20);
    ros::Rate rate_waiting(1);
    while (ros::ok())
    {
        if (m_is_cloud_received && m_is_lane_received)
        {
            estimateDepth();
        }
        else
        {
            ROS_INFO("Waiting for messages.");
            rate_waiting.sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

bool CameraDepthEstimator::readParameters()
{
    if (!m_node_handle.getParam("projected_cloud_topic", m_point_cloud_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("lane_cloud_topic", m_lane_cloud_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("search_radius", m_search_radius))
    {
        return false;
    }
    return true;
}

void CameraDepthEstimator::projectedCloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    m_is_cloud_received = true;

    // ROS PointCloud2 to Open3D Point Cloud conversion.

    // Create PointCloud2 iterators.
    sensor_msgs::PointCloud2ConstIterator<float> iterator_x(*t_point_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterator_y(*t_point_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iterator_z(*t_point_cloud, "z");
    sensor_msgs::PointCloud2ConstIterator<float> iterator_pX(*t_point_cloud, "pX");
    sensor_msgs::PointCloud2ConstIterator<float> iterator_pY(*t_point_cloud, "pY");

    // Delete previous points.
    m_o3d_cloud->points_.clear();
    m_o3d_cloud->colors_.clear();

    // Resize std::vector with new t_point_cloud size.
    const size_t cloud_size = t_point_cloud->width * t_point_cloud->height;
    m_o3d_cloud->points_.resize(cloud_size);
    m_o3d_cloud->colors_.resize(cloud_size);

    // Iterate through PointCloud2 and push back points to o3d cloud.
    for (size_t i = 0; i < cloud_size;
         ++i, ++iterator_x, ++iterator_y, ++iterator_z, ++iterator_pX, ++iterator_pY)
    {
        const Eigen::Vector3d lidar_point{ *iterator_x, *iterator_y, *iterator_z };
        const Eigen::Vector3d image_point{ *iterator_pX, *iterator_pY, 0 };

        // Add 3D lidar points to "colors_" field. We reserve "points_" filed for 2D lane points
        // because we'll create KDtree with lane points.
        m_o3d_cloud->colors_.push_back(lidar_point);
        m_o3d_cloud->points_.push_back(image_point);
    }
}

void CameraDepthEstimator::laneCloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    m_is_lane_received = true;
    m_lane_cloud = t_point_cloud;
}
void CameraDepthEstimator::estimateDepth()
{
    open3d::geometry::KDTreeFlann tree(*m_o3d_cloud);

    // Create PointCloud2 iterators.
    sensor_msgs::PointCloud2ConstIterator<float> iterator_x(*m_lane_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterator_y(*m_lane_cloud, "y");

    const size_t cloud_size = m_lane_cloud->height * m_lane_cloud->width;

    std::vector<std::array<float, 5>> points;

    // Iterate through PointCloud2 and find nearest neighbours.
    for (size_t i = 0; i < cloud_size; ++i, ++iterator_x, ++iterator_y)
    {
        std::vector<int> indices;
        std::vector<double> distance2;
        tree.SearchRadius(Eigen::Vector3d{ *iterator_x, *iterator_y, 0 }, m_search_radius, indices,
                          distance2);

        for (auto indice : indices)
        {
            std::array<float, 5> point{ 0, 0, 0, 0, 0 };

            const Eigen::Vector3d lidar_point = m_o3d_cloud->colors_[indice];
            point[0] = lidar_point[0];
            point[1] = lidar_point[1];
            point[2] = lidar_point[2];
            point[3] = *iterator_x;
            point[4] = *iterator_y;

            points.push_back(point);
        }
    }

    sensor_msgs::PointCloud2 p_point_cloud;
    sensor_msgs::PointCloud2Modifier modifier(p_point_cloud);
    modifier.resize(points.size());

    modifier.setPointCloud2Fields(
        5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z",
        1, sensor_msgs::PointField::FLOAT32, "pX", 1, sensor_msgs::PointField::FLOAT32, "pY", 1,
        sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_x(p_point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_y(p_point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_z(p_point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_pX(p_point_cloud, "pX");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_pY(p_point_cloud, "pY");

    for (size_t i = 0; i < points.size(); ++i, ++p_point_cloud_x, ++p_point_cloud_y,
                ++p_point_cloud_z, ++p_point_cloud_pX, ++p_point_cloud_pY)
    {
        *p_point_cloud_x = points[i][0];
        *p_point_cloud_y = points[i][1];
        *p_point_cloud_z = points[i][2];
        *p_point_cloud_pX = points[i][3];
        *p_point_cloud_pY = points[i][4];
    }

    std_msgs::Header header;
    header.frame_id = "velodyne";
    p_point_cloud.header = header;
    m_point_cloud_pub.publish(p_point_cloud);
}

}  // namespace aesk_object_localization