#include "aesk_object_localization/ROIFilter/ROIFilter.hpp"

namespace aesk_object_localization
{
ROIFilter::ROIFilter(ros::NodeHandle& t_node_handle)
  : m_node_handle{ t_node_handle }, m_use_voxel{ false }
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Lidar topic: %s", m_lidar_topic.c_str());
    for (const auto& bound : m_roi_bounds)
    {
        ROS_INFO("%s: %f", bound.first.c_str(), bound.second);
    }
    logBoolean("use_voxel", m_use_voxel);
    ROS_INFO("voxel_size: %f", m_voxel_size);

    m_lidar_sub = m_node_handle.subscribe(m_lidar_topic, 1, &ROIFilter::lidarCallback, this);
    m_filtered_pub = m_node_handle.advertise<sensor_msgs::PointCloud2>("cloud", 1);

    ROS_INFO("Successfully launched roi_filter node.");
}

bool ROIFilter::readParameters()
{
    if (!m_node_handle.getParam("lidar_topic", m_lidar_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("roi_bounds", m_roi_bounds))
    {
        return false;
    }
    if (!m_node_handle.getParam("voxel_downsample/use", m_use_voxel))
    {
        return false;
    }
    if (!m_node_handle.getParam("voxel_downsample/voxel_size", m_voxel_size))
    {
        return false;
    }
    return true;
}

void ROIFilter::logBoolean(const std::string& t_field, const bool& t_bool)
{
    if (t_bool)
    {
        ROS_INFO("%s: true", t_field.c_str());
    }
    else
    {
        ROS_INFO("%s: false", t_field.c_str());
    }
}

void ROIFilter::lidarCallback(const sensor_msgs::PointCloud2& t_msg)
{
    std::shared_ptr<utils::PointCloudOpen3D> cloud_raw = utils::PointCloudOpen3D::CreateFromPointCloud2(t_msg);

    const Eigen::Vector3d min_bounds{ m_roi_bounds["xmin"], m_roi_bounds["ymin"],
                                      m_roi_bounds["zmin"] };
    const Eigen::Vector3d max_bounds{ m_roi_bounds["xmax"], m_roi_bounds["ymax"],
                                      m_roi_bounds["zmax"] };
    open3d::geometry::AxisAlignedBoundingBox roi_box{ min_bounds, max_bounds };

    cloud_raw->UpdateFromO3D(cloud_raw->Crop(roi_box));

    if (m_use_voxel)
    {
        cloud_raw->UpdateFromO3D(cloud_raw->VoxelDownSample(m_voxel_size));
    }

    cloud_raw->Publish(m_filtered_pub);
}
}  // namespace aesk_object_localization
