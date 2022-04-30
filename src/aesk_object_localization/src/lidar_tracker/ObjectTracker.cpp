#include "aesk_object_localization/lidar_tracker/ObjectTracker.hpp"

namespace aesk_object_localization
{
ObjectTracker::ObjectTracker(std::shared_ptr<utils::PointCloudOpen3D>& t_cloud)
  : m_cloud{ t_cloud }, m_gate_margin{ 0, 0, 0 }
{
}

void ObjectTracker::setGateExtent(const Eigen::Vector3d& t_gate_extent)
{
    m_gate_extent = t_gate_extent;
}

void ObjectTracker::setGateMargin(const Eigen::Vector3d& t_gate_margin)
{
    m_gate_margin = t_gate_margin;
}

Object ObjectTracker::track(const Object& t_object, const bool& t_is_oriented) const
{
    // Create gate. The gate will be used to crop source point cloud.
    const Eigen::Matrix3d rotation_matrix{ t_object.m_orientation.normalized().toRotationMatrix() };

    open3d::geometry::OrientedBoundingBox gate{ t_object.m_position + m_gate_margin,
                                                rotation_matrix, m_gate_extent };

    if (!t_is_oriented)
    {
        const Eigen::Quaterniond qua{ 0, 0, 0, 1 };
        gate.R_ = qua.normalized().toRotationMatrix();
    }

    Object object;
    object.m_id = t_object.m_id;

    if (m_gate_extent.x() == 0 && m_gate_extent.y() == 0 && m_gate_extent.z() == 0)
    {
        ROS_ERROR("Extent is 0.");
        object.m_position.x() = 0;
        object.m_position.y() = 0;
        object.m_position.z() = 0;
        object.m_orientation.x() = 0;
        object.m_orientation.y() = 0;
        object.m_orientation.z() = 0;
        object.m_orientation.w() = 1;
        return object;
    }

    // Crop the source cloud with the gate.
    const auto cropped_cloud = m_cloud->Crop(gate);

    // TODO: Try RemoveRadiusOutlier here.

    //    object.setPointCloud(cropped_cloud); // BUG BUG BUG TODO!

    if (cropped_cloud->points_.empty())
    {
        ROS_ERROR("No points to create bounding box.");
        object.m_position.x() = 0;
        object.m_position.y() = 0;
        object.m_position.z() = 0;
        object.m_orientation.x() = 0;
        object.m_orientation.y() = 0;
        object.m_orientation.z() = 0;
        object.m_orientation.w() = 1;
        return object;
    }

    //    ROS_INFO("Points: %zu", cropped_cloud->points_.size());
    // We need at least four points to create oriented bounding box.
    //    if (cropped_cloud->points_.size() >= 0)
    if (cropped_cloud->points_.size() < 4 || !t_is_oriented)
    {
        ROS_WARN("Not enough points to create oriented bounding box.");

        // If there are less than four points, create AxisAlignedBoundingBox.
        const auto new_bbox = cropped_cloud->GetAxisAlignedBoundingBox();

        object.m_position = new_bbox.GetCenter();

        Eigen::Quaterniond orientation;
        orientation.x() = 0;
        orientation.y() = 0;
        orientation.z() = 0;
        orientation.w() = 1;
        object.m_orientation = orientation;

        const auto min_bound = new_bbox.GetMinBound();
        const auto max_bound = new_bbox.GetMaxBound();
        const Eigen::Vector3d size{ max_bound.x() - min_bound.x(), max_bound.y() - min_bound.y(),
                                    max_bound.z() - min_bound.z() };
        object.m_extent = size;
    }
    else if (t_is_oriented)
    {
        // Create new oriented bounding box.
        const auto new_bbox = cropped_cloud->GetOrientedBoundingBox();

        object.m_position = new_bbox.GetCenter();
        const Eigen::Quaterniond orientation{ new_bbox.R_ };
        object.m_orientation = orientation;
        object.m_extent = new_bbox.extent_;
    }
    else
    {
        ROS_ERROR("No points to create bounding box.");
        object.m_position.x() = 0;
        object.m_position.y() = 0;
        object.m_position.z() = 0;
        object.m_orientation.x() = 0;
        object.m_orientation.y() = 0;
        object.m_orientation.z() = 0;
        object.m_orientation.w() = 1;
        return object;
    }
    return object;
}

}  // namespace aesk_object_localization