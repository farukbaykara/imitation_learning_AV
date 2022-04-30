#pragma once

#include "open3d/Open3D.h"
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace utils
{
typedef open3d::geometry::PointCloud o3dCloud;

/// \class PointCloudOpen3D
///
/// \brief A point cloud consists of point coordinates, and optionally point
/// colors and point normals.
class PointCloudOpen3D : public o3dCloud
{
  public:
    /// \brief Factory function to create a pointcloud from a ROS PointCloud2 msg.
    ///
    /// This function iterates through PointCloud2 and adds points to Open3D cloud.
    ///
    /// \param t_cloud The input ROS PointCloud2 msg.
    ///
    /// \return The Open3D point cloud as utils::PointCloudOpen3D.
    static std::shared_ptr<PointCloudOpen3D>
    CreateFromPointCloud2(const sensor_msgs::PointCloud2& t_cloud);

    /// \brief A function to convert Open3D pointcloud to ROS PointCloud2 msg.
    ///
    /// This function iterates through Open3D cloud and adds points to PointCloud2.
    ///
    /// \param t_msg The input ROS PointCloud2 msg.
    void ConvertToPointCloud2(sensor_msgs::PointCloud2& t_msg) const;

    /// \brief A function to publish pointcloud as PointCloud2.
    ///
    /// This function first converts pointcloud to PointCloud2 then
    /// publishes it with the given ROS publisher.
    ///
    /// \param t_publisher The ROS publisher.
    void Publish(const ros::Publisher& t_publisher) const;

    /// \brief A function to publish pointcloud as PointCloud2.
    ///
    /// This function first converts pointcloud to PointCloud2 then
    /// publishes it with the given ROS publisher.
    ///
    /// \param t_publisher The ROS publisher.
    /// \param t_header The Header msg.
    void Publish(const ros::Publisher& t_publisher, const std_msgs::Header& t_header) const;

    /// \brief A function to update existing fields from o3d cloud.
    ///
    /// This function reads and updates points_, colors_ and normals_
    /// fileds from given o3d cloud.
    ///
    /// \param t_publisher The ROS publisher.
    /// \param t_header The Header msg.
    void UpdateFromO3D(const std::shared_ptr<o3dCloud>& t_cloud);

    /// \brief A function to create ROS Header.
    ///
    /// This function creates a header with the frame id "velodyne".
    ///
    static std_msgs::Header createHeader();

    /// \brief A function to create ROS Header.
    ///
    /// This function creates a header with the given frame id.
    ///
    /// \param t_frame_id The frame id.
    static std_msgs::Header createHeader(const std::string& t_frame_id);
};
}  // namespace utils