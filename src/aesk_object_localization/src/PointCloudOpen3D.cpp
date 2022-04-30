#include "aesk_object_localization/PointCloudOpen3D.hpp"

namespace utils
{
std::shared_ptr<PointCloudOpen3D>
PointCloudOpen3D::CreateFromPointCloud2(const sensor_msgs::PointCloud2& t_cloud)
{
    std::shared_ptr<PointCloudOpen3D> o3d_cloud = std::make_shared<PointCloudOpen3D>();

    sensor_msgs::PointCloud2ConstIterator<float> x_iter{ t_cloud, "x" };
    sensor_msgs::PointCloud2ConstIterator<float> y_iter{ t_cloud, "y" };
    sensor_msgs::PointCloud2ConstIterator<float> z_iter{ t_cloud, "z" };

    const size_t cloud_size = t_cloud.width * t_cloud.height;

    for (size_t i = 0; i < cloud_size; ++i, ++x_iter, ++y_iter, ++z_iter)
    {
        const Eigen::Vector3d point{ *x_iter, *y_iter, *z_iter };
        o3d_cloud->points_.push_back(point);
    }

    return o3d_cloud;
}

void PointCloudOpen3D::ConvertToPointCloud2(sensor_msgs::PointCloud2& t_msg) const
{
    sensor_msgs::PointCloud2Modifier modifier{ t_msg };

    if (colors_.size() == 0)
    {
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points_.size());

        sensor_msgs::PointCloud2Iterator<float> x_iter{ t_msg, "x" };
        sensor_msgs::PointCloud2Iterator<float> y_iter{ t_msg, "y" };
        sensor_msgs::PointCloud2Iterator<float> z_iter{ t_msg, "z" };

        for (size_t i = 0; i < points_.size(); ++i, ++x_iter, ++y_iter, ++z_iter)
        {
            *x_iter = static_cast<float>(points_[i].x());
            *y_iter = static_cast<float>(points_[i].y());
            *z_iter = static_cast<float>(points_[i].z());
        }
    }
    else
    {
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(points_.size());

        sensor_msgs::PointCloud2Iterator<float> x_iter{ t_msg, "x" };
        sensor_msgs::PointCloud2Iterator<float> y_iter{ t_msg, "y" };
        sensor_msgs::PointCloud2Iterator<float> z_iter{ t_msg, "z" };
        sensor_msgs::PointCloud2Iterator<float> r_iter{ t_msg, "r" };
        sensor_msgs::PointCloud2Iterator<float> g_iter{ t_msg, "g" };
        sensor_msgs::PointCloud2Iterator<float> b_iter{ t_msg, "b" };

        for (size_t i = 0; i < points_.size();
             ++i, ++x_iter, ++y_iter, ++z_iter, ++r_iter, ++g_iter, ++b_iter)
        {
            *x_iter = static_cast<float>(points_[i].x());
            *y_iter = static_cast<float>(points_[i].y());
            *z_iter = static_cast<float>(points_[i].z());
            *r_iter = static_cast<float>(colors_[i].x());
            *g_iter = static_cast<float>(colors_[i].y());
            *b_iter = static_cast<float>(colors_[i].z());
        }
    }
}

void PointCloudOpen3D::Publish(const ros::Publisher& t_publisher) const
{
    std_msgs::Header header;
    header.frame_id = "velodyne";
    header.stamp = ros::Time::now();
    Publish(t_publisher, header);
}

void PointCloudOpen3D::Publish(const ros::Publisher& t_publisher,
                               const std_msgs::Header& t_header) const
{
    sensor_msgs::PointCloud2 cloud;
    cloud.header = t_header;
    ConvertToPointCloud2(cloud);
    t_publisher.publish(cloud);
}

void PointCloudOpen3D::UpdateFromO3D(const std::shared_ptr<o3dCloud>& t_cloud)
{
    points_ = t_cloud->points_;
    colors_ = t_cloud->colors_;
    normals_ = t_cloud->normals_;
    covariances_ = t_cloud->covariances_;
}

std_msgs::Header PointCloudOpen3D::createHeader()
{
    return createHeader("velodyne");
}

std_msgs::Header PointCloudOpen3D::createHeader(const std::string& t_frame_id)
{
    std_msgs::Header header;
    header.frame_id = t_frame_id;
    header.stamp = ros::Time::now();
    return header;
}

}  // namespace utils
