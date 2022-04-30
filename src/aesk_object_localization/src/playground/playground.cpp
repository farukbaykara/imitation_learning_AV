//
// Created by meb on 1/15/22.
//

#include <random>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <aesk_object_localization/playground_paramsConfig.h>
#include "open3d/Open3D.h"

ros::Publisher lidar_pub;
ros::Publisher cluster_pub;
ros::Publisher marker_array_pub;
aesk_object_localization::playground_paramsConfig conf;

void draw_gui(const std::shared_ptr<open3d::geometry::PointCloud>& t_cloud)
{
    const std::vector<std::shared_ptr<const open3d::geometry::Geometry>> list{ t_cloud };
    open3d::visualization::DrawGeometries(list);
}

void publish_cloud(const ros::Publisher& publisher, const std::vector<Eigen::Vector3d>& points,
                   const std::vector<Eigen::Vector3d>& colors = {})
{
    sensor_msgs::PointCloud2 p_point_cloud;
    sensor_msgs::PointCloud2Modifier modifier(p_point_cloud);
    modifier.resize(points.size());
    modifier.setPointCloud2Fields(
        6, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z",
        1, sensor_msgs::PointField::FLOAT32, "r", 1, sensor_msgs::PointField::FLOAT32, "g", 1,
        sensor_msgs::PointField::FLOAT32, "b", 1, sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_x(p_point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_y(p_point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_z(p_point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_r(p_point_cloud, "r");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_g(p_point_cloud, "g");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_b(p_point_cloud, "b");

    ROS_INFO("Cluster size: %d", points.size());

    for (size_t i = 0; i < points.size(); ++i, ++p_point_cloud_x, ++p_point_cloud_y,
                ++p_point_cloud_z, ++p_point_cloud_r, ++p_point_cloud_g, ++p_point_cloud_b)
    {
        *p_point_cloud_x = points[i].x();
        *p_point_cloud_y = points[i].y();
        *p_point_cloud_z = points[i].z();

        if (colors.empty())
        {
            *p_point_cloud_r = 0;
            *p_point_cloud_g = 0;
            *p_point_cloud_b = 0;
        }
        else
        {
            *p_point_cloud_r = static_cast<float>(colors[i].x());
            *p_point_cloud_g = static_cast<float>(colors[i].y());
            *p_point_cloud_b = static_cast<float>(colors[i].z());
        }
    }

    std_msgs::Header header;
    header.frame_id = "velodyne";
    p_point_cloud.header = header;
    publisher.publish(p_point_cloud);
}

void publish_markers(const std::vector<std::shared_ptr<open3d::geometry::PointCloud>>& clusters)
{
    visualization_msgs::MarkerArray marker_array;
    int id = 1;
    for (const auto& cluster : clusters)
    {
        //        draw_gui(cluster);
        std::shared_ptr<open3d::geometry::OrientedBoundingBox> bbox =
            std::make_shared<open3d::geometry::OrientedBoundingBox>(
                cluster->GetOrientedBoundingBox());
//        open3d::geometry::OrientedBoundingBox bbox = cluster->GetOrientedBoundingBox();
//        const tf2::Matrix3x3 rot{ bbox.R_.row(0)[0], bbox.R_.row(0)[1], bbox.R_.row(0)[2],
//                                  bbox.R_.row(1)[0], bbox.R_.row(1)[1], bbox.R_.row(1)[2],
//                                  bbox.R_.row(2)[0], bbox.R_.row(2)[1], bbox.R_.row(2)[2] };
        const tf2::Matrix3x3 rot{ bbox->R_.row(0)[0], bbox->R_.row(0)[1], bbox->R_.row(0)[2],
                                  bbox->R_.row(1)[0], bbox->R_.row(1)[1], bbox->R_.row(1)[2],
                                  bbox->R_.row(2)[0], bbox->R_.row(2)[1], bbox->R_.row(2)[2] };

//        const std::vector<std::shared_ptr<const open3d::geometry::Geometry>> list{ cluster, bbox };
//        open3d::visualization::DrawGeometries(list);

        const Eigen::Vector3d min_bounds = bbox->GetMinBound();
        const Eigen::Vector3d max_bounds = bbox->GetMaxBound();

        const double scale_x = (max_bounds.x() - min_bounds.x());
        const double scale_y = (max_bounds.y() - min_bounds.y());
        const double scale_z = (max_bounds.z() - min_bounds.z());

        tf2::Quaternion qua;
        rot.getRotation(qua);

        visualization_msgs::Marker sign_marker;
        sign_marker.header.frame_id = "velodyne";
        sign_marker.header.stamp = ros::Time::now();
        sign_marker.id = id;
        sign_marker.type = sign_marker.CUBE;
        sign_marker.action = sign_marker.ADD;
        sign_marker.pose.position.x = bbox->GetCenter().x();
        sign_marker.pose.position.y = bbox->GetCenter().y();
        sign_marker.pose.position.z = bbox->GetCenter().z();
        sign_marker.pose.orientation.x = qua.x();
        sign_marker.pose.orientation.y = qua.y();
        sign_marker.pose.orientation.z = qua.z();
        sign_marker.pose.orientation.w = qua.w();
        sign_marker.scale.x = scale_x;
        sign_marker.scale.y = scale_y;
        sign_marker.scale.z = scale_z;
        sign_marker.color.a = 1;
        sign_marker.color.r = 1;
        sign_marker.color.g = 0;
        sign_marker.color.b = 1;
        sign_marker.lifetime = ros::Duration(1);
        marker_array.markers.push_back(sign_marker);
        ++id;
    }
    marker_array_pub.publish(marker_array);
}

int generateRandom()
{
    std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(1, 255);
    return dist6(rng);
}

void lidar_callback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    std::shared_ptr<open3d::geometry::PointCloud> cloud =
        std::make_shared<open3d::geometry::PointCloud>();

    // Create PointCloud2 iterators.
    sensor_msgs::PointCloud2ConstIterator<float> iterator_x(*t_point_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iterator_y(*t_point_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iterator_z(*t_point_cloud, "z");

    const size_t cloud_size = t_point_cloud->width * t_point_cloud->height;

    for (size_t i = 0; i < cloud_size; ++i, ++iterator_x, ++iterator_y, ++iterator_z)
    {
        Eigen::Vector3d point{ *iterator_x, *iterator_y, *iterator_z };
        cloud->points_.push_back(point);
    }

    Eigen::Vector3d min{ static_cast<double>(conf.min_x), static_cast<double>(conf.min_y),
                         static_cast<double>(conf.min_z) };
    Eigen::Vector3d max{ static_cast<double>(conf.max_x), static_cast<double>(conf.max_y),
                         static_cast<double>(conf.max_z) };
    open3d::geometry::AxisAlignedBoundingBox roi{ min, max };

    cloud = cloud->Crop(roi);
    if (cloud->points_.empty())
    {
        return;
    }

    std::vector<int> labels = cloud->ClusterDBSCAN(conf.eps, conf.min_points, false);

    std::set<int> unique_labels{ labels.begin(), labels.end() };
    size_t cluster_number = unique_labels.size();
    if (unique_labels.count(-1) != 0)
    {
        cluster_number = cluster_number - 1;
    }

    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> clusters;
    clusters.reserve(cluster_number);

    std::vector<Eigen::Vector3d> cluster_colors;

    for (size_t i = 0; i < cluster_number; ++i)
    {
        std::shared_ptr<open3d::geometry::PointCloud> empty_cloud =
            std::make_shared<open3d::geometry::PointCloud>();
        clusters.push_back(empty_cloud);

        const Eigen::Vector3d color{ generateRandom(), generateRandom(), generateRandom() };

        cluster_colors.push_back(color);
    }

    std::shared_ptr<open3d::geometry::PointCloud> outCloud =
        std::make_shared<open3d::geometry::PointCloud>();

    for (size_t i = 0; i < labels.size(); ++i)
    {
        if (labels[i] == -1)
        {
            continue;
        }
        for (size_t j = 0; j < cluster_number; ++j)
        {
            const Eigen::Vector3d point = cloud->points_[i];
            clusters[j]->points_.push_back(point);

            outCloud->points_.push_back(point);
            outCloud->colors_.push_back(cluster_colors[j]);
        }
    }

    publish_markers(clusters);
    publish_cloud(lidar_pub, cloud->points_);
    publish_cloud(cluster_pub, outCloud->points_, outCloud->colors_);
    //    draw_gui(cloud);
}

void paramCallback(aesk_object_localization::playground_paramsConfig& config, uint32_t level)
{
    ROS_INFO("min roi: %f, %f, %f", config.min_x, config.min_y, config.min_z);
    ROS_INFO("max roi: %f, %f, %f", config.max_x, config.max_y, config.max_z);
    conf = config;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "playground");
    ros::NodeHandle node_handle("~");

    lidar_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/o3d", 1);
    cluster_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/clusters", 1);
    marker_array_pub = node_handle.advertise<visualization_msgs::MarkerArray>("/bboxes", 1);

    ros::Subscriber lidar_sub = node_handle.subscribe("/velodyne_points", 1, &lidar_callback);

    dynamic_reconfigure::Server<aesk_object_localization::playground_paramsConfig> server;
    dynamic_reconfigure::Server<aesk_object_localization::playground_paramsConfig>::CallbackType f;

    f = boost::bind(&paramCallback, _1, _2);
    server.setCallback(f);

    ROS_INFO("Successfully launched node.");
    ros::spin();

    //    auto sphere = open3d::geometry::TriangleMesh::CreateSphere(1.0);
    //    sphere->ComputeVertexNormals();
    //    sphere->PaintUniformColor({ 0.0, 1.0, 0.0 });
    //    open3d::visualization::DrawGeometries({ sphere });

    return 0;
}