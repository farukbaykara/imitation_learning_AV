#include "aesk_object_localization/sign_finder/SignFinder.hpp"

namespace aesk_object_localization
{
SignFinder::SignFinder(ros::NodeHandle& t_node_handle) : m_node_handle{ t_node_handle }
{
    ROS_INFO("Creating object");
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Lidar topic: %s", m_lidar_topic.c_str());
    ROS_INFO("Detection 2D topic: %s", m_detection2d_topic.c_str());

    m_lidar_sub =
        m_node_handle.subscribe(m_lidar_topic, 1, &SignFinder::projectedCloudCallback, this);
    m_detection2d_sub =
        m_node_handle.subscribe(m_detection2d_topic, 1, &SignFinder::detection2dCallback, this);
    m_detection3d_pub = m_node_handle.advertise<vision_msgs::Detection3DArray>("detections", 1);
    m_pc2_pub = m_node_handle.advertise<sensor_msgs::PointCloud2>("noktalar", 1);

    ROS_INFO("Successfully launched sign_finder node.");
}

bool SignFinder::readParameters()
{
    if (!m_node_handle.getParam("lidar_topic", m_lidar_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("detection_2d_topic", m_detection2d_topic))
    {
        return false;
    }
    return true;
}

void SignFinder::projectedCloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_msg)
{
    m_projected_cloud = *t_msg;
}
void SignFinder::detection2dCallback(const vision_msgs::Detection2DArray& t_msg)
{
    if (m_projected_cloud.height == 0)
    {
        return;
    }
    m_detection2d_array = t_msg;

    const size_t raw_size{ m_projected_cloud.width * m_projected_cloud.height };

    // This will store 3D signs.
    vision_msgs::Detection3DArray signs;
    signs.header = utils::PointCloudOpen3D::createHeader();

    // Iterate through bboxes coming from object detection.
    for (const auto& bbox_2d : m_detection2d_array.detections)
    {
        // For each 2D bbox, iterate all points.
        sensor_msgs::PointCloud2Iterator<float> iterator_x{ m_projected_cloud, "x" };
        sensor_msgs::PointCloud2Iterator<float> iterator_y{ m_projected_cloud, "y" };
        sensor_msgs::PointCloud2Iterator<float> iterator_z{ m_projected_cloud, "z" };
        sensor_msgs::PointCloud2Iterator<float> iterator_px{ m_projected_cloud, "pX" };
        sensor_msgs::PointCloud2Iterator<float> iterator_py{ m_projected_cloud, "pY" };

        const double xmin{ bbox_2d.bbox.center.x - (bbox_2d.bbox.size_x / 2) };
        const double xmax{ bbox_2d.bbox.center.x + (bbox_2d.bbox.size_x / 2) };
        const double ymin{ bbox_2d.bbox.center.y - (bbox_2d.bbox.size_y / 2) };
        const double ymax{ bbox_2d.bbox.center.y + (bbox_2d.bbox.size_y / 2) };

        // This will store the 3D points inside the 2D bbox.
        std::shared_ptr<utils::PointCloudOpen3D> sign_cropped =
            std::make_shared<utils::PointCloudOpen3D>();

        // Iterate through points and if the point is inside 2D bbox, add it to sign_cropped.
        for (size_t i = 0; i < raw_size;
             ++i, ++iterator_px, ++iterator_py, ++iterator_x, ++iterator_y, ++iterator_z)
        {
            //            ROS_INFO("*iterator_px: %f", *iterator_px);
            //            ROS_INFO("*iterator_pY: %f", *iterator_py);
            //            ROS_INFO("*iterator_x:  %f", *iterator_x);
            //            ROS_INFO("*iterator_y:  %f", *iterator_y);
            //            ROS_INFO("*iterator_z:  %f", *iterator_z);
            //            ROS_INFO("i: %zu", i);
            //            ROS_INFO("raw_size: %zu", raw_size);
            //            ROS_INFO("xmin: %f", xmin);
            //            ROS_INFO("xmax: %f", xmax);
            //            ROS_INFO("ymin: %f", ymin);
            //            ROS_INFO("ymax: %f", ymax);
            if (xmin < *iterator_px && *iterator_px < xmax && ymin < *iterator_py &&
                *iterator_py < ymax)
            {
                const Eigen::Vector3d point{ *iterator_x, *iterator_y, *iterator_z };
                sign_cropped->points_.push_back(point);
            }
        }
        ROS_INFO("Points size: %zu", sign_cropped->points_.size());
        // We need at least four points to generate minimum bounding box.
        if (sign_cropped->points_.size() < 4)
        {
            ROS_WARN("Not enough points to create minimum bounding box.");
            continue;
        }

        Object object;
        if (sign_cropped->points_.size() < 4)
        {
            ROS_WARN("Not enough points to create oriented bounding box.");

            // If there are less than four points, create AxisAlignedBoundingBox.
            const auto new_bbox = sign_cropped->GetAxisAlignedBoundingBox();

            object.m_position = new_bbox.GetCenter();

            Eigen::Quaterniond orientation;
            orientation.x() = 0;
            orientation.y() = 0;
            orientation.z() = 0;
            orientation.w() = 1;
            object.m_orientation = orientation;

            const auto min_bound = new_bbox.GetMinBound();
            const auto max_bound = new_bbox.GetMaxBound();
            const Eigen::Vector3d size{ max_bound.x() - min_bound.x(),
                                        max_bound.y() - min_bound.y(),
                                        max_bound.z() - min_bound.z() };
            object.m_extent = size;
        }
        else
        {
            // Create new oriented bounding box.
            const auto new_bbox = sign_cropped->GetOrientedBoundingBox();

            object.m_position = new_bbox.GetCenter();
            const Eigen::Quaterniond orientation{ new_bbox.R_ };
            object.m_orientation = orientation;
            object.m_extent = new_bbox.extent_;
        }

        // Create detection 3D object and publish it.
        vision_msgs::Detection3D detection;
        detection.header = utils::PointCloudOpen3D::createHeader();
        detection.bbox.center.position.x = object.m_position.x();
        detection.bbox.center.position.y = object.m_position.y();
        detection.bbox.center.position.z = object.m_position.z();
        detection.bbox.center.orientation.x = object.m_orientation.x();
        detection.bbox.center.orientation.y = object.m_orientation.y();
        detection.bbox.center.orientation.z = object.m_orientation.z();
        detection.bbox.center.orientation.w = object.m_orientation.w();
        detection.bbox.size.x = object.m_extent.x();
        detection.bbox.size.y = object.m_extent.y();
        detection.bbox.size.z = object.m_extent.z();
        detection.source_cloud.header = utils::PointCloudOpen3D::createHeader();
        sign_cropped->ConvertToPointCloud2(detection.source_cloud);

        vision_msgs::ObjectHypothesisWithPose hypothesis;
        hypothesis.id = bbox_2d.results[0].id;
        hypothesis.score = bbox_2d.results[0].score;
        detection.results.push_back(hypothesis);

        signs.detections.push_back(detection);
    }
    m_detection3d_pub.publish(signs);
}
}  // namespace aesk_object_localization
