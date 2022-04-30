#include "aesk_visualization/projected_cloud_viz/ProjectedCloudViz.hpp"

namespace aesk_visualization
{
ProjectedCloudViz::ProjectedCloudViz(ros::NodeHandle& t_node_handle_cloud)
  : m_node_handle(t_node_handle_cloud)
  , m_image_transport(m_node_handle)
  , m_is_cloud_received{ false }
  , m_is_lane_received{ false }
  , m_is_image_received{ false }
  , m_is_bboxes_received{ false }
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Lane point cloud topic: : %s", m_point_cloud_topic.c_str());
    ROS_INFO("Image topic: : %s", m_image_topic.c_str());

    m_point_cloud_sub = m_node_handle.subscribe<sensor_msgs::PointCloud2>(
        m_point_cloud_topic, 1, &ProjectedCloudViz::cloudCallback, this);

    m_lane_cloud_sub = m_node_handle.subscribe<sensor_msgs::PointCloud2>(
        m_lane_cloud_topic, 1, &ProjectedCloudViz::laneCloudCallback, this);

    m_image_sub =
        m_image_transport.subscribe(m_image_topic, 1, &ProjectedCloudViz::imageCallback, this);

    m_bboxes_sub = m_node_handle.subscribe<vision_msgs::Detection2DArray>(
        m_bboxes_topic, 1, &ProjectedCloudViz::detectionsCallback, this);

    cv::namedWindow("Projected Point Cloud", cv::WINDOW_NORMAL);

    ROS_INFO("Successfully launched node.");

    ros::Rate rate(20);
    ros::Rate rate_waiting(1);
    while (ros::ok())
    {
        if (m_is_image_received)
        {
            if (m_is_cloud_received)
            {
                drawProjectedPoints(m_cloud, cv::Scalar{ 87, 182, 240 }, 2);
            }
            if (m_is_lane_received)
            {
                drawProjectedPoints(m_lane, cv::Scalar{ 200, 0, 200 }, 2);
            }
            if (m_is_bboxes_received)
            {
                drawBoundingBoxes(m_detections, cv::Scalar{ 100, 50, 50 }, 2);
            }
            cv::imshow("Projected Point Cloud", m_cv_ptr->image);
            cv::waitKey(1);
        }
        else
        {
            ROS_INFO("Waiting for image.");
            rate_waiting.sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

bool ProjectedCloudViz::readParameters()
{
    if (!m_node_handle.getParam("projected_cloud_topic", m_point_cloud_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("image_topic", m_image_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("lane_cloud_topic", m_lane_cloud_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("bboxes_topic", m_bboxes_topic))
    {
        return false;
    }
    return true;
}

void ProjectedCloudViz::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    m_is_cloud_received = true;
    m_cloud = *t_point_cloud;
}

void ProjectedCloudViz::laneCloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    m_is_lane_received = true;
    m_lane = *t_point_cloud;
}

void ProjectedCloudViz::imageCallback(const sensor_msgs::ImageConstPtr& t_image)
{
    m_is_image_received = true;
    m_cv_ptr = cv_bridge::toCvCopy(t_image, sensor_msgs::image_encodings::BGR8);
}

void ProjectedCloudViz::detectionsCallback(
    const vision_msgs::Detection2DArrayConstPtr& t_detections)
{
    m_is_bboxes_received = true;
    m_detections = *t_detections;
}

void ProjectedCloudViz::drawProjectedPoints(const sensor_msgs::PointCloud2& t_cloud,
                                            const cv::Scalar& t_color, const int& t_thickness)
{
    sensor_msgs::PointCloud2ConstIterator<float> cloud_x(t_cloud, "pX");
    sensor_msgs::PointCloud2ConstIterator<float> cloud_y(t_cloud, "pY");

    for (size_t i = 0; i < t_cloud.height * t_cloud.width; ++i, ++cloud_x, ++cloud_y)
    {
        cv::Point point{ static_cast<int>(*cloud_x), static_cast<int>(*cloud_y) };
        cv::circle(m_cv_ptr->image, point, 1, t_color, t_thickness);
    }
}

void ProjectedCloudViz::drawBoundingBoxes(const vision_msgs::Detection2DArray& t_detections,
                                          const cv::Scalar& t_color, const int& t_thickness)
{
    for (const auto& box : t_detections.detections)
    {
        cv::Point min_bound{ static_cast<int>(box.bbox.center.x - (box.bbox.size_x / 2)),
                             static_cast<int>(box.bbox.center.y - (box.bbox.size_y / 2)) };
        cv::Point max_bound{ static_cast<int>(box.bbox.center.x + (box.bbox.size_x / 2)),
                             static_cast<int>(box.bbox.center.y + (box.bbox.size_y / 2)) };
        cv::rectangle(m_cv_ptr->image, min_bound, max_bound, t_color, t_thickness);
    }
}

}  // namespace aesk_visualization