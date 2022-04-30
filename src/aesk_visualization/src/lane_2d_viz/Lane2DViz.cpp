#include "aesk_visualization/lane_2d_viz/Lane2DViz.hpp"

namespace aesk_visualization
{
Lane2DViz::Lane2DViz(ros::NodeHandle& t_node_handle_cloud)
  : m_node_handle(t_node_handle_cloud)
  , m_image_transport(m_node_handle)
  , m_is_cloud_received{ false }
  , m_is_image_received{ false }
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Lane point cloud topic: : %s", m_point_cloud_topic.c_str());
    ROS_INFO("Image topic: : %s", m_image_topic.c_str());

    m_point_cloud_sub = m_node_handle.subscribe<sensor_msgs::PointCloud2>(
        m_point_cloud_topic, 1, &Lane2DViz::cloudCallback, this);

    m_image_sub = m_image_transport.subscribe(m_image_topic, 1, &Lane2DViz::imageCallback, this);

    cv::namedWindow("Viz Lane", cv::WINDOW_NORMAL);

    ROS_INFO("Successfully launched node.");

    ros::Rate rate(20);
    while (ros::ok())
    {
        if (m_is_cloud_received && m_is_image_received)
        {
            drawPoints();
        }
        ros::spinOnce();
        rate.sleep();
    }
}

bool Lane2DViz::readParameters()
{
    if (!m_node_handle.getParam("lane_topic", m_point_cloud_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("image_topic", m_image_topic))
    {
        return false;
    }
    return true;
}

void Lane2DViz::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    m_is_cloud_received = true;
    m_cloud = *t_point_cloud;
}

void Lane2DViz::imageCallback(const sensor_msgs::ImageConstPtr& t_image)
{
    m_is_image_received = true;
    m_cv_ptr = cv_bridge::toCvCopy(t_image, sensor_msgs::image_encodings::BGR8);
}

void Lane2DViz::drawPoints()
{
    sensor_msgs::PointCloud2ConstIterator<float> cloud_x(m_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> cloud_y(m_cloud, "y");

    for (size_t i = 0; i < m_cloud.height * m_cloud.width; ++i, ++cloud_x, ++cloud_y)
    {
        cv::Point point{ static_cast<int>(*cloud_x), static_cast<int>(*cloud_y) };
        cv::Scalar color{ 0, 0, 100 };
        cv::circle(m_cv_ptr->image, point, 1, color, 4);
    }

    cv::imshow("Viz Lane", m_cv_ptr->image);
    cv::waitKey(1);
}

}  // namespace aesk_visualization