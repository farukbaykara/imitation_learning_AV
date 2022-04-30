#include "aesk_object_localization/lane3d_combiner/Lane3DCombiner.hpp"

namespace aesk_object_localization
{
Lane3DCombiner::Lane3DCombiner(ros::NodeHandle& t_node_handle)
  : m_node_handle{ t_node_handle }, m_synchronizer(policy_t(10), m_left_sub, m_right_sub)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Left topic: %s", m_left_topic.c_str());
    ROS_INFO("Right topic: %s", m_right_topic.c_str());

    m_left_sub.subscribe(m_node_handle, m_left_topic, 1);
    m_right_sub.subscribe(m_node_handle, m_right_topic, 1);

    m_synchronizer.registerCallback(boost::bind(&Lane3DCombiner::lane3DCallback, this, _1, _2));

    m_combined_pub = m_node_handle.advertise<sensor_msgs::PointCloud2>("lane3d_combined", 1);

    ROS_INFO("Successfully launched lane3d_combiner node.");
}

bool Lane3DCombiner::readParameters()
{
    if (!m_node_handle.getParam("left_topic", m_left_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("right_topic", m_right_topic))
    {
        return false;
    }
    return true;
}

void Lane3DCombiner::lane3DCallback(const sensor_msgs::PointCloud2ConstPtr& t_left,
                                    const sensor_msgs::PointCloud2ConstPtr& t_right)
{
    const size_t left_size{ t_left->height * t_left->width };
    const size_t right_size{ t_right->height * t_right->width };
    const size_t combined_size{ left_size + right_size };

    sensor_msgs::PointCloud2 combined_cloud;
    sensor_msgs::PointCloud2Modifier modifier(combined_cloud);

    modifier.resize(combined_size);
    modifier.setPointCloud2Fields(
        5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z",
        1, sensor_msgs::PointField::FLOAT32, "pX", 1, sensor_msgs::PointField::FLOAT32, "pY", 1,
        sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> combined_x(combined_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> combined_y(combined_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> combined_z(combined_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> combined_pX(combined_cloud, "pX");
    sensor_msgs::PointCloud2Iterator<float> combined_pY(combined_cloud, "pY");

    sensor_msgs::PointCloud2ConstIterator<float> left_x(*t_left, "x");
    sensor_msgs::PointCloud2ConstIterator<float> left_y(*t_left, "y");
    sensor_msgs::PointCloud2ConstIterator<float> left_z(*t_left, "z");
    sensor_msgs::PointCloud2ConstIterator<float> left_pX(*t_left, "pX");
    sensor_msgs::PointCloud2ConstIterator<float> left_pY(*t_left, "pY");

    sensor_msgs::PointCloud2ConstIterator<float> right_x(*t_right, "x");
    sensor_msgs::PointCloud2ConstIterator<float> right_y(*t_right, "y");
    sensor_msgs::PointCloud2ConstIterator<float> right_z(*t_right, "z");
    sensor_msgs::PointCloud2ConstIterator<float> right_pX(*t_right, "pX");
    sensor_msgs::PointCloud2ConstIterator<float> right_pY(*t_right, "pY");

    for (size_t i = 0; i < left_size; ++i, ++combined_x, ++combined_y, ++combined_z, ++combined_pX,
                ++combined_pY, ++left_x, ++left_y, ++left_z, ++left_pX, ++left_pY)
    {
        *combined_x = *left_x;
        *combined_y = *left_y;
        *combined_z = *left_z;
        *combined_pX = *left_pX;
        *combined_pY = *left_pY;
    }

    for (size_t i = 0; i < right_size; ++i, ++combined_x, ++combined_y, ++combined_z, ++combined_pX,
                ++combined_pY, ++right_x, ++right_y, ++right_z, ++right_pX, ++right_pY)
    {
        *combined_x = *right_x;
        *combined_y = *right_y;
        *combined_z = *right_z;
        *combined_pX = *right_pX;
        *combined_pY = *right_pY;
    }

    std_msgs::Header header;
    header.frame_id = "velodyne";
    combined_cloud.header = header;

    m_combined_pub.publish(combined_cloud);
}
}  // namespace aesk_object_localization