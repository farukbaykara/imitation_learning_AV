#include "aesk_object_localization/PointCloudRos.hpp"

#include <utility>

namespace pc
{

PointCloudRos::PointCloudRos(sensor_msgs::PointCloud2 t_pc2)
  : m_pc2{ std::move(t_pc2) }
  , m_iterator_x{ m_pc2, "x" }
  , m_iterator_y{ m_pc2, "y" }
  , m_iterator_z{ m_pc2, "z" }
{
}

void PointCloudRos::Publish(const ros::Publisher& t_publisher) const
{
}
int* PointCloudRos::begin()
{
    return nullptr;
}
int* PointCloudRos::end()
{
    return nullptr;
}

}  // namespace pc
