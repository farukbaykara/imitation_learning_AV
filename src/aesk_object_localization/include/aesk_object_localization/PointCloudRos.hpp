#pragma once

#include "aesk_object_localization/PointCloudInterface.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace pc
{
class PointCloudRos : public PointCloudInterface
{
  public:
    explicit PointCloudRos(sensor_msgs::PointCloud2 t_pc2);
    void Publish(const ros::Publisher& t_publisher) const override;
    int* begin();
    int* end();

  private:
    sensor_msgs::PointCloud2 m_pc2;
    sensor_msgs::PointCloud2ConstIterator<float> m_iterator_x;
    sensor_msgs::PointCloud2ConstIterator<float> m_iterator_y;
    sensor_msgs::PointCloud2ConstIterator<float> m_iterator_z;
};
}  // namespace pc