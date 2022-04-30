#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace pc
{

class PointCloudCreator
{

};

class PointCloudInterface
{
  public:
    virtual void Publish(const ros::Publisher& t_publisher) const = 0;
};
}  // namespace pc