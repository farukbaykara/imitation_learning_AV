#pragma once

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace pcl
{
struct PointXYZpXpY
{
    PCL_ADD_POINT4D;
    float pX;
    float pY;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
}  // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZpXpY,
                                  (float, x, x)(float, y, y)(float, z, z)(float, pX, pX)(float, pY,
                                                                                         pY))
