//
// Created by goktug on 20.01.2021.
//

#ifndef LEO_CAMERA_LIDAR_CALIBRATION_TOOL_POINTXYZIR_H
#define LEO_CAMERA_LIDAR_CALIBRATION_TOOL_POINTXYZIR_H

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>

namespace pcl {
    struct PointXYZIR
    {
        PCL_ADD_POINT4D;
        float    intensity;
        uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (uint16_t, ring, ring))

#endif //LEO_CAMERA_LIDAR_CALIBRATION_TOOL_POINTXYZIR_H
