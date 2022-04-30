#pragma once

#include <Eigen/Dense>
#include <vision_msgs/Detection3D.h>
#include "open3d/Open3D.h"
#include "aesk_object_localization/PointCloudOpen3D.hpp"

namespace aesk_object_localization
{
class Object
{
  public:
    /*!
     * Constructor.
     */
    Object();

    /*!
     * Constructor.
     * @param t_id the object id comes from object detection.
     * @param t_position the 3D position of the object.
     * @param t_orientation the orientation of the object.
     */
    Object(const long& t_id, const Eigen::Vector3d& t_position,
           const Eigen::Quaterniond& t_orientation);

    /*!
     * Sets the points of the object.
     * @param t_cloud the desired object.
     */
    void setPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& t_cloud);

    /*!
     * Calculate the distance between objects.
     * @param t_object the reference object.
     */
    double calcDistance(const Object& t_object) const;

    /*!
     * The == operator.
     * @param t_object the reference object.
     */
    bool operator==(const Object& t_object) const;

    /*!
     * Get the vision_msgs/Detection3D object.
     * @return the Detection3D object.
     */
    vision_msgs::Detection3D getDetection3D() const;

    /*!
     * Get the vision_msgs/Detection3D object for the given gate extent.
     * @param t_gate_extent the size of gate.
     * @return the Detection3D object.
     */
    vision_msgs::Detection3D createGateDetection3D(const Eigen::Vector3d& t_gate_extent) const;

    /*!
     * Check whether all coordinates of the object is zero.
     * @return true if all coordinates are zero.
     */
    bool isZero() const;

    /*!
     * Check whether all coordinates of the object is near car.
     * @return true if all coordinates are near.
     */
    bool isTooNear() const;

    //! The object ID that comes from object detection.
    long m_id;

    //! The x,y,z position of the object.
    Eigen::Vector3d m_position;

    //! The orientation of the object with quaternion.
    Eigen::Quaterniond m_orientation;

    //! The size of the object.
    Eigen::Vector3d m_extent;

    //! The points inside the bounding box.
    std::shared_ptr<utils::PointCloudOpen3D> m_cloud;
};
}  // namespace aesk_object_localization