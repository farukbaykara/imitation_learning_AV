#pragma once

#include "Object.hpp"
#include "aesk_object_localization/PointCloudOpen3D.hpp"
#include <ros/ros.h>

namespace aesk_object_localization
{
class ObjectTracker
{
  public:
    /*!
     * Constructor.
     * @param t_cloud the source cloud that will be used to find objects.
     */
    explicit ObjectTracker(std::shared_ptr<utils::PointCloudOpen3D>& t_cloud);

    /*!
     * Sets gate size.
     * @param t_gate_extent the gate size.
     */
    void setGateExtent(const Eigen::Vector3d& t_gate_extent);

    /*!
     * Sets gate margin.
     * @param t_gate_margin the gate size.
     */
    void setGateMargin(const Eigen::Vector3d& t_gate_extent);

    /*!
     * Track the given object in source cloud.
     * @param t_object the object which will be tracked.
     * @return the tracked object.
     */
    Object track(const Object& t_object, const bool& t_is_oriented) const;

  private:
    //! The source cloud which will be used for tracking.
    std::shared_ptr<utils::PointCloudOpen3D>& m_cloud;

    //! The gate will be used to crop source cloud.
    Eigen::Vector3d m_gate_extent;

    //! The margin of the gate.
    Eigen::Vector3d m_gate_margin;
};

}  // namespace aesk_object_localization