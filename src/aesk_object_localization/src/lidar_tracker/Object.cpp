#include "aesk_object_localization/lidar_tracker/Object.hpp"

namespace aesk_object_localization
{

Object::Object()
{
}

Object::Object(const long& t_id, const Eigen::Vector3d& t_position,
               const Eigen::Quaterniond& t_orientation)
  : m_id{ t_id }, m_position{ t_position }, m_orientation{ t_orientation }
{
}

bool Object::operator==(const Object& t_object) const
{
    if (m_id != t_object.m_id)
    {
        return false;
    }

    // Check objects by their location.
    const double distance{ calcDistance(t_object) };

    // If the distance is lower than a threshold, then this two objects are same.
    if (distance < 1)
        return true;
    return false;
}

double Object::calcDistance(const Object& t_object) const
{
    // Find the distance between objects. sqrt( (x1-x2)^2 + (y1-y2)^2 + (z1-z2)^2 )
    const double distance{ sqrt(pow(m_position.x() - t_object.m_position.x(), 2) +
                                pow(m_position.y() - t_object.m_position.y(), 2) +
                                pow(m_position.z() - t_object.m_position.z(), 2)) };
    return distance;
}

vision_msgs::Detection3D Object::getDetection3D() const
{
    vision_msgs::Detection3D detection;

    detection.header = utils::PointCloudOpen3D::createHeader();
    detection.bbox.center.position.x = m_position.x();
    detection.bbox.center.position.y = m_position.y();
    detection.bbox.center.position.z = m_position.z();
    detection.bbox.center.orientation.x = m_orientation.x();
    detection.bbox.center.orientation.y = m_orientation.y();
    detection.bbox.center.orientation.z = m_orientation.z();
    detection.bbox.center.orientation.w = m_orientation.w();
    detection.bbox.size.x = m_extent.x();
    detection.bbox.size.y = m_extent.y();
    detection.bbox.size.z = m_extent.z();

    //    detection.source_cloud.header = utils::PointCloudOpen3D::createHeader();
    //    m_cloud->ConvertToPointCloud2(detection.source_cloud);

    vision_msgs::ObjectHypothesisWithPose hypothesis;
    hypothesis.id = m_id;
    detection.results.push_back(hypothesis);

    return detection;
}

void Object::setPointCloud(const std::shared_ptr<open3d::geometry::PointCloud>& t_cloud)
{
    m_cloud->UpdateFromO3D(t_cloud);
}

bool Object::isZero() const
{
    if (m_position.x() == 0 && m_position.y() == 0 && m_position.z() == 0)
    {
        return true;
    }
    return false;
}

bool Object::isTooNear() const
{
    // if (m_position.x() == 0 && m_position.y() == 0 && m_position.z() == 0)
    if (-1 < m_position.x() && m_position.x() < 1 && -1 < m_position.y() && m_position.x() < 1)
    {
        return true;
    }
    return false;
}

vision_msgs::Detection3D Object::createGateDetection3D(const Eigen::Vector3d& t_gate_extent) const
{
    vision_msgs::Detection3D gate_detection{ getDetection3D() };
    gate_detection.bbox.size.x = t_gate_extent.x();
    gate_detection.bbox.size.y = t_gate_extent.y();
    gate_detection.bbox.size.z = t_gate_extent.z();
    return gate_detection;
}
}  // namespace aesk_object_localization