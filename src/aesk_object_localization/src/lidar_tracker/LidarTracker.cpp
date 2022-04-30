#include "aesk_object_localization/lidar_tracker/LidarTracker.hpp"

namespace aesk_object_localization
{
LidarTracker::LidarTracker(ros::NodeHandle& t_node_handle) : m_node_handle(t_node_handle)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Point cloud topic: %s", m_cloud_topic.c_str());
    ROS_INFO("Sign detection topic: %s", m_sign_detections_topic.c_str());
    ROS_INFO("Object count: %d", m_object_count);
    for (const auto& bound : m_tracker_ares_bounds)
    {
        ROS_INFO("%s: %f", bound.first.c_str(), bound.second);
    }

    m_object_histogram.fill(0);

    m_cloud_sub = m_node_handle.subscribe<sensor_msgs::PointCloud2>(
        m_cloud_topic, 1, &LidarTracker::cloudCallback, this);

    m_sign_detections_sub = m_node_handle.subscribe<vision_msgs::Detection3DArray>(
        m_sign_detections_topic, 1, &LidarTracker::signDetectionsCallback, this);

    m_tracked_detections_pub =
        m_node_handle.advertise<vision_msgs::Detection3DArray>("tracked_detections", 1);
    m_gate_publisher = m_node_handle.advertise<vision_msgs::Detection3DArray>("gates", 1);

    ROS_INFO("Successfully launched lidar_tracker node.");
}

bool LidarTracker::readParameters()
{
    if (!m_node_handle.getParam("cloud_topic", m_cloud_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("sign_detections_topic", m_sign_detections_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("object_count", m_object_count))
    {
        return false;
    }
    if (!m_node_handle.getParam("tracker_ares_bounds", m_tracker_ares_bounds))
    {
        return false;
    }
    if (!m_node_handle.getParam("tracker_gate_extent", m_gate_extent))
    {
        return false;
    }
    if (!m_node_handle.getParam("tracker_gate_margin", m_gate_margin))
    {
        return false;
    }
    return true;
}

void LidarTracker::signDetectionsCallback(const vision_msgs::Detection3DArrayConstPtr& t_detections)
{
    //    printActiveObjects();
    //    printStagingArea();
    // Check m_active_objects that it has the object or not.
    for (auto detection : t_detections->detections)
    {
        // Create the object that we'll search in active objects.
        const Eigen::Vector3d position{ detection.bbox.center.position.x,
                                        detection.bbox.center.position.y,
                                        detection.bbox.center.position.z };
        Eigen::Quaterniond orientation;
        orientation.x() = detection.bbox.center.orientation.x;
        orientation.y() = detection.bbox.center.orientation.y;
        orientation.z() = detection.bbox.center.orientation.z;
        orientation.w() = detection.bbox.center.orientation.w;
        const long id{ detection.results[0].id };
        Object object{ id, position, orientation };

        std::vector<Object>::iterator obj_iterator;
        obj_iterator = std::find(m_active_objects.begin(), m_active_objects.end(), object);

        if (obj_iterator != m_active_objects.end())
        {
            // The object already exists in active ones.
            ROS_INFO("Object %zu is already in active objects vector.", id);
            continue;
        }
        else
        {
            // The object doesn't exist in active ones.
            ROS_INFO("Couldn't find object %zu in active objects vector.", id);
            ROS_INFO("Object %zu counter: %d", id, 0);  // TODO!!!

            // Search the object in m_staging_area.
            std::vector<std::pair<Object, long>>::iterator staging_iterator;
            staging_iterator = std::find_if(
                m_staging_area.begin(), m_staging_area.end(),
                [object](const std::pair<Object, long>& obj) { return object == obj.first; });
            if (staging_iterator != m_staging_area.end())
            {
                // The object exists in staging area. Check its counter.
                if (staging_iterator->second == m_object_count)
                {
                    // Counter is satisfied. Add the object to active ones and reset counter.
                    m_active_objects.push_back(object);
                    ROS_INFO("Object %zu added to active ones.", id);

                    // Delete object from m_staging_area
                    m_staging_area.erase(staging_iterator);
                }
                else
                {
                    // Counter isn't satisfied. Increase it.
                    ++staging_iterator->second;
                    ROS_INFO("Increasing object %zu counter.", id);
                }
            }
            else
            {
                // The object doesn't exist in staging area. Add it.
                const std::pair<Object, long> new_object = std::make_pair(object, 0);
                m_staging_area.push_back(new_object);
                ROS_INFO("New object added to staging area.");
            }
        }
    }
}

void LidarTracker::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    if (m_active_objects.empty())
    {
        return;
    }

    // Convert PointCloud2 to utility::PointCloudOpen3D.
    auto cloud_raw = utils::PointCloudOpen3D::CreateFromPointCloud2(*t_point_cloud);

    const Eigen::Vector3d gate_extent{ m_gate_extent["x"], m_gate_extent["y"], m_gate_extent["z"] };
    const Eigen::Vector3d gate_margin{ m_gate_margin["x"], m_gate_margin["y"], m_gate_margin["z"] };

    ObjectTracker tracker{ cloud_raw };
    tracker.setGateExtent(gate_extent);
    tracker.setGateMargin(gate_margin);

    vision_msgs::Detection3DArray tracked_detections;
    vision_msgs::Detection3DArray gates;
    tracked_detections.header = utils::PointCloudOpen3D::createHeader();
    gates.header = utils::PointCloudOpen3D::createHeader();

    std::vector<int> delete_ids{};

    // For each object in m_active_objects, find new positions and orientations.
    for (size_t i = 0; i < m_active_objects.size(); ++i)
    {
        Object new_object = tracker.track(m_active_objects[i], true);

        printActiveObjects();

        // Check if the object is within the specified boundaries.
        if (!(m_tracker_ares_bounds["xmin"] < new_object.m_position.x() &&
              new_object.m_position.x() < m_tracker_ares_bounds["xmax"] &&
              m_tracker_ares_bounds["ymin"] < new_object.m_position.y() &&
              new_object.m_position.y() < m_tracker_ares_bounds["ymax"] &&
              m_tracker_ares_bounds["zmin"] < new_object.m_position.z() &&
              new_object.m_position.z() < m_tracker_ares_bounds["zmax"]))
        {
            // Object is out of specified area. Delete it.
            delete_ids.push_back(static_cast<int>(i));
            ROS_INFO("Object %zu is out of bounds, it will be deleted.", m_active_objects[i].m_id);
            continue;
        }

        if (new_object.isZero())
        {
            ROS_INFO("Object %d is zero.", new_object.m_id);
            delete_ids.push_back(static_cast<int>(i));
            continue;
        }

        if (new_object.isTooNear())
        {
            ROS_INFO("Object %d is too near.", new_object.m_id);
            delete_ids.push_back(static_cast<int>(i));
            continue;
        }

        // Create detection 3D object.
        const vision_msgs::Detection3D detection = new_object.getDetection3D();
        tracked_detections.detections.push_back(detection);

        const vision_msgs::Detection3D gate_detection =
            new_object.createGateDetection3D(gate_extent);
        gates.detections.push_back(gate_detection);

        m_active_objects[i] = new_object;

        //        ROS_INFO("Tracked object %zu.", m_active_objects[i].m_id);
    }

    // Delete the outsider objects.
    std::reverse(delete_ids.begin(), delete_ids.end());
    for (auto index : delete_ids)
    {
        quickDelete(m_active_objects, index);
        ROS_INFO("Deleted object %d.", index);
    }

    m_tracked_detections_pub.publish(tracked_detections);
    m_gate_publisher.publish(gates);
}

void LidarTracker::printActiveObjects()
{
    std::cout << "------\nACTIVE OBJECTS:\n";
    for (const auto& object : m_active_objects)
    {
        std::cout << "ID: " << object.m_id << '\n';
        std::cout << "Position:\n" << object.m_position << '\n';
        std::cout << "Orıentation:\n"
                  << object.m_orientation.x() << "," << object.m_orientation.y() << ","
                  << object.m_orientation.z() << "," << object.m_orientation.w() << '\n'
                  << '\n';
    }
    std::cout << '\n';
}

void LidarTracker::printStagingArea()
{
    std::cout << "STAGING AREA:\n";
    for (const auto& object : m_staging_area)
    {
        std::cout << "ID: " << object.first.m_id << '\n';
        std::cout << "Position:\n" << object.first.m_position << '\n';
        std::cout << "Orıentation:\n"
                  << object.first.m_orientation.x() << "," << object.first.m_orientation.y() << ","
                  << object.first.m_orientation.z() << "," << object.first.m_orientation.w() << '\n'
                  << '\n';
    }
}

void LidarTracker::quickDelete(std::vector<Object>& t_vector, const int& t_index)
{
    t_vector[t_index] = t_vector.back();
    t_vector.pop_back();
}

}  // namespace aesk_object_localization