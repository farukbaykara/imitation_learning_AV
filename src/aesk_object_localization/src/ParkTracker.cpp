#include "aesk_object_localization/ParkTracker.hpp"

namespace aesk_object_localization
{
ParkTracker::ParkTracker(ros::NodeHandle& t_node_handle)
  : m_node_handle(t_node_handle)
  , m_object_counter{ 0 }
  , m_is_finished{ false }
  , m_action_client("park_cluster", true)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Point cloud topic: %s", m_cloud_topic.c_str());
    ROS_INFO("Finish line topic: %s", m_finish_topic.c_str());
    ROS_INFO("Sign markers topic: %s", m_sign_markers_topic.c_str());
    ROS_INFO("Park sign YOLO id: %d", m_park_id);
    ROS_INFO("Object count: %d", m_object_count);
    for (auto a : m_tracker_box)
    {
        ROS_INFO("%s: %f", a.first.c_str(), a.second);
    }

    ROS_INFO("Waiting for action server to start.");
    m_action_client.waitForServer();
    ROS_INFO("Action server started.");

    m_finish_line_subscriber = m_node_handle.subscribe<std_msgs::Bool>(
        m_finish_topic, 1, &ParkTracker::finishCallback, this);

    m_sign_markers_subscriber = m_node_handle.subscribe<visualization_msgs::MarkerArray>(
        m_sign_markers_topic, 1, &ParkTracker::signCallback, this);

    m_projected_cloud_subscriber = m_node_handle.subscribe<sensor_msgs::PointCloud2>(
        m_cloud_topic, 1, &ParkTracker::cloudCallback, this);

    m_point_cloud_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>("my_box", 1);

    m_park_sign_publisher = m_node_handle.advertise<visualization_msgs::Marker>("park_sign", 1);

    m_area_publisher = m_node_handle.advertise<visualization_msgs::Marker>("park_area", 1);

    ROS_INFO("Successfully launched node.");
}

bool ParkTracker::readParameters()
{
    if (!m_node_handle.getParam("cloud_topic", m_cloud_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("finish", m_finish_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("park_id", m_park_id))
    {
        return false;
    }
    if (!m_node_handle.getParam("object_count", m_object_count))
    {
        return false;
    }
    if (!m_node_handle.getParam("tracker_box", m_tracker_box))
    {
        return false;
    }
    if (!m_node_handle.getParam("sign_markers_topic", m_sign_markers_topic))
    {
        return false;
    }
    return true;
}

double ParkTracker::calculateDistance(const double& t_x_coordinate, const double& t_y_coordinate,
                                      const double& t_z_coordinate)
{
    return sqrt(pow(t_x_coordinate, 2.0) + pow(t_y_coordinate, 2.0) + pow(t_z_coordinate, 2.0));
}

void ParkTracker::finishCallback(const std_msgs::BoolConstPtr& t_is_finished)
{
    m_is_finished = t_is_finished->data;
    // if (m_is_finished)
    // {
    //     std::cout << "Finish true\n";
    // }
    // else
    // {
    //     std::cout << "Finish false\n";
    // }
}

void ParkTracker::signCallback(const visualization_msgs::MarkerArrayConstPtr& t_marker_array)
{
    if (!m_is_finished || m_object_counter > m_object_count || t_marker_array->markers.size() == 0)
    {
        // If we haven't finished the parkour yet, return until we finish it. OR
        // We may have seen park sign enough times. In that case, we are tracking it. OR
        // Sometimes marker array arrives empty. In that case we shouldn't continue rest.
        return;
    }

    // Pair: id, distance
    std::vector<std::pair<int, double>> sign_distances;
    for (size_t i = 0; i < t_marker_array->markers.size(); i++)
    {
        if (t_marker_array->markers[i].id == m_park_id)
        {
            double sign_distance = calculateDistance(t_marker_array->markers[i].pose.position.x,
                                                     t_marker_array->markers[i].pose.position.y,
                                                     t_marker_array->markers[i].pose.position.z);
            sign_distances.push_back(std::pair<int, double>{ i, sign_distance });
        }
    }

    // We may see other signs rather than park.
    // In that case, "sign_distances" vector may contain other signs and it may be empty.
    // Because we push_back only park signs.
    if (sign_distances.size() == 0)
    {
        return;
    }

    //  Compare only second elements of pairs and get the miminum value of the vector, which is the
    //  nearest park sign.
    std::pair<int, double> nearest_sign =
        *std::min_element(sign_distances.cbegin(), sign_distances.cend(),
                          [](const auto& lhs, const auto& rhs) { return lhs.second < rhs.second; });

    m_last_x = t_marker_array->markers[nearest_sign.first].pose.position.x;
    m_last_y = t_marker_array->markers[nearest_sign.first].pose.position.y;
    m_last_z = t_marker_array->markers[nearest_sign.first].pose.position.z;

    ++m_object_counter;
    ROS_INFO("Object counter: %d", m_object_counter);
}

void ParkTracker::cloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    if (!m_is_finished || m_object_counter <= m_object_count)
    {
        return;
    }

    // ROS_INFO("Last x: %f", m_last_x);
    // ROS_INFO("Last y: %f", m_last_y);
    // ROS_INFO("Last z: %f", m_last_z);

    // Convert ROS PointCloud2 message to PCL type.
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_raw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*t_point_cloud, *point_cloud_raw);

    // Crop the point cloud so that only the sign remains.
    pcl::CropBox<pcl::PointXYZ> cropBoxFilter(true);
    cropBoxFilter.setInputCloud(point_cloud_raw);
    Eigen::Vector4f min_pt(m_last_x - m_tracker_box["x_min"], m_last_y - m_tracker_box["y_min"],
                           m_last_z - m_tracker_box["z_min"], 1.0f);
    Eigen::Vector4f max_pt(m_last_x + m_tracker_box["x_max"], m_last_y + m_tracker_box["y_max"],
                           m_last_z + m_tracker_box["z_max"], 1.0f);

    cropBoxFilter.setMin(min_pt);
    cropBoxFilter.setMax(max_pt);

    std::vector<int> indices;
    cropBoxFilter.filter(indices);

    pcl::PointCloud<pcl::PointXYZ> point_cloud_sign;
    cropBoxFilter.filter(point_cloud_sign);

    // Convert PCL cloud to PointCloud2 in order to send goal.
    sensor_msgs::PointCloud2 goal_cloud;
    pcl::toROSMsg(point_cloud_sign, goal_cloud);

    // Create and send goal.
    aesk_object_localization::ParkGoal action_goal;
    action_goal.cloud = goal_cloud;
    m_action_client.sendGoal(action_goal);

    // Wait for the action to return.
    bool finished_before_timeout = m_action_client.waitForResult(ros::Duration(0.2));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = m_action_client.getState();
        // ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }

    // We're updating sign coordintaes with the new DBSCAN clustered values.
    m_last_x = m_action_client.getResult()->point.x;
    m_last_y = m_action_client.getResult()->point.y;
    m_last_z = m_action_client.getResult()->point.z;

    // // Publish croped cloud.
    // sensor_msgs::PointCloud2 point_cloud_projected_msg;
    // pcl::toROSMsg(point_cloud_sign, point_cloud_projected_msg);
    // point_cloud_projected_msg.header.frame_id = "velodyne";
    // point_cloud_projected_msg.header.stamp = ros::Time::now();
    // m_point_cloud_publisher.publish(point_cloud_projected_msg);

    // Publish park sign marker.
    visualization_msgs::Marker park_marker;
    park_marker.header.frame_id = "velodyne";
    park_marker.header.stamp = ros::Time::now();
    park_marker.id = m_park_id;
    park_marker.type = park_marker.CUBE;
    park_marker.action = park_marker.ADD;
    park_marker.pose.position.x = m_last_x;
    park_marker.pose.position.y = m_last_y;
    park_marker.pose.position.z = m_last_z;
    park_marker.pose.orientation.x = 0;
    park_marker.pose.orientation.y = 0;
    park_marker.pose.orientation.z = 0;
    park_marker.pose.orientation.w = 1;
    park_marker.scale.x = 0.1;
    park_marker.scale.y = 0.8;
    park_marker.scale.z = 0.8;
    park_marker.color.a = 1;
    park_marker.color.r = 0;
    park_marker.color.g = 0;
    park_marker.color.b = 1;
    park_marker.lifetime = ros::Duration(0.5);
    m_park_sign_publisher.publish(park_marker);

    // Publish park area marker.
    double x_min = m_last_x - m_tracker_box["x_min"];
    double y_min = m_last_y - m_tracker_box["y_min"];
    double z_min = m_last_z - m_tracker_box["z_min"];
    double x_max = m_last_x + m_tracker_box["x_max"];
    double y_max = m_last_y + m_tracker_box["y_max"];
    double z_max = m_last_z + m_tracker_box["z_max"];
    visualization_msgs::Marker area_marker;
    area_marker.header.frame_id = "velodyne";
    area_marker.header.stamp = ros::Time::now();
    area_marker.id = 0;
    area_marker.type = park_marker.CUBE;
    area_marker.action = park_marker.ADD;
    area_marker.pose.position.x = (x_min + x_max) / 2;
    area_marker.pose.position.y = (y_min + y_max) / 2;
    area_marker.pose.position.z = (z_min + z_max) / 2;
    area_marker.pose.orientation.x = 0;
    area_marker.pose.orientation.y = 0;
    area_marker.pose.orientation.z = 0;
    area_marker.pose.orientation.w = 1;
    area_marker.scale.x = x_max - x_min;
    area_marker.scale.y = y_max - y_min;
    area_marker.scale.z = z_max - z_min;
    area_marker.color.a = 0.3;
    area_marker.color.r = 1;
    area_marker.color.g = 0;
    area_marker.color.b = 0;
    area_marker.lifetime = ros::Duration(0.5);
    m_area_publisher.publish(area_marker);
}

}  // namespace aesk_object_localization