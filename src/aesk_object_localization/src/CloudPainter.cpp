#include "aesk_object_localization/CloudPainter.hpp"

namespace aesk_object_localization
{
CloudPainter::CloudPainter(ros::NodeHandle& t_node_handle_cloud)
  : m_node_handle(t_node_handle_cloud)
  , m_action_server(m_node_handle, "cloud_projector",
                    boost::bind(&CloudPainter::actionServerCallback, this, _1), false)
  , tfListener{ tfBuffer }
  , m_mat_point_transformer{ Eigen::Matrix4d::Identity() }
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("Lidar topic: : %s", m_lidar_topic.c_str());
    ROS_INFO("Camera topic: : %s", m_camera_topic.c_str());

    double intrinsic_params[9];
    for (int i = 0; i < 9; i++)
    {
        intrinsic_params[i] = m_intrinsic_parameters[i];
    }
    cv::Mat(3, 3, CV_64F, &intrinsic_params).copyTo(m_intrinsic_matrix);

    ROS_INFO("Intrinsic matrix:");
    std::cout << m_intrinsic_matrix << '\n';

    ROS_INFO("Region of interest:");
    for (const auto& roi : m_region_of_interest)
    {
        std::cout << roi.first << ": " << roi.second << '\n';
    }

    ROS_INFO("Image width: %d", m_image_width);
    ROS_INFO("Image height: %d", m_image_height);

    createMatrix();

    // std::cout << "trans_velo_to_cam:\n" << trans_velo_to_cam << '\n';

    m_point_cloud_pub = m_node_handle.advertise<sensor_msgs::PointCloud2>("cloud_projected", 1);

    m_point_cloud_sub = m_node_handle.subscribe<sensor_msgs::PointCloud2>(
        m_lidar_topic, 1, &CloudPainter::topicCallback, this);

    m_finish_sub = m_node_handle.subscribe<std_msgs::Bool>(m_finish_topic, 1,
                                                           &CloudPainter::finishCallback, this);

    m_action_server.start();

    ROS_INFO("Successfully launched node.");

    ros::spin();
}

bool CloudPainter::readParameters()
{
    if (!m_node_handle.getParam("lidar_topic", m_lidar_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("camera_topic", m_camera_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("finish", m_finish_topic))
    {
        return false;
    }
    if (!m_node_handle.getParam("intrinsic_parameters", m_intrinsic_parameters))
    {
        return false;
    }
    if (!m_node_handle.getParam("roi", m_region_of_interest))
    {
        return false;
    }
    if (!m_node_handle.getParam("roi_finish", m_region_of_interest_finish))
    {
        return false;
    }
    if (!m_node_handle.getParam("image_width", m_image_width))
    {
        return false;
    }
    if (!m_node_handle.getParam("image_height", m_image_height))
    {
        return false;
    }
    if (!m_node_handle.getParam("camera_frame_id", m_camera_frame_id))
    {
        return false;
    }
    return true;
}

bool CloudPainter::waitForCameraLidarTransform(geometry_msgs::TransformStamped& trans_velo_to_cam)
{
    while (true)
    {
        try
        {
            trans_velo_to_cam =
                tfBuffer.lookupTransform(m_camera_frame_id, "velodyne", ros::Time::now());
            if (trans_velo_to_cam.transform.translation.x != 0)
            {
                return true;
            }
        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
        }
    }
}

void CloudPainter::createMatrix()
{
    // Create intrinsic, extrinsic and projection matrix's.
    Eigen::Matrix4d mat_velo_to_cam;

    geometry_msgs::TransformStamped trans_velo_to_cam;

    //    ROS_INFO("Launching tf.");
    while (!waitForCameraLidarTransform(trans_velo_to_cam))
    {
        //        std::cout << "Waiting for transform.";
    }

    Eigen::Affine3d affine_velo_to_cam_ = tf2::transformToEigen(trans_velo_to_cam);

    mat_velo_to_cam = affine_velo_to_cam_.matrix();

    Eigen::MatrixXd mat_cam_intrinsic_3x3(3, 3);

    mat_cam_intrinsic_3x3 << m_intrinsic_matrix.at<double>(0), m_intrinsic_matrix.at<double>(1),
        m_intrinsic_matrix.at<double>(2), m_intrinsic_matrix.at<double>(3),
        m_intrinsic_matrix.at<double>(4), m_intrinsic_matrix.at<double>(5),
        m_intrinsic_matrix.at<double>(6), m_intrinsic_matrix.at<double>(7),
        m_intrinsic_matrix.at<double>(8);

    Eigen::MatrixXd mat_projector_3x4 = mat_cam_intrinsic_3x3 * mat_velo_to_cam.topLeftCorner(3, 4);

    m_mat_point_transformer.topLeftCorner(3, 4) = mat_projector_3x4;
}

void CloudPainter::finishCallback(const std_msgs::BoolConstPtr& t_is_finish)
{
    m_is_finish = t_is_finish->data;
}

void CloudPainter::actionServerCallback(
    const aesk_object_localization::ProjectedCloudGoalConstPtr& goal)
{
    ROS_INFO("Goal arrived.");
    // m_action_server.publishFeedback(m_feedback);
    m_action_server.setSucceeded(m_result);
}

void CloudPainter::topicCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud)
{
    createMatrix();
    sensor_msgs::PointCloud2 p_point_cloud;

    sensor_msgs::PointCloud2Modifier modifier(p_point_cloud);
    modifier.resize(t_point_cloud->height * t_point_cloud->width);

    modifier.setPointCloud2Fields(
        5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z",
        1, sensor_msgs::PointField::FLOAT32, "pX", 1, sensor_msgs::PointField::FLOAT32, "pY", 1,
        sensor_msgs::PointField::FLOAT32);

    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_x(p_point_cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_y(p_point_cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_z(p_point_cloud, "z");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_pX(p_point_cloud, "pX");
    sensor_msgs::PointCloud2Iterator<float> p_point_cloud_pY(p_point_cloud, "pY");

    sensor_msgs::PointCloud2ConstIterator<float> t_point_cloud_x(*t_point_cloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> t_point_cloud_y(*t_point_cloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> t_point_cloud_z(*t_point_cloud, "z");

    for (size_t i = 0; i < t_point_cloud->height * t_point_cloud->width; ++i, ++t_point_cloud_x,
                ++t_point_cloud_y, ++t_point_cloud_z, ++p_point_cloud_x, ++p_point_cloud_y,
                ++p_point_cloud_z, ++p_point_cloud_pX, ++p_point_cloud_pY)
    {
        if (!(m_region_of_interest["x_min"] < *t_point_cloud_x &&
              *t_point_cloud_x < m_region_of_interest["x_max"] &&

              m_region_of_interest["y_min"] < *t_point_cloud_y &&
              *t_point_cloud_y < m_region_of_interest["y_max"] &&

              m_region_of_interest["z_min"] < *t_point_cloud_z &&
              *t_point_cloud_z < m_region_of_interest["z_max"]) &&
            !m_is_finish)
        {
            continue;
        }

        if (!(m_region_of_interest_finish["x_min"] < *t_point_cloud_x &&
              *t_point_cloud_x < m_region_of_interest_finish["x_max"] &&

              m_region_of_interest_finish["y_min"] < *t_point_cloud_y &&
              *t_point_cloud_y < m_region_of_interest_finish["y_max"] &&

              m_region_of_interest_finish["z_min"] < *t_point_cloud_z &&
              *t_point_cloud_z < m_region_of_interest_finish["z_max"]) &&
            m_is_finish)
        {
            continue;
        }

        cv::Point point_in_image;

        Eigen::Vector4d pos;
        pos << *t_point_cloud_x, *t_point_cloud_y, *t_point_cloud_z, 1;

        Eigen::Vector4d vec_image_plane_coords = m_mat_point_transformer * pos;

        if (vec_image_plane_coords(2) <= 0)
            continue;

        point_in_image.x = (int)(vec_image_plane_coords(0) / vec_image_plane_coords(2));
        point_in_image.y = (int)(vec_image_plane_coords(1) / vec_image_plane_coords(2));
        //        ROS_INFO("Point %f, %f, %f, %u, %u", *t_point_cloud_x, *t_point_cloud_y,
        //        *t_point_cloud_z,
        //                 point_in_image.x, point_in_image.y);

        if (point_in_image.x < 0 || point_in_image.y < 0 || point_in_image.x >= m_image_width ||
            point_in_image.y >= m_image_height)
        {
            continue;
        }

        *p_point_cloud_x = *t_point_cloud_x;
        *p_point_cloud_y = *t_point_cloud_y;
        *p_point_cloud_z = *t_point_cloud_z;
        *p_point_cloud_pX = point_in_image.x;
        *p_point_cloud_pY = point_in_image.y;
    }

    std_msgs::Header header;
    header.frame_id = "velodyne";
    p_point_cloud.header = header;
    m_point_cloud_pub.publish(p_point_cloud);

    m_result.cloud = p_point_cloud;
}

}  // namespace aesk_object_localization