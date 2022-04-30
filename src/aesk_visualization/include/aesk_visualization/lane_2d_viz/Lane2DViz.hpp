#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace aesk_visualization
{
class Lane2DViz
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit Lane2DViz(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Lane 2D cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud);

    /*!
     * Image callback method.
     * @param t_image the received Image message.
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& t_image);

    /*!
     * Draw points method.
     */
    void drawPoints();

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! Drivable area topic name.
    std::string m_point_cloud_topic;

    //! Image topic name.
    std::string m_image_topic;

    //! Drivable area point cloud subscriber.
    ros::Subscriber m_point_cloud_sub;

    //! Image transport object for dealing with images easily.
    image_transport::ImageTransport m_image_transport;

    //! Image subscriber.
    image_transport::Subscriber m_image_sub;

    //! Stores lane cloud.
    sensor_msgs::PointCloud2 m_cloud;

    //! Stores the image converted to OpenCV mat from sensor_msgs/Image
    cv_bridge::CvImagePtr m_cv_ptr;

    //! Flag for receiving point cloud.
    bool m_is_cloud_received{};

    //! Flag for receiving image.
    bool m_is_image_received{};
};

}  // namespace aesk_visualization
