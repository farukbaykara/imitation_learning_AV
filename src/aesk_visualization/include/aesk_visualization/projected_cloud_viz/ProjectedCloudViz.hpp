#pragma once

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <vision_msgs/Detection2DArray.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

namespace aesk_visualization
{
class ProjectedCloudViz
{
  public:
    /*!
     * Constructor.
     * @param t_node_handle the ROS node handle.
     */
    explicit ProjectedCloudViz(ros::NodeHandle& t_node_handle);

  private:
    /*!
     * Reads and verifies the ROS parameters.
     * @return true if successful.
     */
    bool readParameters();

    /*!
     * Projected cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud);

    /*!
     * Lane 2D cloud callback method.
     * @param t_point_cloud the received PointCloud2 message.
     */
    void laneCloudCallback(const sensor_msgs::PointCloud2ConstPtr& t_point_cloud);

    /*!
     * Image callback method.
     * @param t_image the received Image message.
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& t_image);

    /*!
     * Image callback method.
     * @param t_image the received Image message.
     */
    void detectionsCallback(const vision_msgs::Detection2DArrayConstPtr & t_detections);

    /*!
     * Draw points method.
     */
    void drawProjectedPoints(const sensor_msgs::PointCloud2& t_cloud, const cv::Scalar& t_color,
                             const int& t_thickness);

    /*!
     * Draw bounding boxes method.
     */
    void drawBoundingBoxes(const vision_msgs::Detection2DArray & t_detections, const cv::Scalar& t_color,
                           const int& t_thickness);

    //! ROS node handle.
    ros::NodeHandle& m_node_handle;

    //! Projected cloud topic name.
    std::string m_point_cloud_topic;

    //! Image topic name.
    std::string m_image_topic;

    //! Drivable area topic name.
    std::string m_lane_cloud_topic;

    //! Bounding boxes topic name.
    std::string m_bboxes_topic;

    //! Projected point cloud subscriber.
    ros::Subscriber m_point_cloud_sub;

    //! Drivable area point cloud subscriber.
    ros::Subscriber m_lane_cloud_sub;

    //! Bounding boxes subscriber.
    ros::Subscriber m_bboxes_sub;

    //! Image transport object for dealing with images easily.
    image_transport::ImageTransport m_image_transport;

    //! Image subscriber.
    image_transport::Subscriber m_image_sub;

    //! Stores projected cloud.
    sensor_msgs::PointCloud2 m_cloud;

    //! Stores lane cloud.
    sensor_msgs::PointCloud2 m_lane;

    //! Stores the image converted to OpenCV mat from sensor_msgs/Image
    cv_bridge::CvImagePtr m_cv_ptr;

    //! Stores 2D detections.
    vision_msgs::Detection2DArray m_detections;

    //! Flag for receiving projected point cloud.
    bool m_is_cloud_received;

    //! Flag for receiving lane point cloud.
    bool m_is_lane_received;

    //! Flag for receiving image.
    bool m_is_image_received;

    //! Flag for receiving YOLO bounding boxes.
    bool m_is_bboxes_received;
};

}  // namespace aesk_visualization