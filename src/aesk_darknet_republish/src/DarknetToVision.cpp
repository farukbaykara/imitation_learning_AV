#include "DarknetToVision.hpp"

namespace aesk_darknet_republish
{

DarknetToVision::DarknetToVision(ros::NodeHandle& t_node_handle) : m_node_handle(t_node_handle)
{
    if (!readParameters())
    {
        ROS_ERROR("Could not read parameters.");
        ros::requestShutdown();
    }

    ROS_INFO("YOLO topic: : %s", m_yolo_topic.c_str());

    m_yolo_subscriber =
        m_node_handle.subscribe(m_yolo_topic.c_str(), 1, &DarknetToVision::yoloTopicCallback, this);

    m_vision_publisher = m_node_handle.advertise<vision_msgs::Detection2DArray>("vision_boxes", 1);

    ROS_INFO("Successfully launched  DarknetToVision node.");
}

bool DarknetToVision::readParameters()
{
    if (!m_node_handle.getParam("yolo_topic", m_yolo_topic))
    {
        return false;
    }
    return true;
}

void DarknetToVision::yoloTopicCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& t_yolo_boxes)
{
    if (t_yolo_boxes->bounding_boxes.empty())
    {
        return;
    }

    vision_msgs::Detection2DArray vis_msg;
    vis_msg.header = t_yolo_boxes->image_header;

    for (const auto& box : t_yolo_boxes->bounding_boxes)
    {
        vision_msgs::Detection2D detection;
        vision_msgs::ObjectHypothesisWithPose pose;

        detection.bbox.size_x = static_cast<double>(box.xmax - box.xmin);
        detection.bbox.size_y = static_cast<double>(box.ymax - box.ymin);
        detection.bbox.center.x = static_cast<double>(box.xmax + box.xmin) / 2;
        detection.bbox.center.y = static_cast<double>(box.ymax + box.ymin) / 2;
        pose.id = box.id;
        pose.score = box.probability;

        detection.results.push_back(pose);
        vis_msg.detections.push_back(detection);
    }

    m_vision_publisher.publish(vis_msg);
}

}  // namespace aesk_darknet_republish