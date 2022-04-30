#! /usr/bin/env python3

import rospy

import cv2
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from darknet_ros_msgs.msg import BoundingBoxes, BoundingBox
from aesk_object_localization.msg import PointCloud2WithId, ArrayOfPointCloud2s

import sensor_msgs.point_cloud2 as pc2

# TODO: Subscribe to darknet_ros/BoundingBoxes
# Show sign name

# Later TODO: Show the 3D ego coordinates of signs.


def ImageCallback(image_input):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(image_input, desired_encoding="")

    for cloudBox in viz.boxArray.clouds:
        # Read PointCloud2 mesasge
        points_filtered = pc2.read_points_list(cloudBox.cloud, field_names=(
            "x", "y", "z", "pX", "pY"), skip_nans=True)

        for point in points_filtered:
            point_x = int(round(point.pX))
            point_y = int(round(point.pY))
            cv2.circle(cv_image, (point_x, point_y), 5, (0, 0, 255), -1)

    for boundingBox in viz.yoloBoxes.bounding_boxes:
        cv2.rectangle(cv_image, (boundingBox.xmin, boundingBox.ymin),
                      (boundingBox.xmax, boundingBox.ymax), (255, 0, 0), 1)

    # # Resize image
    # scale_percent = 40  # percent of original size
    # width = int(cv_image.shape[1] * scale_percent / 100)
    # height = int(cv_image.shape[0] * scale_percent / 100)
    # dim = (width, height)
    # resized = cv2.resize(cv_image, dim, interpolation=cv2.INTER_AREA)

    viz.boxArray.clouds = []
    viz.yoloBoxes.bounding_boxes = []

    cv2.imshow("Kamera", cv_image)
    cv2.resizeWindow("Kamera", 800, 600)
    cv2.waitKey(1)


def ProjectedPointsCallback(box_input):
    viz.boxArray = box_input
    # print(box_input)


def YoloCallback(yolo_input):
    viz.yoloBoxes = yolo_input


class Visualizer():
    def __init__(self):
        self.boxArray = ArrayOfPointCloud2s()
        self.yoloBoxes = BoundingBoxes()


viz = Visualizer()


if __name__ == "__main__":
    rospy.init_node("visualizer", anonymous=False)

    rospy.Subscriber('/camera_driver/image_undistorted', Image, ImageCallback, queue_size=1)
    rospy.Subscriber('/darknet_ros/bounding_boxes',
                     BoundingBoxes, YoloCallback)
    rospy.Subscriber('/cloud_filter/filtered_clouds', ArrayOfPointCloud2s,
                     ProjectedPointsCallback)

    rospy.spin()
