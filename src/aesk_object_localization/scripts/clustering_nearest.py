#! /usr/bin/env python

from collections import Counter

import rospy
from visualization_msgs.msg import Marker, MarkerArray

from aesk_object_localization.msg import PointCloud2WithId, ArrayOfPointCloud2s
import sensor_msgs.point_cloud2 as pc2

from sklearn.cluster import DBSCAN
import numpy as np
import matplotlib.pyplot as plt

from math import sqrt


def most_Common(lst):
    data = Counter(lst)
    return data.most_common(1)[0][0]


def ProjectedPointsCallback(clouds_input):
    marker_array = MarkerArray()

    for cloud_with_id in clouds_input.clouds:

        # print("Box id:", cloud_with_id.id)

        points_filtered = pc2.read_points_list(cloud_with_id.cloud, field_names=(
            "x", "y", "z"), skip_nans=True)

        # points = []
        # for point in cloud_with_id.projectedPoints:
        #     point3D = (point.lidarX, point.lidarY, point.lidarZ)
        #     points.append(point3D)

        try:
            clustering = DBSCAN(eps=1, min_samples=4).fit(points_filtered)
        except:
            return

        labels = clustering.labels_

        # print "Labels:"
        # print labels

        # n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)

        # print("Points length:", len(points_filtered))
        # print(clustering.labels_)
        # print("Number of clusters:", n_clusters_)

        unique_labels = set(labels)
        found_clusters = [[]] * len(unique_labels)

        print "\n\n****************************************"
        # print "Labels:"
        # print labels

        for index, label in enumerate(labels):
            if label == -1:
                continue
            # print "-------"
            # print "label:", int(label)
            # print "index:", index
            # print "points_filtered[index]:", points_filtered[index]
            # print "Before found_clusters[int(label)]:", found_clusters[int(label)]
            found_clusters[int(label)].append(points_filtered[index])
            # print "After found_clusters[int(label)]:", found_clusters[int(label)]

        print "Found clusters size:"
        print len(found_clusters)

        position_of_clusters = [None] * len(unique_labels)
        distance_of_clusters = [None] * len(unique_labels)

        for index, cluster in enumerate(found_clusters):
            sum_x = 0
            sum_y = 0
            sum_z = 0
            for point in cluster:
                sum_x += point[0]
                sum_y += point[1]
                sum_z += point[2]
            mean_x = sum_x / len(cluster)
            mean_y = sum_y / len(cluster)
            mean_z = sum_z / len(cluster)
            position_of_clusters[index] = (mean_x, mean_y, mean_z)
            distance_of_clusters[index] = sqrt(mean_x**2 + mean_y**2 + mean_z**2)

        print "Distances:"
        print distance_of_clusters
        print "Positions:"
        print position_of_clusters
        print "Min index:"
        print np.argmin(distance_of_clusters)

        # Create a marker
        marker = Marker()
        marker.header.frame_id = "velodyne"
        #marker.header.stamp = rospy.get_time()
        marker.id = cloud_with_id.id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = position_of_clusters[np.argmin(distance_of_clusters)][0]
        marker.pose.position.y = position_of_clusters[np.argmin(distance_of_clusters)][1]
        marker.pose.position.z = position_of_clusters[np.argmin(distance_of_clusters)][2]
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.5
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 1
        marker.color.b = 0
        marker.lifetime = rospy.Duration(0.5)
        marker_array.markers.append(marker)

        # print("------")
        
    helper.marker_pub.publish(marker_array)
    


class Helper():
    def __init__(self):
        self.marker_pub = rospy.Publisher(
            "/sign_markers_nearest", MarkerArray, queue_size=1)


helper = Helper()


def main():
    rospy.init_node("clustering_nearest", anonymous=False)

    rospy.Subscriber("/cloud_filter/filtered_clouds", ArrayOfPointCloud2s,
                     ProjectedPointsCallback, queue_size=1)

    rospy.spin()


if __name__ == "__main__":
    main()
