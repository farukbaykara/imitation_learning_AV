#! /usr/bin/env python3

import rospy
import actionlib
from collections import Counter
from sklearn.cluster import DBSCAN
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker, MarkerArray
from aesk_object_localization.msg import ParkAction, ParkResult, ParkGoal
from aesk_object_localization.msg import SignTrackerAction, SignTrackerResult, SignTrackerGoal, PointCloud2WithId


def most_Common(lst):
    data = Counter(lst)
    return data.most_common(1)[0][0]


def action_callback(goal):
    # rospy.loginfo("Received park action.")

    result_msg = SignTrackerResult()
    for cloud_with_id in goal.clouds:
        points_filtered = pc2.read_points_list(
            cloud_with_id.cloud, field_names=("x", "y", "z"), skip_nans=True)

        try:
            clustering = DBSCAN(eps=1, min_samples=4).fit(points_filtered)
        except:
            helper.action_server.set_aborted()
            return

        labels = clustering.labels_

        biggest_cluster_label = most_Common(labels)

        biggest_cluster = []
        for index, label in enumerate(labels):
            if label == biggest_cluster_label:
                biggest_cluster.append(points_filtered[index])

        sum_x = 0
        sum_y = 0
        sum_z = 0
        for point in biggest_cluster:
            sum_x += point[0]
            sum_y += point[1]
            sum_z += point[2]
        mean_x = sum_x / len(biggest_cluster)
        mean_y = sum_y / len(biggest_cluster)
        mean_z = sum_z / len(biggest_cluster)

        # print "x: ", mean_x
        # print "y: ", mean_y
        # print "z: ", mean_z

        # Create a marker
        marker = Marker()
        marker.header.frame_id = "velodyne"
        #marker.header.stamp = rospy.get_time()
        marker.id = cloud_with_id.id
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = mean_x
        marker.pose.position.y = mean_y
        marker.pose.position.z = mean_z
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.5
        marker.scale.y = 0.8
        marker.scale.z = 0.8
        marker.color.a = 1
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.lifetime = rospy.Duration(0.5)
        result_msg.markers.markers.append(marker)

    helper.action_server.set_succeeded(result_msg)


class Helper():
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            "sign_cluster", SignTrackerAction, execute_cb=action_callback, auto_start=False)


helper = Helper()


def main():
    rospy.init_node("clustering_park", anonymous=False)
    helper.action_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
