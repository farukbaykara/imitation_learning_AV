#! /usr/bin/env python3

import rospy
import actionlib
from collections import Counter
from sklearn.cluster import DBSCAN
import sensor_msgs.point_cloud2 as pc2
from aesk_object_localization.msg import ParkAction, ParkResult, ParkGoal


def most_Common(lst):
    data = Counter(lst)
    return data.most_common(1)[0][0]


def action_callback(goal):
    rospy.loginfo("Received park action.")

    points_filtered = pc2.read_points_list(
        goal.cloud, field_names=("x", "y", "z"), skip_nans=True)

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

    result_msg = ParkResult()
    result_msg.point.x = mean_x
    result_msg.point.y = mean_y
    result_msg.point.z = mean_z

    helper.action_server.set_succeeded(result_msg)   


class Helper():
    def __init__(self):
        self.action_server = actionlib.SimpleActionServer(
            "park_cluster", ParkAction, execute_cb=action_callback, auto_start=False)


helper = Helper()


def main():
    rospy.init_node("clustering_park", anonymous=False)
    helper.action_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
