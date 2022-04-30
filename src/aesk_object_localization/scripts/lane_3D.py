#! /usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d

import ros_numpy
import actionlib

import sensor_msgs.point_cloud2 as pc2

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from aesk_object_localization.msg import ProjectedCloudAction, ProjectedCloudGoal

# TODO: THIS CODE MUST BE REWRITTEN IN C++ !!!

class lane3dNode():
    def __init__(self) -> None:
        rospy.Subscriber("/lane_image_pc2", PointCloud2,
                         self.lane_2d_callback, queue_size=1)

        self.lane_publisher = rospy.Publisher(
            "/lane_3d", PointCloud2, queue_size=1)

        self.action_client = actionlib.SimpleActionClient(
            "cloud_painter/cloud_projector", ProjectedCloudAction)

        rospy.loginfo("Waiting for projected point cloud action server.")
        self.action_client.wait_for_server()

        self.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                       PointField('y', 4, PointField.FLOAT32, 1),
                       PointField('z', 8, PointField.FLOAT32, 1),
                       PointField('pX', 12, PointField.FLOAT32, 1),
                       PointField('pY', 16, PointField.FLOAT32, 1)]

        rospy.loginfo("Initialized lane 3D node.")

    def get_pXpY_points(self, cloud_array, remove_nans=False, dtype=np.float):
        '''Pulls out x, y, and z columns from the cloud recordarray, and returns
        a 3xN matrix.
        '''
        # remove crap points
        # if remove_nans:
        #     mask = np.isfinite(cloud_array['pX']) & np.isfinite(cloud_array['pY'])
        #     cloud_array = cloud_array[mask]

        # pull out x, y, and z values
        points = np.zeros(cloud_array.shape + (2,), dtype=dtype)
        points[..., 0] = cloud_array['pX']
        points[..., 1] = cloud_array['pY']

        return points

    def get_xy_points(self, cloud_array, remove_nans=False, dtype=np.float):
        '''Pulls out x, y, and z columns from the cloud recordarray, and returns
        a 3xN matrix.
        '''
        # remove crap points
        # if remove_nans:
        #     mask = np.isfinite(cloud_array['pX']) & np.isfinite(cloud_array['pY'])
        #     cloud_array = cloud_array[mask]

        # pull out x, y, and z values
        points = np.zeros(cloud_array.shape + (2,), dtype=dtype)
        points[..., 0] = cloud_array['x']
        points[..., 1] = cloud_array['y']

        return points

    def lane_2d_callback(self, lane_2d: PointCloud2) -> None:
        rospy.loginfo("Arrived lane 2d PointCloud2.")

        # Get projected lidar point cloud.
        action_goal = ProjectedCloudGoal()
        self.action_client.send_goal(action_goal)
        self.action_client.wait_for_result()

        # Read and create a numpy array of incoming points.
        projected_cloud_numpy = ros_numpy.point_cloud2.pointcloud2_to_array(
            self.action_client.get_result().cloud)

        # Split xyz fields.
        cloud_xyz = ros_numpy.point_cloud2.get_xyz_points(projected_cloud_numpy)

        # Split pXpY fields.
        cloud_pXpY = self.get_pXpY_points(projected_cloud_numpy)
        augment = np.zeros((cloud_pXpY.shape[0], 1), dtype=np.int64)
        cloud_pXpY = np.append(cloud_pXpY, augment, axis=1)

        # Create Open3D cloud from numpy array.
        projected_cloud_open3d = o3d.geometry.PointCloud()

        # Since we're going to find neighbours for pX pY, we need to create KdTree with pixel values.
        # Therefore, our main points are cloud_pXpY.
        projected_cloud_open3d.points = o3d.utility.Vector3dVector(cloud_pXpY)

        # We also need to store 3D xyz values. We use colors field for that.
        projected_cloud_open3d.colors = o3d.utility.Vector3dVector(cloud_xyz)

        # Build KDTree. We use pX pY values.
        projected_cloud_tree = o3d.geometry.KDTreeFlann(projected_cloud_open3d)

        # Read 2D lane point cloud.
        lane_2d_cloud = ros_numpy.point_cloud2.pointcloud2_to_array(lane_2d)

        # Split xyz fields.
        lane_2d_cloud = self.get_xy_points(lane_2d_cloud)
        augment = np.zeros((lane_2d_cloud.shape[0], 1), dtype=np.int64)
        lane_2d_cloud = np.append(lane_2d_cloud, augment, axis=1)

        # Create an empty array to hold found 3D lane points.
        lane_3d_cloud = []

        # For each lane point, match them with the nearest 3D lidar point.
        for lane_index in range(0, lane_2d_cloud.shape[0], 1500):
            lane_point = lane_2d_cloud[lane_index]

            # Search for nearest neighbour.
            [k, idx, _] = projected_cloud_tree.search_knn_vector_3d(
                lane_point, 2)

            nearest_point = np.asarray(projected_cloud_open3d.colors)[idx[1:]]
            nearest_point_pixel = np.asarray(projected_cloud_open3d.points)[idx[1:]]
            points = []
            for i, j in zip(nearest_point, nearest_point_pixel):
                point = (i[0], i[1], i[2], j[0], j[1])
                points.append(point)
            rospy.loginfo(f"Nearest points: {nearest_point}")
            lane_3d_cloud.extend(points)

        lane_msg = pc2.create_cloud(
            Header(frame_id="velodyne"), self.fields, lane_3d_cloud)

        self.lane_publisher.publish(lane_msg)


def main():
    rospy.init_node("lane_3D", anonymous=False)
    node = lane3dNode()
    rospy.spin()


if __name__ == "__main__":
    main()
