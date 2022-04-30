#!/usr/bin/env python3
import dynamic_reconfigure.encoding
import rospy

from dynamic_reconfigure.server import Server
from aesk_object_localization.cfg import extrinsicsConfig
import tf_conversions

import tf2_ros
import geometry_msgs.msg
import turtlesim.msg


class ExtrinsicsPublihser:
    def __init__(self):
        srv = Server(extrinsicsConfig, self.callback)
        rospy.loginfo("Started node.")

        rate = rospy.Rate(100)
        while (not rospy.is_shutdown()):
            self.handleTf(self.config)
            rate.sleep()

    def handleTf(self, config: dynamic_reconfigure.encoding.Config):
        br = tf2_ros.StaticTransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "velodyne"
        t.child_frame_id = "camera_right"
        t.transform.translation.x = config["x"]
        t.transform.translation.y = config["y"]
        t.transform.translation.z = config["z"]
        q = tf_conversions.transformations.quaternion_from_euler(config["roll"], config["pitch"], config["yaw"])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        br.sendTransform(t)

    def callback(self, config: dynamic_reconfigure.encoding.Config, level):
        print("---")
        rospy.loginfo("x: {x}".format(**config))
        rospy.loginfo("y: {y}".format(**config))
        rospy.loginfo("z: {z}".format(**config))
        rospy.loginfo("roll: {roll}".format(**config))
        rospy.loginfo("pitch: {pitch}".format(**config))
        rospy.loginfo("yaw: {yaw}".format(**config))

        self.config = config

        return config


def main():
    rospy.init_node("dynamic_extrinsics", anonymous=False)
    pub = ExtrinsicsPublihser()


if __name__ == "__main__":
    main()
