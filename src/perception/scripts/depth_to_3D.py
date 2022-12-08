#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image




class DepthEstimator:

    def __init__(self):


        self.image_depth = Image() 

        self.vehicle_cmd_pub = rospy.Publisher("/vehicle_cmd", Twist, queue_size=10)

        self.depth_sub = rospy.Subscriber("/zed/zed_node/depth/depth_registered",Image,self.depthCallback)

        self.wp_sub = rospy.Subscriber('/waypoint_topic_2d', PoseStamped,
                        self.waypointCallback)


    def waypointCallback(self,msg_wp):
        #print("waypoitn callback girdi")

        #depths = self.image_depth.data[30]
        #print(depths)

        u = msg_wp.pose.position.x
        v = msg_wp.pose.position.y

        idx = round(u + self.image_depth.width*v)

        print(self.image_depth.data[idx])
        


    def depthCallback(self,msg_depth):
        #print("depth callback girdi")

        self.image_depth = msg_depth





if __name__ == '__main__':

    rospy.init_node('depth_estimator', anonymous=False)
    DepthEstimator()
    rospy.spin()


