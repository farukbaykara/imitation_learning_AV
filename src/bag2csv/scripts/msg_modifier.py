#! /usr/bin/env python3


from numpy import int64
import message_filters
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import Image
import yaml
import rosparam
import os 
from lgsvl_msgs.msg import CanBusData


class CloudModifier:
    def __init__(self):


        self.lidar1_topic = '/zed/zed_node/left_raw/image_raw_color'
        self.lidar2_topic = '/joy_cmd'
        self.published_topic = '/joy_modified'

        self.sub_lidar_2 = rospy.Subscriber(self.lidar2_topic,CanBusData,self.lidar2Callback)
        self.sub_lidar_1 = rospy.Subscriber(self.lidar1_topic,Image,self.lidar1Callback)

        self.joy_modified = CanBusData()

        self.pub_lidar = rospy.Publisher(self.published_topic,CanBusData,queue_size=10)



        rospy.loginfo("Msg Modifier Constructed")

        self.timestamp_secs = 0
        self.timestamp_nsecs = 0

    
    def lidar2Callback(self,msg_lidar):
        self.joy_modified = msg_lidar

        
        self.joy_modified.header.stamp.secs = self.timestamp_secs


        self.pub_lidar.publish(self.joy_modified)


    def lidar1Callback(self,msg_lidar):

        self.timestamp_secs = msg_lidar.header.stamp.secs


    
if __name__ == '__main__':
    rospy.init_node('msg_modifier')
    CloudModifier()
    rospy.spin()
