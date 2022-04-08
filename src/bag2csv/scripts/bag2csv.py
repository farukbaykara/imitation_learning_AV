#! /usr/bin/env python3

from curses.ascii import CAN
from tkinter.messagebox import NO
import cv2

import rospy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import message_filters
import time
from sensor_msgs.msg import Image
from lgsvl_msgs.msg import CanBusData
import csv





class DataPicker():
    def init(self) -> None:
        
        self.counter = 0
        self.raw_canbus = CanBusData()
        self.raw_steering = CanBusData().steer_pct
        
        
        self.raw_canbus = None
        self.raw_image = None
        self.raw_steering = None
        self.cv_image = None
    
        


        self.image_topic = rospy.get_param('~image_topic')
        self.image_output_path = rospy.get_param('~image_output_path')
        self.canbus_topic = rospy.get_param('~canbus_topic')
        self.csv_output_path = rospy.get_param('~csv_output_path')
        self.csv_file_name = rospy.get_param('~csv_file_name')

        
        self.bridge = CvBridge()

        self.raw_image_topic = message_filters.Subscriber(self.image_topic, Image)


        self.raw_canbus_topic = message_filters.Subscriber(self.canbus_topic,CanBusData)


        timeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.raw_image_topic, self.raw_canbus_topic], queue_size=10, slop=0.1, allow_headerless=True)
        timeSynchronizer.registerCallback(self.callback)

    def callback(self, raw_image, raw_canbus):

        self.raw_image = raw_image
        self.raw_steering = raw_canbus

        print(self.raw_steering)
        
        self.cv_image = self.bridge.imgmsg_to_cv2(raw_image,desired_encoding="")


    def pick_data(self):

        if self.raw_image is None or self.raw_steering is None:
            print('Not picked----------------------')
            return

        if self.cv_image is None:
            return

        print('Data picked---------------------'+str(self.counter))
        
        
        image_path_name = self.image_output_path + 'data' + str(self.counter).zfill(7)+'.jpg'

        cv2.imwrite(image_path_name, img=self.cv_image)
        self.counter+=1



        data = [image_path_name,self.raw_steering]

        with open(self.csv_output_path+self.csv_file_name,'w',encoding='UTF8',newline='') as file:
            writer = csv.writer(file)
            writer.writerow(data)



        
        
        self.raw_image = None
        self.raw_steering = None
        self.cv_image = None


def main():

    rospy.init_node('data_picker', anonymous=False)

    data_picker = DataPicker()

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        data_picker.pick_data()
        rate.sleep()

if __name__=='main':

    main()