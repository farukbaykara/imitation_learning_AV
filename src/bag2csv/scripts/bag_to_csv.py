#! /usr/bin/env python3

from io import StringIO
import string
from tokenize import String
import cv2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rospy
import time
import message_filters
from lgsvl_msgs.msg import CanBusData
import csv
import pandas as pd
from pathlib import Path
import os


class Main():
    def __init__(self):
        
        self.counter = 0
        self.cv_image = None
        self.raw_canbus = CanBusData()
        self.raw_canbus = None
        self.csv_name = 'driving_log.csv'
        self.csv_dir = '/home/aydin/data/'
        self.csv_full_path = self.csv_dir+self.csv_name
        self.bagfile_name = 'data2'
        self.camera_topic = '/zed/zed_node/left_raw/image_raw_color'
        self.CanBus_topic ='/joy_modified'
        self.add_or_new = 'new'
        self.image_path = '/home/aydin/data/IMG'
        self.bridge = CvBridge()


        


        if self.add_or_new=='new':
            column_data = pd.DataFrame(columns=['Image','Steering'])
            column_data.to_csv(self.csv_full_path, mode='a', index = False)

        self.raw_image_topic = message_filters.Subscriber(self.camera_topic, Image)


        self.raw_canbus_topic = message_filters.Subscriber(self.CanBus_topic,CanBusData)


        timeSynchronizer = message_filters.ApproximateTimeSynchronizer([self.raw_image_topic, self.raw_canbus_topic], queue_size=10, slop=0.1, allow_headerless=False)
        timeSynchronizer.registerCallback(self.callback)
        rospy.loginfo("I will publish to the topic %s", str('node basladi'))

    def callback(self, raw_image, raw_canbus):
        print("callback girdi")
        self.raw_image = raw_image
        self.raw_canbus = raw_canbus

        self.cv_image = self.bridge.imgmsg_to_cv2(raw_image, "bgr8")

        #self.cv_image = self.bridge.imgmsg_to_cv2(self.raw_image,desired_encoding='passthrough')
    
    def save_data(self):
        
        if self.cv_image is None or self.raw_canbus is None:
            return
        
        print('-------------------'+str(self.counter))
        image_path = self.image_path+self.bagfile_name+'_'+str(self.counter).zfill(7)+'.jpg'
        
        cv2.imwrite(image_path, img=self.cv_image)
        self.counter+=1

        
        
        data = pd.DataFrame([[image_path , self.raw_canbus.steer_pct]], columns=['Image','Steering'])
        #data['Steering'] = data['Steering'].astype(str)
        #Çalıştı, csv ye ek olarak ekleniyor ama resimler üzerine yazıyor. 
        data.to_csv(self.csv_full_path, mode='a', header = False,index = False) 



        
        self.raw_canbus = None
        self.cv_image = None

def main():

    rospy.init_node("bag2csv", anonymous=False)

    rospy.loginfo('PLEASE, PLAY SOME BAG FILE')

    main_o = Main()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        main_o.save_data()
        rate.sleep()

if __name__ == '__main__':

    main()