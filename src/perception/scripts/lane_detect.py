#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Float64, String
from std_srvs.srv import Empty, EmptyResponse
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
import matplotlib.image
import matplotlib.pyplot
import math
import os
from numpy import arctan2





def callback(data):
    #hello_str = " Callback called %s" % rospy.get_time()
    #rospy.loginfo(hello_str)
    
    
    bridge=CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    
    #encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 60]
    #result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
    
    
    gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    
    
    lower_white = np.array([0,0,0], dtype=np.uint8)
    upper_white = np.array([0,0,255], dtype=np.uint8)

    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(cv_image, lower_white, upper_white)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(cv_image,cv_image, mask= mask)

    mask_white = cv2.inRange(gray_image, 140, 255)
    #cv2.imshow('image',mask_white)
    #cv2.waitKey(1)
    

    kernel_size = np.ones((5,5),np.float32)/5
    gauss_gray = cv2.filter2D(mask_white,-1,kernel_size)

    low_threshold = 50
    high_threshold = 150
    canny_edges = cv2.Canny(gauss_gray,low_threshold,high_threshold)
    #cv2.imshow("canny",canny_edges)
    
    bot_left=[223,112]
    bot_right=[395,112]
    apex_right=[620,338]
    apex_left=[0,338]

    v= [np.array([bot_left,bot_right,apex_right,apex_left],dtype=np.int32)]
    mask_zero=np.zeros_like(canny_edges)
    

    if len(canny_edges.shape) >2:
        channel_count=canny_edges.shape[2]
        ignore_mask_color=(255,)*channel_count
    else:
        ignore_mask_color=255
    
    cv2.fillPoly(mask_zero,v,ignore_mask_color)
    ROI=cv2.bitwise_and(canny_edges,mask_zero)
    
    mask_zero2=np.zeros_like(cv_image)
    if len(cv_image.shape) >2:
        channel_count2=cv_image.shape[2]
        ignore_mask_color2=(255,)*channel_count2
    else:
        ignore_mask_color2=255
    
    cv2.fillPoly(mask_zero2,v,ignore_mask_color2)
    ROI_complete=cv2.bitwise_and(cv_image,mask_zero2)
    
    #cv2.imshow("Region ", ROI)
    

    rho = 0.8
    theta = np.pi/180
    #threshold is minimum number of intersections in a grid for candidate lin$
    threshold = 35
    min_line_len = 50
    max_line_gap = 200

    global i,total_sl,x_1,x_2,y_1,y_2
    i=0
    total_sl=0

    lines = cv2.HoughLinesP(ROI, rho, theta, threshold, np.array([]))
    line_img = np.zeros((ROI.shape[0], ROI.shape[1], 3), dtype=np.uint8)
    x_1=0
    y_1=0
    x_2=0
    y_2=0
    #cv2.imshow("line", line_img)
    for line in lines:
        for x1,y1,x2,y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), [255, 0, 0], 2)
            sl=float(y2-y1)/float(x2-x1)

            #cv2.line(line_img,(0,0),((x_2-x_1)/2),((y_2-y_1)/2),(255,0,0),5)
            i=i+1
            #print sl
            total_sl=total_sl+sl



    #cv2.draw_lines(line_img, lines)
    theta=math.degrees(math.atan(total_sl/i))
    rospy.loginfo(theta)
    slope_pub.publish(theta)
    
    
    
    complete = cv2.addWeighted(ROI_complete, 0.8, line_img, 1, 0)
    cv2.imshow('image',complete)
    cv2.waitKey(1)
    
    rate.sleep()


def listener():
    
    rospy.init_node('slope_calculator', anonymous=True)
    rate=rospy.Rate(1)
    image_sub = rospy.Subscriber("/camera/comressed" ,Image, callback)
    slope_pub = rospy.Publisher('slope_topic', Float64, queue_size=1)
    
   
    #image_pub= rospy.Publisher('opencv',Image,)
    
    hello_str = "Slope Calculation Started %s" % rospy.get_time()
    rospy.loginfo(hello_str)
    rospy.spin()
    
if __name__ == '__main__':
    rospy.init_node('slope_calculator', anonymous=True)
    rate=rospy.Rate(1)
    slope_pub = rospy.Publisher('slope_topic', Float64, queue_size=1)
    
    listener()