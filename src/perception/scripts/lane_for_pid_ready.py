#!/usr/bin/env python

import rospy
import cv2

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from perception.msg import waypoints as wp_message

import pickle, pprint
import os
import matplotlib.pyplot as plt
import numpy as np
import cv2
import glob
import scipy.misc
import roslib
from scipy.ndimage import filters
from math import *
from sim_test import *



def imageInfoCallback(data,waypoint_pub):

    np_arr = np.fromstring(data.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    processed_img = preprocess_img(cv_image)
    left_lane_points, right_lane_points = get_lane_points(processed_img)


    for i in range(len(left_lane_points)):
        cv2.circle(processed_img, left_lane_points[i], 5, (255, 255, 0), 3)
        cv2.circle(processed_img, right_lane_points[i], 5, (255, 0, 255), 3)
    cv2.imshow("image", processed_img)
    cv2.waitKey(1)



    wp = wp_message()
    wp.waypoint_sol_x = left_lane_points[0][0]
    wp.waypoint_sag_x = right_lane_points[0][0]

    waypoint_pub.publish(wp)

    print(wp.waypoint_sol_x)
    print(wp.waypoint_sag_x)


def cameraInfoListener():

    rospy.init_node('image_subscriber', anonymous=False)

    rospy.loginfo('Waiting for topic %s to be published..','simulator/middle_camera/image/compressed')
    rospy.wait_for_message('/camera/compressed',CompressedImage)
    rospy.loginfo('%s topic is now available!','simulator/middle_camera/image/compressed')

    waypoint_publisher = rospy.Publisher('waypoint_topic', wp_message, queue_size=10 )


    rospy.Subscriber('/camera/compressed', CompressedImage, imageInfoCallback, waypoint_publisher)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    cameraInfoListener()
