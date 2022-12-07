#!/usr/bin/env python3

import rospy
import cv2

from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from perception.msg import waypoints as wp_message

from geometry_msgs.msg import PoseStamped

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


    # for i in range(len(left_lane_points)):
    #     print(left_lane_points.x)

    wp = wp_message()
    wp.waypoint_sol_x = left_lane_points[3][0]
    wp.waypoint_sol_y = left_lane_points[3][1]
    wp.waypoint_sag_x = right_lane_points[3][0]
    wp.waypoint_sag_y = right_lane_points[3][1]

    # print(wp.waypoint_sol_x)
    # print(wp.waypoint_sag_x)
    

    mid_path_x = (wp.waypoint_sol_x + wp.waypoint_sag_x)/2
    mid_path_y = (wp.waypoint_sol_y+wp.waypoint_sag_y)/2



    for i in range(len(left_lane_points)):
        # cv2.circle(processed_img, left_lane_points[i], 5, (255, 255, 0), 3)
        # cv2.circle(processed_img, right_lane_points[i], 5, (255, 0, 255), 3)
        cv2.circle(processed_img, (int(wp.waypoint_sol_x),int(wp.waypoint_sol_y)), 5, (255, 160, 255), 3)
        cv2.circle(processed_img, (int(wp.waypoint_sag_x),int(wp.waypoint_sag_y)), 5, (255, 160, 255), 3)
        
        cv2.circle(processed_img, (int(mid_path_x),int(mid_path_y)), 5, (255, 160, 255), 3)

        # mid_path_x = (left_lane_points[i][0] + right_lane_points[i][0])/2
        # mid_path_y = (left_lane_points[i][1] + right_lane_points[i][1])/2
        
        # mid_path_arr_x.append(mid_path_x)
        # mid_path_arr_y.append(mid_path_y)

        #cv2.circle(processed_img, (int(mid_path_x[i]),int(mid_path_y[i])), 5, (255, 160, 255), 3)

        #cv2.circle(processed_img, wp.waypoint_sol_y, 5, (255, 0, 255), 3)
        



    # print(mid_path_arr_x)
    # print(mid_path_arr_y)

    
    cv2.imshow("image", processed_img)
    cv2.waitKey(1)

    wp_final = wp_message()
    wp_final.waypoint_sol_x = mid_path_x
    wp_final.waypoint_sol_y = mid_path_y


    wp_pose = PoseStamped()
    wp_pose.pose.position.x = mid_path_x
    wp_pose.pose.position.y = mid_path_y




    waypoint_pub.publish(wp_pose)

    #print(wp.waypoint_sol_x)
    #print(wp.waypoint_sag_x)


def cameraInfoListener():

    rospy.init_node('image_subscriber', anonymous=False)

    rospy.loginfo('Waiting for topic %s to be published..','/zed/zed_node/left_raw/image_raw_color/compressed')
    rospy.wait_for_message('/zed/zed_node/left_raw/image_raw_color/compressed',CompressedImage)
    rospy.loginfo('%s topic is now available!','simulator/middle_camera/image/compressed')

    waypoint_publisher = rospy.Publisher('waypoint_topic', PoseStamped, queue_size=10 )


    rospy.Subscriber('/zed/zed_node/left_raw/image_raw_color/compressed', CompressedImage, imageInfoCallback, waypoint_publisher)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    cameraInfoListener()
