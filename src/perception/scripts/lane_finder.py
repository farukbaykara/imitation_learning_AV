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


def gaussian_blur(img, kernel_size=5):
    return cv2.GaussianBlur(img, (kernel_size, kernel_size), 0)


def grayscale_threshold(img, threshold=(0, 220)):
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    binary_img = np.ones_like(gray)
    binary_img[(gray > threshold[0]) & (gray < threshold[1])] = 0
    return binary_img

def crop(img):
    top_left = (500, 800)
    top_right = (1500, 800)
    bot_left = (400, 900)
    bot_right = (1600, 900)
    white = np.zeros_like(img)
    points = np.array([[bot_left, top_left, top_right, bot_right]], dtype=np.int32)
    cv2.fillPoly(white, points, 255)

    cropped_img = cv2.bitwise_and(img, white)
    return cropped_img

def find_lane_pixels(warped_img):

    histogram = np.sum(warped_img[warped_img.shape[0] // 2:, :], axis=0)
    out_img = np.dstack((warped_img, warped_img, warped_img)) * 255
    midpoint = np.int(histogram.shape[0] // 2)
    part_of_left = np.argmax(histogram[:midpoint])
    part_of_right = np.argmax(histogram[midpoint:]) + midpoint


    nwindows = 11
    margin = 150
    minpix = 50

    window_height = np.int(warped_img.shape[0] // nwindows)

    nonzero = warped_img.nonzero()

    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    leftx_current = part_of_left
    rightx_current = part_of_right

    left_lane_inds = []
    right_lane_inds = []  # 0 olmayan pikseller
    for window in range(nwindows):
        win_y_low = warped_img.shape[0] - (window + 1) * window_height
        win_y_high = warped_img.shape[0] - window * window_height
        win_xleft_low = leftx_current - margin  # dikdorgenin sol kenari
        win_xleft_high = leftx_current + margin  # dikdortgenin sag kenari
        win_xright_low = rightx_current - margin
        win_xright_high = rightx_current + margin
        cv2.rectangle(out_img, (win_xleft_low , win_y_low),  (win_xleft_high, win_y_high) , (0, 255 , 0), 2 )
        cv2.rectangle(out_img, (win_xright_low , win_y_low),  (win_xright_high, win_y_high) , (0, 255 , 0), 2 )

        good_left_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xleft_low) & (
                    nonzerox < win_xleft_high)).nonzero()[0]  # nonzerolardan dikdortgen icinde olanlari donduruyoruz
        good_right_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_xright_low) & (
                    nonzerox < win_xright_high)).nonzero()[0]
        left_lane_inds.append(good_left_inds)
        right_lane_inds.append(good_right_inds)
        if len(good_left_inds) > minpix:
            leftx_current = np.int(np.mean(nonzerox[good_left_inds]))
        if len(good_right_inds) > minpix:
            rightx_current = np.int(np.mean(nonzerox[good_right_inds]))
    try:
        left_lane_inds = (np.concatenate(left_lane_inds, axis=0))
        right_lane_inds = (np.concatenate(right_lane_inds, axis=0))
    except ValueError:
        pass

    leftx = nonzerox[left_lane_inds]
    lefty = nonzeroy[left_lane_inds]
    rightx = nonzerox[right_lane_inds]
    righty = nonzeroy[right_lane_inds]
    leftx = leftx.flatten()
    histogram_middle = int((leftx_current + rightx_current) // 2)
    middle_offset = (histogram_middle - midpoint)
    #print("MIDDDDDDDDDLEEEEEEEEEEEEEEEEEE")
#    #print(middle_offset)
#
    #print(leftx.shape)
    #print(rightx.shape)
    #	leftx = leftx.reshape(leftx.shape[0],1)
    #	print(leftx.shape)
    #	lefty = lefty.reshape(lefty.shape[0],1)
    #	rightx = rightx.reshape(rightx.shape[0],1)
    #
    #	righty = righty.reshape(righty.shape[0],1)

    return leftx, lefty, rightx, righty, out_img, middle_offset

def process_image(image):
    try:
        old_img = image
        img_blur = gaussian_blur(image)

        img_binary = grayscale_threshold(img_blur)

        img_cropped = crop(img_binary)

        #leftx, lefty, rightx, righty, out_img, middle_offset = line_finder(img_cropped)

        #img_warped, M = warp(img_cropped)


        leftx, lefty, rightx, righty, out_img, middle_offset = find_lane_pixels(img_cropped)

        x = np.mean(leftx) + 300
        y = np.mean(lefty) - 300


        print(np.mean(lefty))
        print(np.mean(leftx))


        if isnan(x):
            x = 850

        if isnan(y):
            y = 650




#
#        left_fitx, right_fitx, ploty, out_img, mid_fit= polynomial(leftx, lefty, rightx, righty, out_img)
#
#        result = search_poly(out_img, left_fitx, right_fitx, ploty)
#
#
#        left_curve_rad, right_curve_rad = measure_curvature_real(ploty, leftx, rightx, lefty, righty)
#
#        #dewarped, M, M_inv = transform_perspective_inverse(result) ##OLD
#        dewarped, M, M_inv = transform_perspective_inverse(out_img)
#
#
#        y = np.sum((old_img, dewarped), axis=0)
#
#        z = scipy.misc.toimage(y)
#        z= np.array(z)
#
#
#        # img_warped = transform_perspective(img_cropped, M)
#
#        # final, cur_l, cur_r, mid_offset = curve_finding(img_warped, img_undistort, M, M_inv)
#        cv2.putText(z, "Lane Curvature: " + str((left_curve_rad+right_curve_rad)/2.0) + " (m)", (100, 100),
#                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 255, 255))
#        cv2.putText(z, "Middle_Ofset: " + str(middle_offset) + " (pixel)", (100, 200),
#        cv2.FONT_HERSHEY_PLAIN, 3, (255, 255, 255))
#
#
#
    except Exception as e:
        print(e)
        return image



    return out_img, x, y


def imageInfoCallback(data,waypoint_pub):

    np_arr = np.fromstring(data.data, np.uint8)
#   bridge = CvBridge()
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    #img_file_path = '/home/alperen/AESK_ws/src/perception/test_images'+str(i)
    #cv2.imwrite(img_file_path +'.jpg', cv_image)
    #rospy.loginfo("Saved to: " + img_file_path)
    #cv_image = bridge.imgmsg_to_cv2(data.camera_data, desired_encoding="passthrough")
    processed_image , waypoints_x, waypoint_y = process_image(cv_image)

    wp = wp_message()
    wp.waypoint_sol_x = waypoints_x
    wp.waypoint_sol_y = waypoint_y

    waypoint_pub.publish(wp)

    cv2.imshow("image",processed_image)
    cv2.waitKey(1)


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
