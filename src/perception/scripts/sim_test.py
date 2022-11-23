import cv2
import numpy as np
import matplotlib.pyplot as plt
import time
import math
from scipy.ndimage import gaussian_filter1d
from scipy.signal import argrelextrema

def get_mid_point(rectangle_area):

    nonzerox = rectangle_area.nonzero()[1]
    mean = np.mean(nonzerox)

    if math.isnan(mean):
        return 0
    return np.int(mean)

def get_image_with_lines(img):

    thresx = 175

    binary_image = cv2.threshold(img, thresx, 255, cv2.THRESH_BINARY)[1]

    return binary_image

def preprocess_img(img):

    cut_image = img[:, 120:]
    resized_img = cv2.resize(cut_image, (1280, 720))

    gray_image = cv2.cvtColor(resized_img, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(gray_image, (3, 3), 0)

    return blurred_image


def get_lane_points(gray_image):

    top_cut = 450
    bottom_cut = 600

    cv2.line(gray_image, (0, bottom_cut), (gray_image.shape[1]-1, bottom_cut), 255, 3)
    cv2.line(gray_image, (0, top_cut), (gray_image.shape[1]-1, top_cut), 255, 3)

    # cv2.imshow('title', gray_image)
    # cv2.waitKey()

    img = gray_image[top_cut:bottom_cut, :]

    binary_image = get_image_with_lines(img)

    # img = cv2.threshold(img, thresx, 255, cv2.THRESH_BINARY)[1]
    img = cv2.cvtColor(binary_image, cv2.COLOR_GRAY2BGR)

    height, width = binary_image.shape

    step = height // 10

    top = height - step
    bottom = height
    bottom_img = binary_image[top:]
    bottom_hist = np.sum(bottom_img, axis=0)
    bottom_hist = bottom_hist // 255
    left_point = np.argmax(bottom_hist[:width//2])
    right_point = np.argmax(bottom_hist[width//2:]) + width // 2

    left_rectangle_left = np.clip(left_point - 30, 0, len(bottom_hist) // 2)
    left_rectangle_right = np.clip(left_point + 30, 0, len(bottom_hist) // 2)
    right_rectangle_left = np.clip(right_point - 30, len(bottom_hist) // 2, len(bottom_hist))
    right_rectangle_right = np.clip(right_point + 30, len(bottom_hist) // 2, len(bottom_hist))

    if right_rectangle_left == len(bottom_hist) // 2:
        right_rectangle_left = len(bottom_hist) - 30
    if right_rectangle_right == len(bottom_hist) // 2:
        right_rectangle_right = len(bottom_hist)

    mid_point_left = get_mid_point(binary_image[top:height, left_rectangle_left:left_rectangle_right]) +\
                     left_rectangle_left
    mid_point_right = get_mid_point(binary_image[top:height, right_rectangle_left:right_rectangle_right]) +\
                      right_rectangle_left

    left_lane_points = []
    right_lane_points = []

    left_lane_points.append((mid_point_left, (top+height) // 2 + top_cut))
    right_lane_points.append((mid_point_right, (top+height) // 2 + top_cut))

    cv2.rectangle(img, (left_point-30, height), (left_point+30, top), (45, 40, 255), 3)
    cv2.rectangle(img, (right_point-30, height), (right_point+30, top), (45, 40, 255), 3)

    while top > 0:
        top -= step
        bottom -= step
        img_part = binary_image[top:bottom]

        part_hist = np.sum(img_part, axis=0) // 255

        left_search_area_left = np.clip(left_point - 50, 0, len(part_hist) // 2)
        left_search_area_right = np.clip(left_point + 50, 0, len(part_hist) // 2)

        right_search_area_left = np.clip(right_point - 50, len(part_hist) // 2, len(part_hist))
        right_search_area_right = np.clip(right_point + 50, len(part_hist) // 2, len(part_hist))

        left_point = np.argmax(part_hist[left_search_area_left:left_search_area_right]) + left_search_area_left
        right_point = np.argmax(part_hist[right_search_area_left:right_search_area_right]) + right_search_area_left

        left_rectangle_left = np.clip(left_point - 30, 0, len(part_hist) // 2)
        left_rectangle_right = np.clip(left_point + 30, 0, len(part_hist) // 2)
        right_rectangle_left = np.clip(right_point - 30, len(part_hist) // 2, len(part_hist))
        right_rectangle_right = np.clip(right_point + 30, len(part_hist) // 2, len(part_hist))

        if right_rectangle_left == len(part_hist) // 2:
            right_rectangle_left = len(part_hist) - 30
        if right_rectangle_right == len(part_hist) // 2:
            right_rectangle_right = len(part_hist)

        # print '========================================================================================='
        mid_point_left = get_mid_point(binary_image[top:bottom, left_rectangle_left:left_rectangle_right]) +\
                         left_rectangle_left
        mid_point_right = get_mid_point(binary_image[top:bottom, right_rectangle_left:right_rectangle_right]) +\
                          right_rectangle_left

        left_lane_points.append((mid_point_left, (top + bottom) // 2 + top_cut))
        right_lane_points.append((mid_point_right, (top + bottom) // 2 + top_cut))

        left_rectangle_left = np.clip(left_point - 30, 0, len(part_hist) // 2)
        left_rectangle_right = np.clip(left_point + 30, 0, len(part_hist) // 2)
        right_rectangle_left = np.clip(right_point - 30, len(part_hist) // 2, len(part_hist))
        right_rectangle_right = np.clip(right_point + 30, len(part_hist) // 2, len(part_hist))

        cv2.rectangle(img, (left_rectangle_left, bottom), (left_rectangle_right, top), (45, 40, 255), 2)
        cv2.rectangle(img, (right_rectangle_left, bottom), (right_rectangle_right, top), (45, 40, 255), 2)

    # for i in range(len(left_lane_points)):
    #     cv2.circle(img, left_lane_points[i], 5, (255, 255, 0), 3)
    #     cv2.circle(img, right_lane_points[i], 5, (255, 0, 255), 3)

    # cv2.imshow("title", img)
    # cv2.waitKey()

    # resized_img[450:600, :] = img
    #print(left_lane_points)

    return left_lane_points, right_lane_points
