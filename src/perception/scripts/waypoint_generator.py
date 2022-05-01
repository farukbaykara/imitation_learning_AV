# import rospy
#
# from sensor_msgs.msg import CompressedImage
# from perception.msg import waypoints as wp_message
from sim_test import *
from perception.msg import waypoints as wp_message
import numpy as np
import cv2


def imageInfoCallback(data, waypoint_pub):

    np_arr = np.fromstring(data.data, np.uint8)
    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    processed_img = preprocess_img(cv_image)
    left_lane_points, right_lane_points = get_lane_points(processed_img)


    for i in range(len(left_lane_points)):
        cv2.circle(processed_img, left_lane_points[i], 5, (255, 255, 0), 3)
        cv2.circle(processed_img, right_lane_points[i], 5, (255, 0, 255), 3)
    #cv2.imshow("image", processed_image)
    #cv2.waitKey(1)



    wp = wp_message()
    wp.waypoint_sol_x = left_lane_points[0][0]
    wp.waypoint_sag_y = right_lane_points[0][0]

    waypoint_pub.publish(wp)

    print(waypoint_sol_x)
    print(waypoint_sag_y)

    # cv2.imshow("image", processed_image)
    # cv2.waitKey(1)
