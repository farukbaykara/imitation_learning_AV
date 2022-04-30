#! /usr/bin/env python3

import rospy
import actionlib
import numpy as np
from Bezier import Bezier
from aesk_swerve.msg import BezierResult, BezierAction
from geometry_msgs.msg import Point


def action_callback(goal):
    rospy.loginfo("Received bezier action.")
    ##Get Bezier control points from goal

    temp_control_points = list()

    for point in goal.control_points:
        temp_control_points.append([point.x,point.y])

    bezier_points = np.array(temp_control_points)
    

    ##Call the Bezier algorithm with the array of points 
    bezier = Bezier()
    bezier_t_points = np.arange(0, 1, 0.1)
    waypoints = bezier.Curve( bezier_t_points,  bezier_points)

    ##Create the result message
    result_msg = BezierResult()

    ##Fill the result message array with the bezier curve points
    temp_point_list = list()
    for i in range(0,len(waypoints)):
        wp = waypoints[i]
        temp_point = Point()
        temp_point.x = wp[0]
        temp_point.y = wp[1]
        temp_point_list.append(temp_point)

    result_msg.path = np.array(temp_point_list)

    ##Send the result message
    action_server.set_succeeded(result_msg)   




if __name__ == "__main__":
    rospy.init_node("bezier_action_server", anonymous=False)
    action_server = actionlib.SimpleActionServer("bezier_action_server", BezierAction, execute_cb=action_callback, auto_start=False)
    action_server.start()
    rospy.loginfo("Sucesfully launched action server.")
    rospy.spin()