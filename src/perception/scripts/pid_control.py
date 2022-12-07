#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped



class Controller:

    def __init__(self):
        self.integrator = 0
        self.prev_error = 0
        self.steer_limMax = 3
        self.steer_limMin = -3
        self.integ_limMax = 5
        self.integ_limMin = -5

        self.vehicle_cmd_pub = rospy.Publisher("/vehicle_cmd", Twist, queue_size=10)
        self.wp_sub = rospy.Subscriber('/waypoint_topic', PoseStamped,
                        self.waypointCallback)


    def waypointCallback(self,data):
        #print("waypoitn callback girdi")

        ref_point = PoseStamped()

        ref_point = data
        # print(ref_point.pose.position.x)
        # print(ref_point.pose.position.y)

        kp = 0.1
        ki = 0.01
        kd = 0.0

        car_pos = ref_point.pose.position.x/1250
        act_point = 0.5

        error = car_pos-act_point

        #print(error)

        prop = kp*error

        self.integrator = self.integrator + ki*(error+self.prev_error)

        if(self.integrator>self.integ_limMax):
            self.integrator = self.integ_limMax
        elif(self.integrator<self.integ_limMin):
            self.integrator = self.integ_limMin


        diff = kd*(error - self.prev_error)

        steer = prop + diff + self.integrator

        self.prev_error = error

        if(steer>self.steer_limMax):
            steer = self.steer_limMax
        elif(steer<self.steer_limMin):
            steer = self.steer_limMin

        steer_msg = Twist()

        steer_msg.linear.x = 1
        steer_msg.angular.z = steer

        self.vehicle_cmd_pub.publish(steer_msg)

        # print(steer)

        # pos.x will be followed




if __name__ == '__main__':

    rospy.init_node('controller', anonymous=False)
    Controller()
    rospy.spin()

