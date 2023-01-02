#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from lgsvl_msgs.msg import CanBusData



class Controller:

    def __init__(self):
        self.integrator = 0
        self.prev_error = 0
        self.steer_limMax = 3
        self.steer_limMin = -3
        self.integ_limMax = 5
        self.integ_limMin = -5
        self.set_speed = 0
        self.autonomous_mode = 0
        self.make_zero = 0
        self.kp = 0.2
        self.ki = 0.05
        self.kd = 0.0
        self.increase_kp = 0
        self.increase_ki = 0


        self.vehicle_cmd_pub = rospy.Publisher("/vehicle_cmd", Twist, queue_size=10)
        self.wp_sub = rospy.Subscriber('/waypoint_topic_2d', PoseStamped,
                        self.waypointCallback)
        self.joy_pub = rospy.Subscriber('/joy_cmd',CanBusData,self.joyCallback)

    def joyCallback(self,joyData):

        self.autonomous_mode = joyData.engine_active
        self.make_zero = joyData.high_beams_active
        self.increase_ki = joyData.left_turn_signal_active
        self.increase_kp = joyData.low_beams_active




    def waypointCallback(self,data):
        #print("waypoitn callback girdi")

        ref_point = PoseStamped()

        ref_point = data
        # print(ref_point.pose.position.x)
        # print(ref_point.pose.position.y)

        if(self.increase_kp):
            self.kp += 0.1
        if(self.increase_ki):
            self.ki += 0.1



        car_pos = ref_point.pose.position.x/1250
        act_point = 0.5

        error = car_pos-act_point

        if(self.make_zero):
            self.integrator = 0


        #print(error)
        if(self.autonomous_mode):
            prop = self.kp*error

            self.integrator = self.integrator + self.ki*(error+self.prev_error)

            if(self.integrator>self.integ_limMax):
                self.integrator = self.integ_limMax
            elif(self.integrator<self.integ_limMin):
                self.integrator = self.integ_limMin


            diff = self.kd*(error - self.prev_error)

            steer = prop + diff + self.integrator

            self.prev_error = error

            if(steer>self.steer_limMax):
                steer = self.steer_limMax
            elif(steer<self.steer_limMin):
                steer = self.steer_limMin

            steer_msg = Twist()

            self.set_speed = 1

            steer_msg.linear.x = self.set_speed
            steer_msg.angular.z = steer
            print(steer_msg)
            self.vehicle_cmd_pub.publish(steer_msg)

        # print(steer)

        # pos.x will be followed


    def __del__(self):
        self.set_speed = 0

if __name__ == '__main__':

    rospy.init_node('controller', anonymous=False)
    Controller()
    rospy.spin()


