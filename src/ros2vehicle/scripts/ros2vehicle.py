#!/usr/bin/env python3

from pickle import FALSE
import numpy as np
import rospy
import serial
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import time


def data_sender(data):
    
    ser = serial.Serial(port='/dev/ttyACM0',baudrate=9600,stopbits=1,bytesize=8)#veriyi bastigimiz seri port
    
    velocity        = data.linear.x 
    steer           = data.angular.z 
    
    #print(steer)
    #print(velocity)
    #0-800 arasi
    
    

    # if steer > 1750:
    #         steer = 1750     
    # if steer < 0:
    #         steer = 0
    

    direction = steer # 0-810 arasi
    Set_Speed = velocity # m/s
    
    Set_Speed_uint8 = np.uint8(Set_Speed)
    direction_L = np.uint32(direction) & 0xFF
    direction_H = (np.uint32(direction) >> 8) & 0xFF

    cw = [1,2,Set_Speed_uint8,direction_L , direction_H]
    ser.write(serial.to_bytes(cw))
    
    ser.close()
    ser.open()


    #print("angle: ", direction, "    speed: ", velocity,)


def listener():

    rospy.init_node('ROS2CAN', anonymous=False)

    rospy.Subscriber("/vehicle_cmd", Twist, data_sender)
    
    rospy.spin()

if __name__ == '__main__':

	listener()