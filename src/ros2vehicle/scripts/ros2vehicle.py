#!/usr/bin/env python3

from pickle import FALSE
import numpy as np
import rospy
import serial
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import time
from lgsvl_msgs.msg import CanBusData


def data_sender(data):
    
    ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200,stopbits=1,bytesize=8)#veriyi bastigimiz seri port
    
    velocity        = data.linear.x 
    steer           = data.angular.z 
    
    #print(steer)
    #print(velocity)
    #0-800 arasi
    
    

    # if steer > 1750:
    #         steer = 1750     
    # if steer < 0:
    #         steer = 0
    

    set_steer = 100*steer # 0-810 arasi
    Set_Speed = velocity # m/s
    set_direction = 0
    Set_Speed_uint8 = np.uint8(Set_Speed)
    #direction_L = np.uint32(direction) & 0xFF
    #direction_H = (np.uint32(direction) >> 8) & 0xFF
    if(set_steer > 90):
        set_steer = 90
    elif(set_steer<-90):
        set_steer = -90



    if(set_steer > 0):
        set_direction = 1
        
    elif(set_steer<0):
        set_direction = 0
        set_steer = set_steer * -1


    print("speed:",str(Set_Speed_uint8))
    print("steering:",str(set_steer))
    print("direction:", str(set_direction))
    
    cw = [171,172,Set_Speed_uint8, np.uint8(set_steer), np.uint8(set_direction)]
    ser.write(serial.to_bytes(cw))
    

    #print("angle: ", direction, "    speed: ", velocity,)


def listener():

    rospy.init_node('ROS2CAN', anonymous=False)

    rospy.Subscriber("/vehicle_cmd", Twist, data_sender)

    rospy.Subscriber("/vehicleData",CanBusData,vehicleDataReceiver)
    
    rospy.spin()

if __name__ == '__main__':

	listener()



class Main():
    def __init__(self):
        

        self.raw_vehicle_data = CanBusData()
        self.raw_vehicle_cmd = Twist()
        self.autonomous_mode = 0

        self.ser = serial.Serial(port='/dev/ttyUSB0',baudrate=115200,stopbits=1,bytesize=8)#veriyi bastigimiz seri port

        self.vehicleData_sub = rospy.Subscriber("/vehicleData",CanBusData,self.callback_vehicledata)
        self.vehicleData_sub = rospy.Subscriber("/vehicle_cmd",Twist,self.callback_vehicleCmd)


        rospy.loginfo("I will publish to the topic %s", str('node basladi'))

    def callback_vehicledata(self, raw_vehicle_data):

        print("vehicle data callback girdi")
        self.raw_vehicle_data = raw_vehicle_data
        

    def callback_vehicleCmd(self,raw_vehicle_cmd):

        print("vehicle cmd callback girdi")

        self.raw_vehicle_cmd = raw_vehicle_cmd
    
    def send_to_vehicle(self):
        
        
        steer           = self.raw_vehicle_cmd.angular.z 
        autonomous_mode = self.raw_vehicle_data.engine_active
        #print(steer)
        #print(velocity)
        #0-800 arasi
        if(autonomous_mode):
            velocity = self.raw_vehicle_cmd.linear.x 
        else:
            velocity = 0
        

        # if steer > 1750:
        #         steer = 1750     
        # if steer < 0:
        #         steer = 0
        

        set_steer = 100*steer # 0-810 arasi
        Set_Speed = velocity # m/s
        set_direction = 0
        Set_Speed_uint8 = np.uint8(Set_Speed)
        #direction_L = np.uint32(direction) & 0xFF
        #direction_H = (np.uint32(direction) >> 8) & 0xFF
        if(set_steer > 90):
            set_steer = 90
        elif(set_steer<-90):
            set_steer = -90



        if(set_steer > 0):
            set_direction = 1
            
        elif(set_steer<0):
            set_direction = 0
            set_steer = set_steer * -1


        print("speed:",str(Set_Speed_uint8))
        print("steering:",str(set_steer))
        print("direction:", str(set_direction))
        print("autonomous mode",str(autonomous_mode))
        
        cw = [171,172,Set_Speed_uint8, np.uint8(set_steer), np.uint8(set_direction)]
        ser.write(serial.to_bytes(cw))

def main():

    rospy.init_node("ros2vehicle", anonymous=False)

    main_o = Main()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        
        main_o.send_to_vehicle()
        rate.sleep()

if __name__ == '__main__':

    main()