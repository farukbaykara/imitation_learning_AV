#! /usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from lgsvl_msgs.msg import CanBusData

# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim or any other robot

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def callback(data):
    twist = Twist()

    joy_cmd = CanBusData()

    joy_cmd.engine_active = data.buttons[0]
    joy_cmd.high_beams_active = data.buttons[1]
    joy_cmd.steer_pct = -data.axes[3]
    joy_cmd.engine_rpm = data.axes[4]
    joy_cmd.left_turn_signal_active = data.buttons[3]
    joy_cmd.low_beams_active = data.buttons[2]
    #print(data.buttons[0])
    
    twist.linear.x = 4*data.axes[7]
    twist.angular.z = 4*data.axes[6]
    #print(twist)
    pub.publish(twist)
    pub_joy.publish(joy_cmd)

# Intializes everything
def start():
    # publishing to "turtle1/cmd_vel" to control turtle1
    global pub
    global pub_joy
    pub = rospy.Publisher('/twist_joy', Twist)
    pub_joy = rospy.Publisher('/joy_cmd',CanBusData)
    # subscribed to joystick inputs on topic "joy"
    rospy.Subscriber("joy", Joy, callback)
    # starts the node
    rospy.init_node('Joy2Turtle')
    rospy.spin()

if __name__ == '__main__':
    start()



# global a 

# a = 0

# if(data ==1):
#     a = not a

# else:
#     a = not a

