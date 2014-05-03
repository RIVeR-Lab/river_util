#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
import math
from operator import add

def joystick_callback(data):
        global cmd_vel_pub

        linear = data.axes[1];
        angular = data.axes[0];
        
        twist = Twist();
        twist.angular.z = angular;
        twist.linear.x = linear;
        cmd_vel_pub.publish(twist)
    
def main():
        rospy.init_node('joystick_teleop', anonymous=True)

        global cmd_vel_pub
        cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
        
        rospy.Subscriber("joy", Joy, joystick_callback)
        
        rospy.spin()
        
if __name__ == '__main__':
        main()
