#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

def callback(data):
    twist = Twist()
    twist.linear.x = data.axes[2]
    twist.angular.z = data.axes[0]
    pub.publish(twist)

def start():
    global pub
    rospy.init_node('joy_to_cmd_vel')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("joy", Joy, callback)
    rospy.spin()

if __name__ == '__main__':
    start()

