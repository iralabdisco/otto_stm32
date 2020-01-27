#!/usr/bin/env python

import rospy, serial, struct, time
from geometry_msgs.msg import Twist

ser = serial.Serial(
	port='/dev/ttyUSB0',
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)

current_millis = 0 
last_millis = 0

def callback(data):
    global current_millis
    global last_millis
    #da fixare
    linear = -data.linear.x
    angular = -data.angular.z
    last_millis = current_millis
    current_millis = int(round(time.time() * 1000))
    rospy.loginfo('I heard %f %f', linear, angular)
    if (1):
        msg_output = struct.pack('<ff', linear, angular)
        ser.write(msg_output)
        ser.flush()
    


def listener():

    rospy.init_node('cmd_vel_transmitter', anonymous=True)

    rospy.Subscriber('/cmd_vel', Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
