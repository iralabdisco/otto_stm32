#!/usr/bin/env python

import rospy, serial, struct, time
from geometry_msgs.msg import Twist
from serial import SerialException

ser = serial.Serial(
        baudrate=115200,
        parity=serial.PARITY_ODD,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)

def callback(data):
    linear = -data.linear.x
    angular = -data.angular.z      #da fixare?
    rospy.loginfo('I heard %f %f', linear, angular)
    msg_output = struct.pack('<ff', linear, angular)
    ser.write(msg_output)
    ser.flush()
    


def listener():
    while(ser.is_open == False):
        try:
            ser.port = '/dev/ttyUSB0'
            ser.open()
        except SerialException:
            print('couldnt open /dev/ttyUSB0')
            time.sleep(2)
    
    print('ttyUSB0 opened')
    rospy.init_node('cmd_vel_transmitter', anonymous=True)

    rospy.Subscriber('/cmd_vel', Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
