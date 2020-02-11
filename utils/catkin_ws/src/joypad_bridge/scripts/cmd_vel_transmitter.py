#!/usr/bin/env python

import rospy, serial, struct, time
import otto_communication_pb2
from geometry_msgs.msg import Twist
from serial import SerialException

ser = serial.Serial(
        baudrate=115200,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False)

def callback(data):
    linear = -data.linear.x
    angular = -data.angular.z      #da fixare?
    my_velocity = otto_communication_pb2.VelocityCommand()
    my_velocity.linear_velocity = linear;
    my_velocity.angular_velocity = angular;
    rospy.loginfo('I heard %f %f', linear, angular)
    out_buffer = my_velocity.SerializeToString()
    ser.write(out_buffer)
    ser.reset_output_buffer()
    time.sleep(0.001)
    #ser.flush()
    


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
