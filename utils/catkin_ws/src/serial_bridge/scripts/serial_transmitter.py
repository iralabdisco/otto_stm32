#!/usr/bin/env python

import time

import otto_communication_pb2

import rospy
from geometry_msgs.msg import Twist

import serial
from serial import SerialException

ser = serial.Serial(
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=True,
        exclusive=None)

def callback(data):
    linear = data.linear.x
    angular = -data.angular.z      #da fixare?
    my_velocity = otto_communication_pb2.VelocityCommand()
    my_velocity.linear_velocity = linear
    my_velocity.angular_velocity = angular
    rospy.logdebug('Cmd vel transmitted %f %f', linear, angular)
    out_buffer = my_velocity.SerializeToString()
    if(ser.cts == True):
        ser.write(out_buffer)
    else:
        rospy.logwarn('ST not ready to receive velocity cmd')
    ser.reset_output_buffer()
    
def listener():
    rospy.init_node('serial_transmitter', anonymous=True, log_level=rospy.DEBUG)
    
    serial_port = rospy.get_param("serial_port")
    #dtr is connected to RST, on opening dtr is high by default so it resets the st board
    #after opening the serial port we set it low so the board can boot
    ser.dtr = 0 
    while(ser.is_open == False and not rospy.is_shutdown()):
        try:
            ser.port = serial_port
            ser.open()
        except SerialException:
            rospy.logerr('Couldn\'t open ' + serial_port)
            time.sleep(2)
    
    rospy.loginfo(serial_port + ' opened')

    rospy.Subscriber('/cmd_vel', Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
