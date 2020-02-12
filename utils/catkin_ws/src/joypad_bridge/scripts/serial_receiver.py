#!/usr/bin/env python

import rospy, serial, struct, time
from serial import SerialException
import otto_communication_pb2
from google.protobuf.message import DecodeError
import math
from math import sin, cos, pi
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf

import numpy



ser = serial.Serial(
        baudrate=115200,
        timeout = 1000,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False,
        exclusive=False)
    
def serial_receiver():
    while(ser.is_open == False):
        try:
            ser.port = '/dev/ttyUSB0'
            ser.open()
        except SerialException:
            print('couldnt open /dev/ttyUSB0')
            time.sleep(2)
    
    print('ttyUSB0 opened')
    rospy.init_node('serial_receiver', anonymous=True)

    rospy.Publisher('/odom', Odometry, queue_size=10)
    
    otto_status = otto_communication_pb2.StatusMessage()
    otto_status.linear_velocity = 0
    otto_status.angular_velocity = 0
    otto_status.delta_millis = 0
    otto_status.status = otto_communication_pb2.StatusMessage.Status.UNKNOWN
    
    encoded_buffer = otto_status.SerializeToString()

    status_length = len(encoded_buffer)

    odom_values = numpy.array([0,0,0]) #x, y, th
    icc_x = 0;
    icc_y = 0;
    radius = 0;
    
    while (1):
        encoded_buffer = ser.read(status_length)
        ser.reset_input_buffer()
        try:
            otto_status.ParseFromString(encoded_buffer)
            print(otto_status)
            
            lin_vel = otto_status.linear_velocity
            ang_vel = otto_status.angular_velocity
            d_time = otto_status.delta_millis
            d_time = d_time / 1000.0
            print(d_time)
            if(ang_vel != 0 and d_time > 0):
                radius = lin_vel / ang_vel
                icc_x = (odom_values[0] -radius*sin(odom_values[2]))
                icc_y = (odom_values[1] + radius*cos(odom_values[2]))
                rotation_matrix = numpy.array([[cos(ang_vel*d_time), -sin(ang_vel*d_time), 0], [sin(ang_vel*d_time), cos(ang_vel*d_time), 0], [0, 0, 1]])
                translate_icc_matrix = numpy.array([odom_values[0] - icc_x, odom_values[1] - icc_y, odom_values[2]])
                translate_back_matrix = numpy.array([icc_x, icc_y, ang_vel*d_time])
                odom_values = rotation_matrix.dot(translate_icc_matrix) + translate_back_matrix
            elif(d_time > 0):
                odom_values = numpy.array([odom_values[0]+(lin_vel/d_time), odom_values[1]+(lin_vel/d_time), odom_values[2]])
            
            print(odom_values)
            
            
        except DecodeError:
            print("Decode Error")
      

if __name__ == '__main__':
    serial_receiver()
