#!/usr/bin/env python

import rospy, serial, struct, time
import otto_communication_pb2
from nav_msgs.msg import Odometry
from serial import SerialException
from google.protobuf.message import DecodeError


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
    
    while (1):
        encoded_buffer = ser.read(status_length)
        ser.reset_input_buffer()
        try:
            otto_status.ParseFromString(encoded_buffer)
            print(otto_status)
        except DecodeError:
            print("Decode Error")
      

if __name__ == '__main__':
    serial_receiver()
