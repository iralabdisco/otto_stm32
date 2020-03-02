#!/usr/bin/env python

import rospy, serial, struct, time, datetime
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
        baudrate=9600,
        timeout = None,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        rtscts=False,
        exclusive=False)

def serial_receiver():
    rospy.init_node('serial_receiver', anonymous=True, log_level=rospy.DEBUG)
    serial_port = rospy.get_param("serial_port")
    while(ser.is_open == False and not rospy.is_shutdown()):
        try:
            ser.port = serial_port
            ser.open()
        except SerialException:
            rospy.logerr('Couldn\'t open ' + serial_port)
            time.sleep(2)

    rospy.loginfo(serial_port + ' opened')

    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
    odom_broadcaster = tf.TransformBroadcaster()

    otto_status = otto_communication_pb2.StatusMessage()
    otto_status.linear_velocity = 0
    otto_status.angular_velocity = 0
    otto_status.delta_millis = 0
    otto_status.status = 0

    encoded_buffer = otto_status.SerializeToString()

    status_length = len(encoded_buffer)

    odom_values = numpy.array([0,0,0]) #x, y, th
    icc_x = 0;current_time = rospy.Time.now()
    icc_y = 0;
    radius = 0;
    
    while (not rospy.is_shutdown()):
        ser.reset_input_buffer()
        encoded_buffer = ser.read(status_length)
        
        try:
            otto_status.ParseFromString(encoded_buffer)
            rospy.logdebug(otto_status)
            
            # 3 = RUNNING
            if (otto_status.status == 3):
                lin_vel = otto_status.linear_velocity
                ang_vel = otto_status.angular_velocity
                d_time = otto_status.delta_millis
                d_time = d_time / 1000.0
                if(ang_vel != 0 and d_time > 0):
                    radius = lin_vel / ang_vel
                    icc_x = (odom_values[0] -radius*sin(odom_values[2]))
                    icc_y = (odom_values[1] + radius*cos(odom_values[2]))
                
                    rotation_matrix = numpy.array([[cos(ang_vel*d_time), -sin(ang_vel*d_time), 0], 
                                                    [sin(ang_vel*d_time), cos(ang_vel*d_time), 0], 
                                                    [0, 0, 1]])
                                                
                    translate_icc_matrix = numpy.array([odom_values[0] - icc_x, 
                                                        odom_values[1] - icc_y, 
                                                        odom_values[2]])
                
                    translate_back_matrix = numpy.array([icc_x, 
                                                        icc_y, 
                                                        ang_vel*d_time])
                
                    odom_values = rotation_matrix.dot(translate_icc_matrix) + translate_back_matrix
                    
                elif(d_time > 0):
                    odom_values = numpy.array([odom_values[0]+cos(odom_values[2])*(lin_vel/d_time), 
                                                odom_values[1]+sin(odom_values[2])*(lin_vel/d_time), 
                                                odom_values[2]])

            
                current_time = rospy.Time.now()
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, odom_values[2])
                odom_broadcaster.sendTransform((odom_values[0], odom_values[1], 0.), odom_quat, current_time, "base_link", "odom")
                odom = Odometry()
                odom.header.stamp = current_time
                odom.header.frame_id = "odom"

                odom.pose.pose = Pose(Point(odom_values[0], odom_values[1], 0.), Quaternion(*odom_quat))

                odom.child_frame_id = "base_link"
                odom.twist.twist = Twist(Vector3(lin_vel, 0, 0), Vector3(0, 0, ang_vel))

                odom_pub.publish(odom)

        except DecodeError:
            rospy.logerr("Decode Error")
            ser.reset_input_buffer()


if __name__ == '__main__':
    serial_receiver()
