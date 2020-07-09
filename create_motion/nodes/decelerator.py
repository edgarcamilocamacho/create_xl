#!/usr/bin/env python
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist
from collections import deque

pub = None

vel_lin_x = 0.0
vel_rot_z = 0.0

cmd_hz = 0.0
acc_lin_x = 0.0
acc_rot_z = 0.0

def callback(data):
    global vel_lin_x, vel_rot_z

    if data.linear.x > vel_lin_x:
        vel_lin_x += acc_lin_x
        if vel_lin_x > data.linear.x:
            vel_lin_x = data.linear.x
    elif data.linear.x < vel_lin_x:
        vel_lin_x -= acc_lin_x
        if vel_lin_x < data.linear.x:
            vel_lin_x = data.linear.x
    
    if data.angular.z > vel_rot_z:
        vel_rot_z += acc_rot_z
        if vel_rot_z > data.angular.z:
            vel_rot_z = data.angular.z
    elif data.angular.z < vel_rot_z:
        vel_rot_z -= acc_rot_z
        if vel_rot_z < data.angular.z:
            vel_rot_z = data.angular.z
            
    data.linear.x = vel_lin_x
    data.angular.z = vel_rot_z
    pub.publish(data)
    
def main():
    global pub, acc_lin_x, acc_rot_z, cmd_hz
    pub = rospy.Publisher('/cmd_vel_out', Twist, queue_size=10)
    rospy.init_node('decelerator', anonymous=True)
    rospy.Subscriber('/cmd_vel_in', Twist, callback)
    cmd_hz = rospy.get_param('~cmd_hz')
    acc_lin_x = rospy.get_param('~acc_lin_x') / cmd_hz
    acc_rot_z = rospy.get_param('~acc_rot_z') / cmd_hz
    rospy.spin()

if __name__ == '__main__':
    main()
