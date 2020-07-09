#!/usr/bin/env python
import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from nav_msgs.msg import Odometry
from collections import deque

prev_secs = -1
prev_nsecs = -1
prev_x = 0.0
prev_y = 0.0
prev_r = 0.0

flag_zero = False

pub = None

plot = []

buffer_v = deque(maxlen=3)
buffer_w = deque(maxlen=3)

def callback(data):
    global prev_nsecs, prev_secs, prev_x, prev_y, prev_r, plot, flag_zero, pub
    pub_flag = False
    r = R.from_quat([data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w]).as_euler('xyz')[2]
    if prev_secs != -1:
        dT = float(data.header.stamp.secs - prev_secs) + float(data.header.stamp.nsecs-prev_nsecs)/1000000000.0
        v = np.sqrt( (prev_x-data.pose.pose.position.x)**2 + (prev_y-data.pose.pose.position.y)**2 ) / dT        
        if data.twist.twist.linear.x<0.0:
            v *= -1.0 
        if r>np.pi/2 and prev_r<-np.pi/2:
            w = (r-(prev_r+2*np.pi))/dT
        elif r<-np.pi/2 and prev_r>np.pi/2:
            w = ((r+2*np.pi)-prev_r)/dT    
        else:
            w = (r-prev_r)/dT

        if w==0:
            if flag_zero:
                # plot.append(w)
                pub_flag = True
        else:
            plot.append(w)
            pub_flag = True

        flag_zero = w==0.0
        # rospy.loginfo(v)
        # rospy.loginfo(w)
        # rospy.loginfo('---')
    prev_secs = data.header.stamp.secs
    prev_nsecs = data.header.stamp.nsecs
    prev_x = data.pose.pose.position.x
    prev_y = data.pose.pose.position.y
    prev_r = r

    if pub_flag:
        buffer_v.append(v)
        buffer_w.append(w)
        data.twist.covariance = np.eye(6,6).flatten()
        data.twist.twist.linear.x = np.mean(buffer_v)
        data.twist.twist.linear.y = 0.0
        data.twist.twist.linear.z = 0.0
        data.twist.twist.angular.x = 0.0
        data.twist.twist.angular.y = 0.0
        data.twist.twist.angular.z = np.mean(buffer_w)
        pub.publish(data)



    
def listener():
    global pub
    pub = rospy.Publisher('/odom_wheels_fixed', Odometry, queue_size=10)
    rospy.init_node('create_fix_odom', anonymous=True)
    rospy.Subscriber('/odom_wheels', Odometry, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
    # print('A')
    # plt.plot(plot)
    # plt.show()