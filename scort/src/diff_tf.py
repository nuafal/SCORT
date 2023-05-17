#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

#Parameters
wheeltrack = 0.143
wheelradius = 0.05
TPR = 127.32
frontleft_ticks = 0
frontright_ticks = 0
rearleft_ticks = 0
rearright_ticks = 0
last_frontleft_ticks = 0
last_frontright_ticks = 0
last_rearleft_ticks = 0
last_rearright_ticks = 0

x = 0.0
y = 0.0
th = 0.0

vx =  0.0
vy =  0.0
vth =  0.0

def frontleftTicksCallback(msg):
    global frontleft_ticks 
    frontleft_ticks = msg.data

def frontrightTicksCallback(msg):
    global frontright_ticks 
    frontright_ticks = msg.data
    
def rearleftTicksCallback(msg):
    global rearleft_ticks 
    rearleft_ticks = msg.data

def rearrightTicksCallback(msg):
    global rearright_ticks 
    rearright_ticks = msg.data

    
rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
frontleft_ticks_sub =rospy.Subscriber("/frontleft_ticks", Int16, leftTicksCallback)
frontright_ticks_sub =rospy.Subscriber("/frontright_ticks", Int16, rightTicksCallback)
rearleft_ticks_sub =rospy.Subscriber("/rearleft_ticks", Int16, leftTicksCallback)
rearright_ticks_sub =rospy.Subscriber("/rearright_ticks", Int16, rightTicksCallback)

odom_broadcaster = tf.TransformBroadcaster()

current_time = rospy.Time.now()
last_time = rospy.Time.now()

r = rospy.Rate(10)

while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    delta_FL = frontleft_ticks - last_frontleft_ticks
    delta_FR = frontright_ticks - last_frontright_ticks
    delta_RL = rearleft_ticks - last_rearleft_ticks
    delta_RR = rearright_ticks - last_rearright_ticks
    dfl = 2 * pi * wheelradius * delta_FL / TPR
    dfr = 2 * pi * wheelradius * delta_FR / TPR
    drl = 2 * pi * wheelradius * delta_RL / TPR
    drr = 2 * pi * wheelradius * delta_RR / TPR

    dc = (dfl + dfr + drl + drr) / 4
    dt = (current_time - last_time).to_sec()
    dth = (dfr - dfl - drr + drl) / (2 * wheeltrack)

    if dfr == dfl:
        dx = dfr * cos(th)
        dy = dfr * sin(th)
    else:
        radius = (wheeltrack / 2) * (dfr + dfl) / (dfr - dfl)
        iccX = x - radius * sin(th)
        iccY = y + radius * cos(th)
        dx = cos(dth) * (x - iccX) - sin(dth) * (y - iccY) + iccX - x
        dy = sin(dth) * (x - iccX) + cos(dth) * (y - iccY) + iccY - y
        
    x += dx  
    y += dy 
    th =(th+dth) %  (2 * pi)

    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
       (x, y, 0.),
       odom_quat,
       current_time,
       "base_link",
       "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    if dt>0:
       vx=dx/dt
       vy=dy/dt
       vth=dth/dt

    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    odom_pub.publish(odom)

    last_frontleft_ticks = frontleft_ticks
    last_frontright_ticks = frontright_ticks
    last_rearleft_ticks = rearleft_ticks
    last_rearright_ticks = rearright_ticks
    last_time = current_time
    r.sleep()
