#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import math

pub = rospy.Publisher('/pwm_driver/pwm', Float64MultiArray, queue_size=10)
rospy.init_node('sin_test')

t = 0.0
rate = 20.0
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    msg = Float64MultiArray()
    val = 0.5*(math.sin(2.0*t) + 1.0);
    currents = [val, -val, val, -val]
    msg.data = currents
    pub.publish(msg)
    t = t + 1.0/rate;
    r.sleep()
