#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
import math

pub = rospy.Publisher('/adaptive_lighting/light_driver/pwm', Float64MultiArray, queue_size=10)
rospy.init_node('sin_test')

t = 0.0
intensity = 1.0;
rate = 200.0
r = rospy.Rate(rate)

while not rospy.is_shutdown():
    msg = Float64MultiArray()
    val = intensity * abs(math.sin(t));
    val_shift = intensity * abs(math.cos(t));
    currents = [0, 0, val, val_shift]
    msg.data = currents
    pub.publish(msg)
    t = t + 1.0/rate;
    r.sleep()
