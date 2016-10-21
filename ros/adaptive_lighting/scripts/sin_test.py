import rospy
from std_msgs.msg import Float64
import math

pub = rospy.Publisher('/pwm_driver/pwm', Float64, queue_size=10)
rospy.init_node('sin_test')

t = 0.0
r = rospy.Rate(50)

while not rospy.is_shutdown():
    val = 0.5*(math.sin(5.0*t) + 1.0);
    pub.publish(val)
    t = t + 0.02;
    r.sleep()
