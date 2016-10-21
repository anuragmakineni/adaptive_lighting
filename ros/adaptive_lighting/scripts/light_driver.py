#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
import serial
import string
import time

class pwm_driver(object):
    def __init__(self):
        self.initialized = False
        self.status_sub = rospy.Subscriber("~pwm", Float64, self.pwm_cb)

        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 9600)
        self.t_out = rospy.get_param('~timeout', 0.05)
        self.initialized = False


        try:
            self.s = serial.Serial(self.port, self.baudrate,
                                   timeout=self.t_out)
            self.initialized = True
        except serial.SerialException:
            rospy.logerr("Serial Exception!")

    def pwm_cb(self, msg):
        if self.initialized and self.s.isOpen():
            pwm = msg.data
            control_val = float(pwm) * 255
            control_val = int(round(control_val, 0))

            self.output = str(control_val)
            print(self.output)
            self.s.write(self.output.encode())

if __name__ == '__main__':
    rospy.init_node("pwm_driver")
    node = pwm_driver()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")

