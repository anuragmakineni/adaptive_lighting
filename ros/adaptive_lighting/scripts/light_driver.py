#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import serial
import string
import time

class pwm_driver(object):
    def __init__(self):
        self.initialized = False
        self.status_sub = rospy.Subscriber("~pwm", Float64MultiArray, self.pwm_cb)

        self.port = rospy.get_param('~port', '/dev/ttyUSB0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.t_out = rospy.get_param('~timeout', 0.05)
        self.initialized = False

        self.output = "0";

        try:
            self.s = serial.Serial(self.port, self.baudrate,
                                   timeout=self.t_out)
            self.initialized = True
            self.s.write(self.output.encode())
        except serial.SerialException:
            rospy.logerr("Serial Exception!")

    def pwm_cb(self, msg):
        if self.initialized and self.s.isOpen():
            pwms = np.array(msg.data)
            pwms = np.clip(pwms, 0.0, 1.0)

            pwm_a = float(pwms[0]) * 255
            pwm_a = int(round(pwm_a, 0))

            pwm_b = float(pwms[1]) * 255
            pwm_b = int(round(pwm_b, 0))

            pwm_c = float(pwms[2]) * 255
            pwm_c = int(round(pwm_c, 0))

            pwm_d = float(pwms[3]) * 255
            pwm_d = int(round(pwm_d, 0))

            self.output = "A"+str(pwm_a)+"B" + str(pwm_b) + "C" + str(pwm_c) + "D" + str(pwm_d)
            print(self.output)
            self.s.write(self.output.encode())

if __name__ == '__main__':
    rospy.init_node("pwm_driver")
    node = pwm_driver()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")

