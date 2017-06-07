#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
from array import array
import serial
import string
import time

class pwm_driver(object):
    def __init__(self):
        self.initialized = False
        self.status_sub = rospy.Subscriber("~pwm", Float64MultiArray, self.pwm_cb)

        self.port = rospy.get_param('~port', '/dev/ttyACM0')
        self.baudrate = rospy.get_param('~baudrate', 115200)
        self.t_out = rospy.get_param('~timeout', 0.05)
        self.initialized = False
        self.num_leds = rospy.get_param('n_leds', 9)

        self.packet = bytearray(2*self.num_leds + 2)
        self.packet[0] = ord('P')
        self.packet[-1] = ord('.')
        try:
            self.s = serial.Serial(self.port, self.baudrate,
                                   timeout=self.t_out, write_timeout=1.0)
            self.initialized = True
        except serial.SerialException:
            rospy.logerr("Serial Exception!")

    def pwm_cb(self, msg):
        print('callback')
        if self.initialized and self.s.isOpen():
            pwms = np.array(msg.data)
            pwms = np.clip(pwms, 0.0, 1.0)

            output = array('B')
            output.append(ord('P'))

            for i in range(0, self.num_leds):
                DB1 = np.uint8(int(round(pwms[i] * 4095, 0)) >> 8)
                DB2 = np.uint8(int(round(pwms[i] * 4095, 0)) & 0xFF)
                output.append(DB1)
                output.append(DB2)

            output.append(ord('\n'))
            print(output.tostring())
            written = self.s.write(output.tostring())
            print(written)

if __name__ == '__main__':
    rospy.init_node("pwm_driver")
    node = pwm_driver()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")

