#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time


class image_saver(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("~image_raw", Image, self.image_cb)
        plt.ion()
        self.fig = plt.gcf()
        self.ax = self.fig.gca(projection='3d')
        self.frame_number = 0
        self.show_image = False
        self.export_image = True
        plt.show()

    def image_cb(self, img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        if self.show_image:
            cv2.imshow("Image", v)
            cv2.waitKey(10)

        if self.export_image:
            f_name = 'frame_' + str(self.frame_number).zfill(4) + '.png'
            cv2.imwrite(f_name, v)
            self.frame_number += 1


if __name__ == '__main__':
    rospy.init_node("image_saver")
    node = image_saver()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")
        cv2.destroyAllWindows()
