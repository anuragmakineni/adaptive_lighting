#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, UInt8
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class linear_controller(object):
    def __init__(self):
        # ROS Framework
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("~image_raw", Image, self.image_cb)
        self.led_sub = rospy.Subscriber("~pwm", Float64MultiArray, self.led_cb)
        self.led_pub = rospy.Publisher("~pwm", Float64MultiArray, queue_size=10)
        self.calibrate_srv = rospy.Service("~calibrate", Trigger, self.calibrate)
        self.level_sub = rospy.Subscriber("~level", UInt8, self.update_leds)

        # ROS Parameters
        self.pre_flash_value = rospy.get_param("pre_flash_value", 0.5)
        self.image_width = rospy.get_param("image_width", 2048)
        self.image_height = rospy.get_param("image_height", 1536)
        self.cropped_width = rospy.get_param("cropped_width", 500)
        self.cropped_height = rospy.get_param("cropped_height", self.image_height)
        self.n_leds = rospy.get_param("n_leds", 2)

        # LED Map
        self.led_a = [0, 0, 0, 0]
        self.led_b = [0, 0, 0, 0]
        self.led_c = [0, 0, 1, 0]
        self.led_d = [0, 0, 0, 1]

        # Current State of LED's
        self.initialized = False
        self.current_state = None
        self.background = np.zeros([self.cropped_height, self.cropped_width])
        self.last_frame = None

        # Control Matrices
        self.responses = [None, None, None, None]
        self.A_initialized = False
        self.A = np.zeros([self.cropped_height * self.cropped_width, self.n_leds])
        self.control = np.array([0.0, 0.0, 0.0, 0.0])

        # Set LED's to zero
        level = UInt8()
        level.data = 0
        self.update_leds(level)

    # Perform Least Squares Regression and Output new LED Values
    def update_leds(self, level):
        b = level.data * np.ones([self.cropped_height * self.cropped_width, 1])

        if self.A_initialized:
            sol = np.linalg.lstsq(self.A, b)
            x = sol[0]
            print(x)
        msg = Float64MultiArray()
        msg.data = self.control
        self.led_pub.publish(msg)

    def led_cb(self, msg):
        self.current_state = np.array(msg.data)
        self.initialized = True

    def image_cb(self, img):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
        except CvBridgeError as e:
            print(e)

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv)

        # Crop Image
        image_horz_center = int(self.image_width / 2.0)
        image_vert_center = int(self.image_height / 2.0)

        v= v[image_vert_center - int(self.cropped_height / 2.0):image_vert_center + int(
            self.cropped_height / 2.0),
                   image_horz_center - int(self.cropped_width / 2.0):image_horz_center + int(self.cropped_width / 2.0)]

        self.last_frame = np.array(v)

        # save new response matrix
        if self.initialized and sum(self.current_state) == self.pre_flash_value and (sum(self.current_state > 0) == 1):
            current_led = self.current_state.nonzero()[0][0]
            normalized_response = np.rint(np.array(v) / self.pre_flash_value)
            self.responses[current_led] = normalized_response
            print("Got New Frame for LED: " + str(current_led))

        # save background calibration
        if self.initialized and sum(self.current_state) < 1e-6:
            self.background = np.array(v)
            print("Got New Background Frame")

    # Convert the responses to the A matrix.
    def calibrate(self, srv):
        response_matrices = []
        for i in range(len(self.responses)):
            if self.responses[i] is not None:
                response_matrices.append(self.responses[i] - self.background)

        for i in range(len(response_matrices)):
            self.A[:, i] = response_matrices[i].flatten()

        if len(response_matrices) == self.n_leds:
            self.A_initialized = True
            return TriggerResponse(True, "Complete A")
        elif len(response_matrices) > 0:
            return TriggerResponse(True, "Partial A")
        else:
            return TriggerResponse(False, "Empty A")


if __name__ == '__main__':
    rospy.init_node("linear_controller")
    node = linear_controller()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        print("Shutting Down.")
        cv2.destroyAllWindows()
