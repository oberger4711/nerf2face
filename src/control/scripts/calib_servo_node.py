#!/usr/bin/env python

import time
# ROS
import rospy
# Other libs
from adafruit_servokit import ServoKit

from actuator import Actuator

NODE_NAME = "calib_servo_node.py"
NUM_SAMPLES = 5

class CalibServoNode:
    def __init__(self):
        self.actuator = Actuator()
        self.pan_voltage_samples = []
        self.tilt_voltage_samples = []

    def calibrate(self):
        self.pan_voltage_samples = []
        self.tilt_voltage_samples = []
        for i in range(NUM_SAMPLES):
            fraction = i / (NUM_SAMPLES - 1)
            rospy.loginfo("{:.2%}".format(fraction))
            self.actuator.set_pan(Actuator.PAN_MIN_ANGLE + (Actuator.PAN_MAX_ANGLE - Actuator.PAN_MIN_ANGLE) * fraction)
            self.actuator.set_tilt(Actuator.TILT_MIN_ANGLE + (Actuator.TILT_MAX_ANGLE - Actuator.TILT_MIN_ANGLE) * fraction)
            time.sleep(1) # Let servos control and stop shaking.
            # TODO: Read potis and add to lists.

    def dumpParameters(self):
        # TODO: Save samples as yaml file in cfg dir.
        pass

def main():
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        node = CalibServoNode()
        node.calibrate()
        node.dumpParameters()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
