#!/usr/bin/env python

import time
# ROS
import rospy
import rospkg
# Other libs
from adafruit_servokit import ServoKit
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

from actuator import Actuator

NODE_NAME = "calib_servo_node.py"
NUM_SAMPLES = 5
OUT_YAML_FILE_PATH = "{}/cfg/servo_calib.yaml".format(rospkg.RosPack().get_path("control"))

class CalibServoNode:
    def __init__(self):
        self.actuator = Actuator()
        self.pan_voltage_samples = []
        self.tilt_voltage_samples = []

    def calibrate(self):
        rospy.loginfo("Starting calibration routine with {} samples.".format(NUM_SAMPLES))
        self.pan_voltage_samples = []
        self.tilt_voltage_samples = []
        for i in range(NUM_SAMPLES):
            fraction = i / (NUM_SAMPLES - 1)
            pan_angle = Actuator.PAN_MIN_ANGLE + (Actuator.PAN_MAX_ANGLE - Actuator.PAN_MIN_ANGLE) * fraction
            self.actuator.set_target_pan(pan_angle)
            tilt_angle = Actuator.TILT_MIN_ANGLE + (Actuator.TILT_MAX_ANGLE - Actuator.TILT_MIN_ANGLE) * fraction
            self.actuator.set_target_tilt(tilt_angle)
            if rospy.is_shutdown(): break # Emergency break
            time.sleep(1) # Let servos control and stop shaking.
            pan_voltage = self.actuator.get_actual_pan_voltage()
            tilt_voltage = self.actuator.get_actual_tilt_voltage()
            self.pan_voltage_samples += [pan_voltage]
            self.tilt_voltage_samples += [tilt_voltage]
            rospy.loginfo("{:.2%}: pan: {:.2}, tilt: {:.2}".format(fraction, pan_voltage, tilt_voltage))
            if rospy.is_shutdown(): break # Emergency break

    def dumpParameters(self):
        params = {
                "/shooter_node/pan_angle_start" : Actuator.PAN_MIN_ANGLE,
                "/shooter_node/pan_angle_stop" : Actuator.PAN_MAX_ANGLE,
                "/shooter_node/pan_voltage_samples" : self.pan_voltage_samples,
                "/shooter_node/tilt_angle_start" : Actuator.TILT_MIN_ANGLE,
                "/shooter_node/tilt_angle_stop" : Actuator.TILT_MAX_ANGLE,
                "/shooter_node/tilt_voltage_samples" : self.tilt_voltage_samples
            }
        params_yaml = yaml.dump(params, Dumper=Dumper)
        rospy.loginfo("Dumping parameters to {}.".format(OUT_YAML_FILE_PATH))
        with open(OUT_YAML_FILE_PATH, "w") as f:
            f.write(params_yaml)

def main():
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        node = CalibServoNode()
        node.calibrate()
        node.dumpParameters()
        rospy.loginfo("Done.")
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
