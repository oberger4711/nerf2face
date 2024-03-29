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
        pan_angle_before, tilt_angle_before = None, None
        for i in range(NUM_SAMPLES):
            fraction = i / (NUM_SAMPLES - 1)
            pan_angle = Actuator.PAN_MIN_ANGLE + (Actuator.PAN_MAX_ANGLE - Actuator.PAN_MIN_ANGLE) * fraction
            tilt_angle = Actuator.TILT_MIN_ANGLE + (Actuator.TILT_MAX_ANGLE - Actuator.TILT_MIN_ANGLE) * fraction
            if pan_angle_before is None or tilt_angle_before is None:
                # Current angle is unknown (and cannot be read because no calibration).
                # Jump directly to target.
                self.actuator.set_target_pan(pan_angle)
                self.actuator.set_target_tilt(tilt_angle)
            else:
                # Previous angles known.
                # Plan smooth motion to target.
                self.actuator.move_from_to(pan_angle_before, pan_angle, tilt_angle_before, tilt_angle)
            if rospy.is_shutdown(): break # Emergency break
            time.sleep(0.5)
            self.actuator.set_target_pan(None)
            self.actuator.set_target_tilt(None)
            if rospy.is_shutdown(): break # Emergency break
            time.sleep(1) # Stop servo control and stop shaking.
            pan_voltage = self.actuator.get_actual_pan_voltage()
            tilt_voltage = self.actuator.get_actual_tilt_voltage()
            self.pan_voltage_samples += [pan_voltage]
            self.tilt_voltage_samples += [tilt_voltage]
            rospy.loginfo("{:.2%}: pan: {:.4}, tilt: {:.4}".format(fraction, pan_voltage, tilt_voltage))
            pan_angle_before, tilt_angle_before = pan_angle, tilt_angle
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
