from threading import RLock
import time
# ROS
import rospy
# Other libs
import numpy as np
import board
import busio
i2c = busio.I2C(board.SCL, board.SDA)
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_servokit import ServoKit

class ServoCalibration:
    def __init__(self, min_angle, max_angle, voltage_samples):
        self.angle_samples = np.linspace(min_angle, max_angle, len(voltage_samples))
        self.voltage_samples = np.array(voltage_samples)

    def lookupAngle(self, voltage):
        # Use linear interpolation between samples.
        return np.interp(voltage, self.voltage_samples, self.angle_samples)

class Actuator:
    """ Thread-safe abstraction of the robot actuators.
    """
    PAN_MIN_ANGLE = 0
    PAN_MAX_ANGLE = 110
    TILT_MIN_ANGLE = 100
    TILT_MAX_ANGLE = 130
    TRIGGER_READY_ANGLE = 130
    TRIGGER_PULL_ANGLE = 93

    def __init__(self):
        self.lock = RLock()
        self.servo_kit = ServoKit(channels=16)
        self.pan_servo = self.servo_kit.servo[0]
        self.tilt_servo = self.servo_kit.servo[1]
        self.trigger_servo = self.servo_kit.servo[2]
        self.set_default_orientation()
        self.ads = ADS.ADS1115(i2c)
        self.pan_ai = AnalogIn(self.ads, ADS.P0)
        self.tilt_ai = AnalogIn(self.ads, ADS.P1)

    def set_parameters(self, pan_calib, tilt_calib):
        self.pan_calib = pan_calib
        self.tilt_calib = tilt_calib

    def set_default_orientation(self):
        self.set_target_pan(90)
        self.set_target_tilt(120)
        self.trigger_servo.angle = Actuator.TRIGGER_READY_ANGLE
        rospy.loginfo("Set initial orientation.")

    def set_target_pan(self, new_pan):
        if new_pan is not None:
            clipped_new_pan = np.clip(new_pan, Actuator.PAN_MIN_ANGLE, Actuator.PAN_MAX_ANGLE)
            with self.lock:
                self.pan_servo.angle = clipped_new_pan
                self.target_pan = clipped_new_pan
            #rospy.loginfo("Pan: {}".format(self.pan_servo.angle))
        else:
            with self.lock:
                self.pan_servo.angle = None

    def set_target_tilt(self, new_tilt):
        if new_tilt is not None:
            clipped_new_tilt = np.clip(new_tilt, Actuator.TILT_MIN_ANGLE, Actuator.TILT_MAX_ANGLE)
            with self.lock:
                self.tilt_servo.angle = clipped_new_tilt
                self.target_tilt = clipped_new_tilt
            #rospy.loginfo("Tilt: {}".format(self.tilt_servo.angle))
        else:
            with self.lock:
                self.tilt_servo.angle = None
    
    def get_actual_pan_voltage(self):
        with self.lock:
            return self.pan_ai.voltage
    
    def get_actual_pan(self):
        pan_voltage = self.get_actual_pan_voltage()
        if self.pan_calib is not None:
            return self.pan_calib.lookupAngle(pan_voltage)
        else:
            return None
    
    def get_actual_tilt_voltage(self):
        with self.lock:
            return self.tilt_ai.voltage

    def get_actual_tilt(self):
        tilt_voltage = self.get_actual_tilt_voltage()
        if self.tilt_calib is not None:
            return self.tilt_calib.lookupAngle(tilt_voltage)
        else:
            return None

    def pull_trigger(self):
        self.trigger_servo.angle = Actuator.TRIGGER_PULL_ANGLE
        time.sleep(0.2)
        self.trigger_servo.angle = Actuator.TRIGGER_READY_ANGLE
