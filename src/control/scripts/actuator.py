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

from motion_plan.motion_plan import plan

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
    PAN_MIN_ANGLE = 32
    PAN_MAX_ANGLE = 130
    TILT_MIN_ANGLE = 20
    TILT_MAX_ANGLE = 110
    TRIGGER_READY_ANGLE = 130
    TRIGGER_PULL_ANGLE = 93
    V_MOVE_MAX = 360
    A_MOVE_MAX = 240
    MOVE_FREQUENCY = 60

    def __init__(self):
        self.lock = RLock()
        self.servo_kit = ServoKit(channels=16)
        self.pan_servo = self.servo_kit.servo[0]
        self.tilt_servo = self.servo_kit.servo[1]
        self.trigger_servo = self.servo_kit.servo[2]
        self.ads = ADS.ADS1115(i2c)
        self.pan_ai = AnalogIn(self.ads, ADS.P0)
        self.tilt_ai = AnalogIn(self.ads, ADS.P1)
        self.pan_calib = None
        self.tilt_calib = None

    def set_parameters(self, pan_calib, tilt_calib):
        initial_configuration = self.pan_calib is None or self.tilt_calib is None
        self.pan_calib = pan_calib
        self.tilt_calib = tilt_calib
        if initial_configuration:
            self.set_default_orientation()

    def set_default_orientation(self):
        rospy.loginfo("Moving to initial orientation.")
        self.trigger_servo.angle = Actuator.TRIGGER_READY_ANGLE
        self.move_to(60, 70)
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

    def move_from_to(self, start_pan, dest_pan, start_tilt, dest_tilt):
        pan, duration_pan = plan(start_pan, dest_pan, 0, Actuator.V_MOVE_MAX, Actuator.A_MOVE_MAX)
        tilt, duration_tilt = plan(start_tilt, dest_tilt, 0, Actuator.V_MOVE_MAX, Actuator.A_MOVE_MAX)
        duration_max = max(duration_pan, duration_tilt)
        rospy.loginfo("Move-to duration: {} s".format(duration_max))

        # Move synchronously at specified rate.
        rate = rospy.Rate(Actuator.MOVE_FREQUENCY)
        t_start = time.time()
        t_now = time.time()
        while (t_now - t_start) <= duration_max:
            if rospy.is_shutdown(): break # Emergency break
            self.set_target_pan(pan(t_now - t_start))
            self.set_target_tilt(tilt(t_now - t_start))
            rate.sleep()
            t_now = time.time()

    def move_to(self, dest_pan, dest_tilt):
        start_pan = self.get_actual_pan()
        start_tilt = self.get_actual_tilt()
        self.move_from_to(start_pan, dest_pan, start_tilt, dest_tilt)

    def pull_trigger(self):
        self.trigger_servo.angle = Actuator.TRIGGER_PULL_ANGLE
        time.sleep(0.2)
        self.trigger_servo.angle = Actuator.TRIGGER_READY_ANGLE
