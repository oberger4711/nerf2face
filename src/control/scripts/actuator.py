import time
import threading
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

    def lookup_angle(self, voltage):
        return np.interp(voltage, self.voltage_samples, self.angle_samples)

def read_pan_tilt_voltage_thread(reader):
    while not reader.stop_ev.is_set():
        reader.update_pan_tilt_voltage()

class ServoAngleReader:
    def __init__(self):
        self.pan_calib = None
        self.tilt_calib = None
        self.voltage_lock = threading.Lock()
        self.stop_ev = threading.Event()
        # CONTINUOUS mode is not possible for multiple channels unfortunately.
        # See https://learn.adafruit.com/adafruit-4-channel-adc-breakouts/python-circuitpython
        # Possibly with two ADS1115, I could use continuous mode for both servo voltage reads.
        self.ads = ADS.ADS1115(i2c, mode=ADS.Mode.SINGLE)
        self.pan_ai = AnalogIn(self.ads, ADS.P0)
        self.tilt_ai = AnalogIn(self.ads, ADS.P1)
        self.pan_voltage, self.tilt_voltage = self.pan_ai.voltage, self.tilt_ai.voltage
        self.reading_thread = threading.Thread(target=read_pan_tilt_voltage_thread, args=(self,))
        self.reading_thread.daemon = True

    def set_calibration(self, pan_calib, tilt_calib):
        self.pan_calib, self.tilt_calib = pan_calib, tilt_calib

    def update_pan_tilt_voltage(self):
        pan_voltage = self.pan_ai.voltage # Internal read contains sleep of ~ 10 ms.
        with self.voltage_lock:
            self.pan_voltage = pan_voltage
        tilt_voltage = self.tilt_ai.voltage # Internal read contains sleep of ~ 10 ms.
        with self.voltage_lock:
            self.tilt_voltage = tilt_voltage
    
    def start(self):
        self.reading_thread.start()
    
    def get_actual_pan_and_tilt(self):
        if self.pan_calib is not None and self.tilt_calib is not None:
            pan_voltage, tilt_voltage = None, None
            with self.voltage_lock:
                pan_voltage, tilt_voltage = self.pan_voltage, self.tilt_voltage
            return self.pan_calib.lookup_angle(pan_voltage), self.tilt_calib.lookup_angle(tilt_voltage)
        else:
            return None, None
        

class Actuator:
    """ Thread-safe abstraction of the robot actuators.
    """
    PAN_MIN_ANGLE = 32
    PAN_MAX_ANGLE = 130
    TILT_MIN_ANGLE = 20
    TILT_MAX_ANGLE = 110
    TRIGGER_READY_ANGLE = 120
    TRIGGER_PULL_ANGLE = 93
    V_MOVE_MAX = 360
    A_MOVE_MAX = 240
    MOVE_FREQUENCY = 60

    def __init__(self):
        self.servo_kit = ServoKit(channels=16)
        self.pan_servo = self.servo_kit.servo[0]
        self.tilt_servo = self.servo_kit.servo[1]
        self.trigger_servo = self.servo_kit.servo[2]
        self.servo_angle_reader = ServoAngleReader()
        self.servo_angle_reader.start()

    def set_parameters(self, pan_calib, tilt_calib):
        self.servo_angle_reader.set_calibration(pan_calib, tilt_calib)

    def set_default_orientation(self):
        rospy.loginfo("Moving to default orientation.")
        self.trigger_servo.angle = Actuator.TRIGGER_READY_ANGLE
        self.move_to(60, 70)
        rospy.loginfo("Moved to default orientation.")

    def set_target_pan(self, new_pan):
        if new_pan is not None:
            clipped_new_pan = np.clip(new_pan, Actuator.PAN_MIN_ANGLE, Actuator.PAN_MAX_ANGLE)
            self.pan_servo.angle = clipped_new_pan
            self.target_pan = clipped_new_pan
            #rospy.loginfo("Pan: {}".format(self.pan_servo.angle))
        else:
            self.pan_servo.angle = None

    def set_target_tilt(self, new_tilt):
        if new_tilt is not None:
            clipped_new_tilt = np.clip(new_tilt, Actuator.TILT_MIN_ANGLE, Actuator.TILT_MAX_ANGLE)
            self.tilt_servo.angle = clipped_new_tilt
            self.target_tilt = clipped_new_tilt
            #rospy.loginfo("Tilt: {}".format(self.tilt_servo.angle))
        else:
            self.tilt_servo.angle = None
    
    def get_actual_pan_and_tilt(self):
        return self.servo_angle_reader.get_actual_pan_and_tilt()

    def move_from_to(self, start_pan, dest_pan, start_tilt, dest_tilt):
        pan, duration_pan = plan(start_pan, dest_pan, 0, Actuator.V_MOVE_MAX, Actuator.A_MOVE_MAX)
        tilt, duration_tilt = plan(start_tilt, dest_tilt, 0, Actuator.V_MOVE_MAX, Actuator.A_MOVE_MAX)
        duration_max = max(duration_pan, duration_tilt)
        rospy.loginfo("Planned move-to duration: {} s".format(duration_max))

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
        start_pan, start_tilt = self.servo_angle_reader.get_actual_pan_and_tilt()
        self.move_from_to(start_pan, dest_pan, start_tilt, dest_tilt)

    def pull_trigger(self):
        self.trigger_servo.angle = Actuator.TRIGGER_PULL_ANGLE
        time.sleep(0.2)
        self.trigger_servo.angle = Actuator.TRIGGER_READY_ANGLE
