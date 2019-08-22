from threading import RLock
# ROS
import rospy
# Other libs
import numpy as np
from adafruit_servokit import ServoKit

class Actuator:
    """ Thread-safe abstraction of the robot actuators.
    """
    PAN_MIN_ANGLE = 60
    PAN_MAX_ANGLE = 120
    TILT_MIN_ANGLE = 80
    TILT_MAX_ANGLE = 140

    def __init__(self):
        self.lock = RLock()
        self.servo_kit = ServoKit(channels=16)
        self.pan_servo = self.servo_kit.servo[0]
        self.tilt_servo = self.servo_kit.servo[1]
        self.trigger_servo = self.servo_kit.servo[2]
        self.set_default_orientation()

    def set_default_orientation(self):
        self.set_target_pan(90)
        self.set_target_tilt(90)
        rospy.loginfo("Set initial orientation.")

    def set_target_pan(self, new_pan):
        clipped_new_pan = np.clip(new_pan, Actuator.PAN_MIN_ANGLE, Actuator.PAN_MAX_ANGLE)
        with self.lock:
            self.pan_servo.angle = clipped_new_pan
            self.target_pan = clipped_new_pan
        #rospy.loginfo("Pan: {}".format(self.pan_servo.angle))

    def set_target_tilt(self, new_tilt):
        clipped_new_tilt = np.clip(new_tilt, Actuator.TILT_MIN_ANGLE, Actuator.TILT_MAX_ANGLE)
        with self.lock:
            self.tilt_servo.angle = clipped_new_tilt
            self.target_tilt = clipped_new_tilt
        #rospy.loginfo("Tilt: {}".format(self.tilt_servo.angle))
    
    def get_actual_pan_voltage(self):
        # TODO
        pass
    
    def get_actual_pan(self):
        pan_voltage = self.get_actual_pan_voltage()
        # TODO: Translate using lookup table.
        pass
    
    def get_actual_tilt_voltage(self):
        # TODO
        pass

    def get_actual_tilt(self):
        tilt_voltage = self.get_actual_tilt_voltage()
        # TODO: Translate using lookup table.
        pass
