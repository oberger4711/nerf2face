# ROS
import rospy
from perception_msgs.msg import FaceDetectionStamped
# Other libs
import numpy as np

from actuator import Actuator

def lerp(x, y, alpha):
    return (1 - alpha) * x + alpha * y

class Aimer:
    def __init__(self, actuator):
        self.actuator = actuator
        # Parameters
        self.acc_steps = 1
        self.pan_p = 0
        self.pan_d = 0
        self.tilt_p = 0
        self.tilt_d = 0
        # State
        self.reset()

    def set_parameters(self, acc_steps, pan_p, pan_i, pan_d, tilt_p, tilt_i, tilt_d):
        self.acc_steps = acc_steps
        self.pan_p = pan_p
        self.pan_i = pan_i
        self.pan_d = pan_d
        self.tilt_p = tilt_p
        self.tilt_i = tilt_i
        self.tilt_d = tilt_d

    def reset(self):
        self.step = 0
        self.integral = np.zeros(2)
        self.previous_error = None
        self.previous_stamp = None

    def get_actual_pan_and_tilt(self):
        return self.actuator.get_actual_pan_and_tilt()

    def aim(self, target, stamp):
        if self.previous_stamp is not None:
            time_step = stamp - self.previous_stamp
        else:
            time_step = None
        # Old: used target values to control.
        #actual_pan = self.actuator.target_pan
        #actual_tilt = self.actuator.target_tilt
        # New: Use actual values to control.
        actual_pan, actual_tilt = self.actuator.get_actual_pan_and_tilt()

        # PD control
        actual = np.array([0.5, 0.5])
        error = target - actual
        # P
        velocity = np.array([self.pan_p, self.tilt_p]) * error
        # D
        if self.previous_error is not None and time_step is not None:
            velocity += np.array([self.pan_d, self.tilt_d]) * ((error - self.previous_error) / time_step)
        # I
        if time_step is not None:
            self.integral += error * time_step
            velocity += np.array([self.pan_i, self.tilt_i]) * self.integral
        control_output = np.array([actual_pan, actual_tilt]) + velocity
        if time_step is not None:
            # Anti-windup using clamping
            if control_output[0] > Actuator.PAN_MAX_ANGLE or control_output[0] < Actuator.PAN_MIN_ANGLE:
                self.integral[0] -= error[0] * time_step
            if control_output[1] > Actuator.TILT_MAX_ANGLE or control_output[1] < Actuator.TILT_MIN_ANGLE:
                self.integral[1] -= error[1] * time_step
        self.previous_error = error
        self.previous_stamp = stamp
        #rospy.loginfo("Velocity: {}, {}".format(velocity[0], velocity[1]))
        # Apply
        control_pan, control_tilt = control_output[0], control_output[1]
        if self.step < self.acc_steps:
            # Smooth acceleration a bit (hacky but better for servos).
            self.actuator.set_target_pan(lerp(actual_pan, control_pan, self.step / self.acc_steps)) 
            self.actuator.set_target_tilt(lerp(actual_tilt, control_tilt, self.step / self.acc_steps))
        else:
            # Out of acceleration phase already.
            self.actuator.set_target_pan(control_pan) 
            self.actuator.set_target_tilt(control_tilt)
        # TODO: Use actual values from image timestamps to control.
        self.step += 1
        return error
