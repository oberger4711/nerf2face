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
        self.step = 0
        self.previous_error = None

    def set_parameters(self, acc_steps, pan_p, pan_d, tilt_p, tilt_d):
        self.acc_steps = acc_steps
        self.pan_p = pan_p
        self.pan_d = pan_d
        self.tilt_p = tilt_p
        self.tilt_d = tilt_d

    def reset(self):
        self.step = 0
        self.previous_error = None

    def aim(self, target):
        # PD control
        actual = np.array([0.5, 0.5])
        error = target - actual
        velocity = np.array([self.pan_p, self.tilt_p]) * error
        if self.previous_error is not None:
            velocity += np.array([self.pan_d, self.tilt_d]) * (error - self.previous_error)
        self.previous_error = error
        #rospy.loginfo("Velocity: {}, {}".format(velocity[0], velocity[1]))
        # Apply
        # Old: used target values to control.
        #self.actuator.set_target_pan(self.actuator.target_pan + velocity[0])
        #self.actuator.set_target_tilt(self.actuator.target_tilt + velocity[1])
        # New: Use actual values to control.
        actual_pan = self.actuator.get_actual_pan()
        actual_tilt = self.actuator.get_actual_tilt()
        # Smooth acceleration a bit (hacky but better for servos).
        self.actuator.set_target_pan(lerp(actual_pan, actual_pan - velocity[0], self.step / self.acc_steps)) # Minus because rotation is other way around.
        self.actuator.set_target_tilt(lerp(actual_tilt, actual_tilt + velocity[1], self.step / self.acc_steps))
        # TODO: Use actual values from image timestamps to control.
        self.step += 1
        return error
