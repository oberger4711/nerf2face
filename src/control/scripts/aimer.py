# ROS
import rospy
from perception_msgs.msg import FaceDetectionStamped
# Other libs
import numpy as np

from actuator import Actuator

class Aimer:
    def __init__(self, actuator):
        self.actuator = actuator
        self.pan_p = 0
        self.pan_d = 0
        self.tilt_p = 0
        self.tilt_d = 0
        self.previous_error = None

    def set_parameters(self, pan_p, pan_d, tilt_p, tilt_d):
        self.pan_p = pan_p
        self.pan_d = pan_d
        self.tilt_p = tilt_p
        self.tilt_d = tilt_d

    def reset(self):
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
        self.actuator.set_target_pan(self.actuator.target_pan + velocity[0])
        self.actuator.set_target_tilt(self.actuator.target_tilt + velocity[1])
        return error
