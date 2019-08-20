# ROS
import rospy
from perception_msgs.msg import FaceDetectionStamped
# Other libs
import numpy as np
from adafruit_servokit import ServoKit

class Aimer:
    def __init__(self, servo_kit):
        self.pan_p = 0
        self.pan_d = 0
        self.tilt_p = 0
        self.tilt_d = 0
        self.previous_error = None
        self.servo_kit = servo_kit
        self.pan_servo = self.servo_kit.servo[0]
        self.tilt_servo = self.servo_kit.servo[1]
        self.trigger_servo = self.servo_kit.servo[2]
        self.set_default_orientation()

    def set_default_orientation(self):
        self.set_pan(90)
        self.set_tilt(90)
        rospy.loginfo("Set initial orientation.")

    def set_parameters(self, pan_p, pan_d, tilt_p, tilt_d):
        self.pan_p = pan_p
        self.pan_d = pan_d
        self.tilt_p = tilt_p
        self.tilt_d = tilt_d

    def reset(self):
        self.previous_error = None

    def set_pan(self, new_pan):
        clipped_new_pan = np.clip(new_pan, 60, 120)
        self.pan_servo.angle = clipped_new_pan
        self.pan = clipped_new_pan
        #rospy.loginfo("Pan: {}".format(self.pan_servo.angle))

    def set_tilt(self, new_tilt):
        clipped_new_tilt = np.clip(new_tilt, 80, 140)
        self.tilt_servo.angle = clipped_new_tilt
        self.tilt = clipped_new_tilt
        #rospy.loginfo("Tilt: {}".format(self.tilt_servo.angle))

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
        self.set_pan(self.pan + velocity[0])
        self.set_tilt(self.tilt + velocity[1])
        return error
