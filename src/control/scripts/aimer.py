# ROS
import rospy
from perception_msgs.msg import FaceDetectionStamped
# Other libs
import numpy as np
from adafruit_servokit import ServoKit

class Aimer:
    def __init__(self, servo_kit):
        self.p = 0
        self.d = 0
        self.previous_error = None
        '''
        self.servo_kit = servo_kit
        self.pan_servo = self.servo_kit.servo[0]
        self.tilt_servo = self.servo_kit.servo[1]
        self.trigger_servo = self.servo_kit.servo[2]
        self.set_default_orientation()
        '''

    def set_default_orientation():
        self.set_pan(90)
        self.set_tilt(90)

    def set_parameters(self, p, d):
        self.p = p
        self.d = d

    def reset(self):
        self.previous_error = None

    def set_pan(self, new_pan):
        self.pan_servo_angle = numpy.clip(new_pan, 70, 120)

    def set_tilt(self, new_tilt):
        self.tilt_servo_angle = numpy.clip(new_tilt, 70, 120)

    def aim(self, target):
        # PD control
        actual = np.array([0.5, 0.5])
        error = target - actual
        velocity = self.p * error
        if self.previous_error is not None:
            velocity += self.d * (error - self.previous_error)
        rospy.loginfo("Velocity: {}, {}".format(velocity[0], velocity[1]))
        # Apply
        '''
        set_pan(self.pan_servo.angle + velocity[0])
        set_tilt(self.tilt_servo.angle + velocity[1])
        '''
        return error
