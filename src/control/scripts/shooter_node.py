#!/usr/bin/env python

# ROS
import rospy
import dynamic_reconfigure.server
from control.cfg import ShooterConfig
from perception_msgs.msg import FaceDetectionStamped
import std_srvs.srv
# Other libs
import numpy as np
from adafruit_servokit import ServoKit

from aimer import Aimer

NODE_NAME = "shooter_node"

class ShooterNode:
    def __init__(self):
        #self.servo_kit = ServoKit(channels=16)
        self.servo_kit = None
        self.aimer = Aimer(self.servo_kit)
        self.reconfigure_server = dynamic_reconfigure.server.Server(ShooterConfig, self.handle_reconfigure)
        self.reset_service = rospy.Service("{}/reset".format(NODE_NAME), std_srvs.srv.Empty, self.handle_reset)
        self.face_detection_sub = rospy.Subscriber("{}/face_detection".format(NODE_NAME), FaceDetectionStamped, self.handle_face_detection)

    def handle_reconfigure(self, config, level):
        # Update config and reset.
        self.config = config
        self.aimer.set_parameters(config.aim_p, config.aim_d)
        rospy.loginfo("Reconfigured.")
        return config

    def handle_reset(self, req):
        self.reset()
        return std_srvs.srv.EmptyResponse()

    def reset(self):
        # TODO: Reset state etc.
        rospy.loginfo("Reset.")

    def handle_face_detection(self, det_msg):
        #rospy.loginfo(rospy.get_caller_id() + ": Received detection.")
        if det_msg.face_detection.detected:
            # Mirror coordinates for pan / tilt.
            target = np.ones(2) - np.array([det_msg.face_detection.x_center, det_msg.face_detection.y_center])
            self.aimer.aim(target)
        else:
            self.aimer.reset()


def main():
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        node = ShooterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
