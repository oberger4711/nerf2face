#!/usr/bin/env python

import threading
import time
# ROS
import rospy
import dynamic_reconfigure.server
from control.cfg import ShooterConfig
from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from perception_msgs.msg import FaceDetectionStamped
import std_srvs.srv
# Other libs
import numpy as np

from actuator import Actuator
from aimer import Aimer

NODE_NAME = "shooter_node"

class ShooterNode:
    def __init__(self):
        self.actuator = Actuator()
        self.aimer = Aimer(self.actuator)
        self.reconfigure_server = dynamic_reconfigure.server.Server(ShooterConfig, self.handle_reconfigure)
        self.reset_service = rospy.Service("{}/reset".format(NODE_NAME), std_srvs.srv.Empty, self.handle_reset)
        self.face_detection_sub = rospy.Subscriber("{}/face_detection".format(NODE_NAME), FaceDetectionStamped, self.handle_face_detection)
        self.aim_error_pub = rospy.Publisher("{}/aim_error".format(NODE_NAME), PointStamped, queue_size=100)

    def handle_reconfigure(self, config, level):
        # Update config and reset.
        self.config = config
        self.aimer.set_parameters(config.aim_pan_p, config.aim_pan_d, config.aim_tilt_p, config.aim_tilt_d)
        rospy.loginfo("Reconfigured.")
        return config

    def handle_reset(self, req):
        self.reset()
        return std_srvs.srv.EmptyResponse()

    def reset(self):
        # TODO: Reset state etc.
        rospy.loginfo("Reset.")

    def handle_face_detection(self, det_msg):
        if det_msg.face_detection.detected:
            # Mirror coordinates for pan.
            target = np.array([1 - det_msg.face_detection.x_center, det_msg.face_detection.y_center])
            error = self.aimer.aim(target)
            self.aim_error_pub.publish(PointStamped(Header(0, det_msg.header.stamp, ""), Point(error[0], error[1], 0)))
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
