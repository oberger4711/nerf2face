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

import control.srv
from actuator import Actuator, ServoCalibration
from aimer import Aimer

NODE_NAME = "shooter_node"

class ShooterNode:
    def __init__(self):
        self.actuator = Actuator()
        self.aimer = Aimer(self.actuator)
        self.reconfigure_server = dynamic_reconfigure.server.Server(ShooterConfig, self.handle_reconfigure)
        self.reset_service = rospy.Service("{}/reset".format(NODE_NAME), std_srvs.srv.Empty, self.handle_reset)
        self.move_service = rospy.Service("{}/move".format(NODE_NAME), control.srv.Move, self.handle_move)
        self.face_detection_sub = rospy.Subscriber("{}/face_detection".format(NODE_NAME), FaceDetectionStamped, self.handle_face_detection)
        self.aim_error_pub = rospy.Publisher("{}/aim_error".format(NODE_NAME), PointStamped, queue_size=100)
        self.shot = False

    def handle_reconfigure(self, config, level):
        # Update parameters and reset.
        self.config = config
        pan_calib = ServoCalibration(
            rospy.get_param("{}/pan_angle_start".format(NODE_NAME)),
            rospy.get_param("{}/pan_angle_stop".format(NODE_NAME)),
            rospy.get_param("{}/pan_voltage_samples".format(NODE_NAME))
        )
        tilt_calib = ServoCalibration(
            rospy.get_param("{}/tilt_angle_start".format(NODE_NAME)),
            rospy.get_param("{}/tilt_angle_stop".format(NODE_NAME)),
            rospy.get_param("{}/tilt_voltage_samples".format(NODE_NAME))
        )
        self.actuator.set_parameters(pan_calib, tilt_calib)
        self.aimer.set_parameters(config.aim_acc_steps, config.aim_pan_p, config.aim_pan_d, config.aim_tilt_p, config.aim_tilt_d)
        rospy.loginfo("Reconfigured.")
        return config

    def handle_reset(self, req):
        self.reset()
        return std_srvs.srv.EmptyResponse()

    def reset(self):
        # TODO: Reset state etc.
        self.shot = False
        self.actuator.set_default_orientation()
        rospy.loginfo("Reset.")

    def move(self, pan, tilt):
        rospy.loginfo("Moving to pan = {}, tilt = {}".format(pan, tilt))
        self.actuator.move_to(pan, tilt)
        time.sleep(0.2)
        self.actuator.set_target_pan(None)
        self.actuator.set_target_tilt(None)

    def handle_move(self, req):
        self.move(req.pan, req.tilt)
        return control.srv.MoveResponse()

    def handle_face_detection(self, det_msg):
        #if self.shot: return
        if det_msg.face_detection.detected:
            # Mirror coordinates for pan.
            target = np.array([1 - det_msg.face_detection.x_center, det_msg.face_detection.y_center])
            error = self.aimer.aim(target)
            self.aim_error_pub.publish(PointStamped(Header(0, det_msg.header.stamp, ""), Point(error[0], error[1], 0)))

            #rospy.loginfo(error)
            if not self.shot and np.amax(np.absolute(error)) < 0.10:
                rospy.loginfo("Shooting.")
                self.actuator.set_target_pan(None)
                self.actuator.set_target_tilt(None)
                time.sleep(0.05)
                self.actuator.pull_trigger()
                self.shot = True
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
