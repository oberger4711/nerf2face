#!/usr/bin/env python

import math
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

def face_det_target(face_det):
    # Aim a bit on the side so that shot hits the face.
    return np.array([face_det.x_center + face_det.width * 0.51, face_det.y_center - face_det.height * 0.8])

class ShooterNode:
    def __init__(self):
        self.parked = True
        self.actuator = Actuator()
        self.aimer = Aimer(self.actuator)
        self.reconfigure_server = dynamic_reconfigure.server.Server(ShooterConfig, self.handle_reconfigure)
        self.reset_service = rospy.Service("{}/reset".format(NODE_NAME), std_srvs.srv.Empty, self.handle_reset)
        self.park_service = rospy.Service("{}/park".format(NODE_NAME), std_srvs.srv.Empty, self.handle_park)
        self.move_service = rospy.Service("{}/move".format(NODE_NAME), control.srv.Move, self.handle_move)
        self.face_detection_sub = rospy.Subscriber("{}/face_detection".format(NODE_NAME), FaceDetectionStamped, self.handle_face_detection)
        self.aim_error_pub = rospy.Publisher("{}/aim_error".format(NODE_NAME), PointStamped, queue_size=100)
        self.shot = False

    def handle_reconfigure(self, config, level):
        rospy.loginfo("Reconfiguring.")
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
        self.aimer.set_parameters(config.aim_acc_steps, config.aim_pan_p, config.aim_pan_i, config.aim_pan_d, config.aim_tilt_p, config.aim_tilt_i, config.aim_tilt_d)
        return config

    def handle_reset(self, req):
        self.reset()
        return std_srvs.srv.EmptyResponse()

    def reset(self):
        rospy.loginfo("Resetting.")
        self.aimer.reset()
        self.shot = False
        self.parked = False

    def handle_park(self, req):
        self.park()
        return std_srvs.srv.EmptyResponse()

    def park(self):
        rospy.loginfo("Parking.")
        self.shot = False
        self.parked = True
        self.actuator.set_default_orientation()

    def handle_move(self, req):
        self.move(req.pan, req.tilt)
        return control.srv.MoveResponse()

    def move(self, pan, tilt):
        rospy.loginfo("Moving to pan = {}, tilt = {}".format(pan, tilt))
        self.actuator.move_to(pan, tilt)
        time.sleep(0.2)
        self.actuator.set_target_pan(None)
        self.actuator.set_target_tilt(None)

    def aiming_at_face(self, ts, face_det):
        crosshairs = np.array([0.5, 0.5])
        # Simplify face to circle.
        face_center = face_det_target(face_det)
        face_radius = min(min(face_det.width, face_det.height) / 2, 0.1)
        diff = face_center - crosshairs
        distance_2 = diff[0] * diff[0] + diff[1] * diff[1]
        #rospy.loginfo("radius: {}, distance: {}".format(face_radius, distance_2))
        ready_to_shoot = False
        if (distance_2 < face_radius * face_radius):
            if self.stamp_on_face_start > 0.0:
                if ts - self.stamp_on_face_start > 0.3:
                    ready_to_shoot = True
            else:
                self.stamp_on_face_start = ts
        else:
            self.stamp_on_face_start = -1.0
        return ready_to_shoot

    def handle_face_detection(self, det_msg):
        if self.parked: return # Ignore message when parked.
        #age = rospy.Time.now() - det_msg.header.stamp
        #rospy.loginfo("age: {} s ({} % of framerate)".format(age.to_sec(), age.to_sec() / (1 / 40)))
        #if self.shot: return
        if det_msg.face_detection.detected:
            # Mirror coordinates for pan.
            target = face_det_target(det_msg.face_detection)
            error = self.aimer.aim(target, det_msg.header.stamp.to_time())
            self.aim_error_pub.publish(PointStamped(Header(0, det_msg.header.stamp, ""), Point(error[0], error[1], 0)))

            #rospy.loginfo(error)
            if not self.shot and self.aiming_at_face(det_msg.header.stamp.to_sec(), det_msg.face_detection):
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
