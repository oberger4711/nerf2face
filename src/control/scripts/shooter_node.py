#!/usr/bin/env python

import rospy
import dynamic_reconfigure.server
from control.cfg import ShooterConfig
from perception_msgs.msg import FaceDetectionStamped

NODE_NAME = "shooter_node"

class ShooterNode:
    def __init__(self):
        self.face_detection_sub = rospy.Subscriber("{}/face_detection".format(NODE_NAME), FaceDetectionStamped, self.handle_face_detection)
        self.reconfigure_server = dynamic_reconfigure.server.Server(ShooterConfig, self.handle_reconfigure)

    def handle_reconfigure(self, config, level):
        rospy.loginfo("Reconfigured P: {}, D: {}".format(config.aim_p, config.aim_d))
        return config

    def handle_face_detection(self, det_msg):
        rospy.loginfo(rospy.get_caller_id() + ": Received detection.")
        # TODO: Pass to state.


def main():
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        node = ShooterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
