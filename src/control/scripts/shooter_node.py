#!/usr/bin/env python

import rospy
from perception_msgs.msg import FaceDetectionStamped

NODE_NAME = "shooter_node"

class ShooterNode:
    def __init__(self):
        self.face_detection_sub = rospy.Subscriber("{}/face_detection".format(NODE_NAME), FaceDetectionStamped, self.handle_face_detection)

    def handle_face_detection(self, det_msg):
        rospy.loginfo(rospy.get_caller_id() + ": Received detection.")
        # TODO: Pass to state.

def main():
    try:
        rospy.init_node(NODE_NAME, anonymous=True)
        node = ShooterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
