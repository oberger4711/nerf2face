#!/usr/bin/env python

import subprocess
import time

# ROS
import rospy
import std_srvs.srv
# Raspberry
import gpiozero
import signal

NODE_NAME = "nerf2face_node"

GPIO_PIN_STATE_BUTTON = 4

def call_empty_service(service_path):
    rospy.wait_for_service(service_path)
    try:
        service = rospy.ServiceProxy(service_path, std_srvs.srv.Empty)
        service()
    except rospy.ServiceException as e:
        rospy.loginfo("Failed to call service '{}': {}".format(service_path, e))

def reset_shooter():
    call_empty_service("/shooter_node/reset")

def park_shooter():
    call_empty_service("/shooter_node/park")

def reset_face_tracker():
    call_empty_service("/face_tracker_node/reset")

class Nerf2FaceNode:
    def __init__(self):
        self.state = "PARKED"
        self.enter_parked()
        self.state_button = gpiozero.Button(GPIO_PIN_STATE_BUTTON, hold_time=2)
        self.state_button.when_pressed = self.toggle_state
        self.state_button.when_held = self.shutdown
    
    def enter_shooting(self):
        self.state = "SHOOTING"
        reset_shooter()
        reset_face_tracker()

    def enter_parked(self):
        self.state = "PARKED"
        park_shooter()
        reset_face_tracker()

    def toggle_state(self):
        if self.state == "PARKED":
            self.enter_shooting()
        else:
            self.enter_enter_parked()

    def shutdown(self):
        rospy.loginfo("Shutting down...")
        time.sleep(0.3)
        subprocess.check_call(["sudo", "poweroff"])

    def loop(self):
        signal.pause() # Wait for button events.

def main():
    try:
        rospy.init_node(NODE_NAME, anonymous=False)
        node = Nerf2FaceNode()
        node.loop()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
