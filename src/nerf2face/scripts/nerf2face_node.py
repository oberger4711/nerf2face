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

GPIO_PIN_IN_STATE_BUTTON = 4
GPIO_PIN_OUT_PARKED_LED = 5
GPIO_PIN_OUT_SHOOTING_LED = 6

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
        self.parked_led = gpiozero.LED(GPIO_PIN_OUT_PARKED_LED)
        self.shooting_led = gpiozero.LED(GPIO_PIN_OUT_SHOOTING_LED)
        self.state_button = gpiozero.Button(GPIO_PIN_IN_STATE_BUTTON, pull_up=False, hold_time=3)
        self.state_button.when_pressed = self.toggle_state
        self.state_button.when_held = self.shutdown
        self.enter_parked()
    
    def enter_shooting(self):
        rospy.loginfo("Switching to SHOOTING state.")
        self.state = "SHOOTING"
        self.parked_led.off()
        self.shooting_led.on()
        reset_shooter()
        reset_face_tracker()

    def enter_parked(self):
        rospy.loginfo("Switching to PARKED state.")
        self.state = "PARKED"
        self.shooting_led.off()
        self.parked_led.on()
        park_shooter()
        reset_face_tracker()

    def toggle_state(self):
        if self.state == "PARKED":
            self.enter_shooting()
        else:
            self.enter_parked()

    def shutdown(self):
        rospy.loginfo("Shutting down...")
        self.parked_led.off()
        self.shooting_led.off()
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
