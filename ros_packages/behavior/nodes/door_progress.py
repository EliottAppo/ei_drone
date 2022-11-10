#!/usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import String, Bool
from multiprocessing import Lock

class Door(Behavior):
    def __init__(self, ):
        super().__init__("EnterDetectedDoor")
        self.door_detected = False
        self.door_detected_mutex = Lock()
        # "self.door_count" stores how many times a door was detected consecutively, while
        # "self.door_min_count" is the minimum necessary value to consider that a door was
        #  in fact detected
        self.door_count = 0
        self.door_count_mutex = Lock()
        self.door_min_count = 3
        
        self.door_exists_sub = rospy.Subscriber("/door_exists", Bool, self.door_exists_cb)
        self.command_pub = rospy.Publisher("/command", String, queue_size=1)

    def door_exists_cb(self, msg):
        with self.door_count_mutex:
            if msg.data:
                self.door_count += 1
        if self.door_count >= self.door_min_count:
            with self.door_detected_mutex:
                self.door_detected = True

    def get_door_detected(self):
        with self.door_detected_mutex:
            return self.door_detected

    def reset_detection(self):
        with self.door_count_mutex:
            self.door_count = 0
        with self.door_detected_mutex:
            self.door_detected = False

    def loop(self):
        while not rospy.is_shutdown():
            if self.get_status():
                if self.get_door_detected():
                    self.deactivate()
                    self.reset_detection()
                    self.command_pub.publish("CrossDoor")
                else:
                    # continue moving in the corrdior
                    # command_linear_x =  0.0
                    # command_linear_y =  0.0
                    # command_angular_z = 0.0
                    # linear_x_pub.publish(command_linear_x)
                    # linear_y_pub.publish(command_linear_y)
                    # angular_z_pub.publish(command_angular_z)
                    rospy.sleep(0.1) 