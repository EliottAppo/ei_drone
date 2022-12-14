#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from multiprocessing import Lock
from behavior.msg import BehaviorStatus
from time import time
from std_msgs.msg import String


class Scheduler:
    def __init__(self, pub_behavior):
        self.queue = []
        self.time = None
        self.pub_behavior = pub_behavior

    def create_schedule(self, events):
        self.time = time()
        for behavior in events:
            self.queue.append(behavior)

    def publish(self):
        for behavior in self.queue:
            current_time = time()
            date, name = behavior
            if current_time-self.time > date:
                msg = BehaviorStatus()
                msg.name = name
                msg.status = True
                self.pub_behavior.publish(msg)
                self.queue.remove(self.queue[0])


class Command:
    def __init__(self):
        self.behaviors = [
            "TakeOff", "Land", "Hover", "MoveForward", "MoveBackward", "MoveLeft", "MoveRight", "MoveUp", "MoveDown", "TurnRight", "TurnLeft", "UTurnRight", "UTurnLeft", "SlideLeft", "SlideRight", "AlignCorridor", "CenterCorridor", "MoveForwardVp"
        ]
        self.commands = {
            "TakeOff": [(0, "TakeOff")],
            "Land": [(0, "Hover"), (0.3, "Land")],
            "Hover": [(0, "Hover")],
            "MoveForward": [(0, "MoveForward")],
            "MoveBackward": [(0, "MoveBackward")],
            "MoveLeft": [(0, "MoveLeft")],
            "MoveRight": [(0, "MoveRight")],
            "MoveUp": [(0, "MoveUp")],
            "MoveDown": [(0, "MoveDown")],
            "TurnRight": [(0, "TurnRight")],
            "TurnLeft": [(0, "TurnLeft")],
            "UTurnRight": [(0, "Hover"), (0.1, "TurnRight"), (6.5, "Hover")],
            "UTurnLeft": [(0, "Hover"), (0.1, "TurnLeft"), (6.5, "Hover")],
            "SlideLeft": [(0, "SlideLeft")],
            "SlideRight": [(0, "SlideRight")],
            "MoveVP": [(0, "AlignCorridor"), (0.1, "CenterCorridor"), (1.5, "MoveForwardVp")],
            "TurnBack" : [(0.5, "Hover"), (0.6, "TurnLeft"), (7, "Hover"),(7.5, "AlignCorridor"), (7.6, "CenterCorridor"), (8.5, "MoveForwardVp")]
        }
        self.sub_command = rospy.Subscriber(
            "/command", String, self.command_cb, queue_size=1, buff_size=2**22)
        self.pub_behavior = rospy.Publisher(
            "/behavior", BehaviorStatus, queue_size=1)

        self.schedule = Scheduler(self.pub_behavior)

    def deactivate(self):
        for behavior in self.behaviors:
            msg = BehaviorStatus()
            msg.name = behavior
            msg.status = False
            self.pub_behavior.publish(msg)

    def command_cb(self, msg):

        self.deactivate()
        events = self.commands[msg.data]
        self.schedule.create_schedule(events)

    def loop(self):
        while not rospy.is_shutdown():
            self.schedule.publish()
            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("command")
    command = Command()
    command.loop()
