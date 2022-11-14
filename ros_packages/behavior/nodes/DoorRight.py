#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Bool
from multiprocessing import Lock
from slideright import SlideRight


class DoorRight(SlideRight):
    def __init__(self):
        super().__init__()
        self.door_exists = False
        self.door_exists_mutex = Lock()
        self.sub_door_exists = rospy.Subscriber(
            '/door_exists', Bool, self.door_exists_cb)
        self.pub_command = rospy.Publisher('/command', String, queue_size=1)
        self.door_count = 0
        self.door_count_mutex = Lock()
        self.door_min_count = 10

    def door_exists_cb(self, msg):
        with self.door_count_mutex:
            if msg.data:
                self.door_count += 1
        if self.door_count >= self.door_min_count:
            with self.door_exists_mutex:
                self.door_exists = True

    def door_existed(self):
        with self.door_exists_mutex:
            return self.door_exists

    def reset_detection(self):
        with self.door_count_mutex:
            self.door_count = 0
        with self.door_exists_mutex:
            self.door_exists = False

    def loop(self):
        while not rospy.is_shutdown():
            if self.get_status():
                if self.door_existed():
                    self.reset_detection()
                    self.set_status(False)
                    self.pub_command.publish('CrossDoor')
            else:
                # NOTE: this is in open loop. In closed loop we have to get x,y,z speeds from the loop feedback
                self.pub_linear_x.publish(0)
                self.pub_linear_y.publish(0.5)
                self.pub_angular_z.publish(0)
                rospy.sleep(0.1)


def main():
    rospy.init_node("DoorRight")
    behavior = DoorRight()
    behavior.loop()


if __name__ == "__main__":
    main()
