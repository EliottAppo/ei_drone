#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Bool
from multiprocessing import Lock


class Door:
    def __init__(self):
        self.door_exists = False
        self.door_exists_mutex = Lock()
        self.sub_door_exists = rospy.Subscriber(
            '/door_exists', Bool, self.door_exists_cb)
        self.pub_command = rospy.Publisher('/command', String, queue_size=1)

    def door_exists_cb(self, msg):
        if msg.data:
            with self.door_exists_mutex:
                self.door_exists = True


    def reset_detection(self):
        with self.door_exists_mutex:
            self.door_exists = False

    def loop(self):
        while not rospy.is_shutdown():
            if self.door_existed():
                self.reset_detection()
                self.pub_command.publish('Cross')
            else:
                #continue flying in corridor
                pass
                

    