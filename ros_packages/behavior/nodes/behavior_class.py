#! usr/bin/env pyhton3

import rospy
from multiprocessing import Lock
from behavior.msg import BehaviorStatus


class Behavior:
    def __init__(self, name):
        self.stat = False
        self.name = name
        self.stat_mutex = Lock()

        self.sub_behavior = rospy.Subscriber(
            '/behavior', BehaviorStatus, self.modifbehav_cb,queue_size=1,buff_size=2**22)

        self.pub_ping = rospy.Publisher(
            '/behaviors_status', BehaviorStatus, queue_size=1)

    def get_status(self):
        with self.stat_mutex:
            return self.stat

    def activate(self):
        with self.stat_mutex:
            self.stat = True
            #print(self.stat)
            #print(self.name)


    def inactivate(self):
        with self.stat_mutex:
            self.stat = False
            #print(self.stat)
            #print(self.name)

    def modifbehav_cb(self, msg):
        #rospy.loginfo('Hello')
        #print(msg.name, self.name, msg.status)
        #print("-----------")
        if msg.name == self.name and msg.status == True:
            self.activate()

        elif msg.name == self.name and msg.status == False:
            self.inactivate()

        elif msg.name == 'ping':
            ping_msg=BehaviorStatus()
            ping_msg.status = self.get_status()
            ping_msg.name = self.name
            self.pub_ping.publish(ping_msg)