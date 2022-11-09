#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from behavior_class import Behavior
from behavior.msg import BehaviorStatus
from std_msgs.msg import Empty
import time

class TakeOff(Behavior):
    def __init__(self):
        super().__init__('TakeOff')
        self.pub_takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=1)
        #self.sub_takeoff = rospy.Subscriber('/behavior_status', BehaviorStatus, self.takeoff_cb)

    

    def on_status_on(self):
        msg=Empty()
        self.pub_takeoff.publish(msg)
        time.sleep(2)
        self.set_status(False)

    def on_status_off(self):
        pass

def main():
	rospy.init_node('takeoff')
	behavior = TakeOff()
	rospy.spin()

if __name__ == '__main__':
    main()