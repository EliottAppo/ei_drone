#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from behavior_class import Behavior
from std_msgs.msg import Empty


class TakeOff(Behavior):
    def __init__(self):
        super().__init__('TakeOff')
        self.pub_takeoff = rospy.Publisher(
            '/bebop/takeoff', Empty, queue_size=1)

    def on_status_on(self):
        msg = Empty()
        self.pub_takeoff.publish(msg)


def main():
    rospy.init_node('takeoff')
    behavior = TakeOff()
    rospy.spin()


if __name__ == '__main__':
    main()
