#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from behavior_class import Behavior
from std_msgs.msg import Empty, Float32
from behavior.msg import BehaviorStatus
#import time


class Land(Behavior):
    def __init__(self):
        super().__init__('Land')
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)

    def on_status_on(self):
        self.pub_land.publish(Empty())


def main():
    rospy.init_node('Land')
    Land()
    rospy.spin()


if __name__ == '__main__':
    main()
