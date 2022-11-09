#! /usr/bin/env pyhton3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Empty


class Hover(Behavior):
    def __init__(self):
        super().__init__('Hover')
        self.pub_hover = rospy.Publisher('/hover', Empty, queue_size=1)

    def on_status_on(self):

        self.pub_hover.publish(Empty())


def main():
    rospy.init_node('Hover')
    Hover()
    rospy.spin()


if __name__ == '__main__':
    main()
