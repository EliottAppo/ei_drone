#! usr/bin/env pyhton3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Empty, Float32
from behavior.msg import BehaviorStatus
import time


class Hover(Behavior):
    def __init__(self):
        super().__init__('Hover')
        self.pub_hover = rospy.Publisher('/hover', Empty, queue_size=1)
        self.pub_linear_x = rospy.Publisher('/linear_x', Float32, queue_size=1)
        self.pub_linear_y = rospy.Publisher('/linear_y', Float32, queue_size=1)
        self.pub_linear_z = rospy.Publisher('/linear_z', Float32, queue_size=1)
        self.pub_angular_z = rospy.Publisher(
            '/angular_z', Float32, queue_size=1)

    def on_status_on(self):
        msg = Empty()
        self.pub_hover.publish(msg)


def main():
    rospy.init_node('Hover')
    behavior = Hover()
    rospy.spin()


if __name__ == '__main__':
    main()
