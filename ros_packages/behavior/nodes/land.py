#! usr/bin/env pyhton3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Empty, Float32
from behavior.msg import BehaviorStatus
import time


class Land(Behavior):
    def __init__(self):
        super().__init__('Land')
        self.pub_land = rospy.Publisher('/bebop/land', Empty, queue_size=1)

    def on_status_on(self):
        msg = Empty()
        self.pub_land.publish(msg)


def main():
    rospy.init_node('Land')
    behavior = Land()
    rospy.spin()


if __name__ == '__main__':
    main()
