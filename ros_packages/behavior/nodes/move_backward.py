#! /usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32


class MoveBackward(Behavior):
    def __init__(self):
        super().__init__("MoveBackward")
        self.pub_linear_x = rospy.Publisher("/linear_x", Float32, queue_size=1)

    def on_status_on(self):
        self.pub_linear_x.publish(-0.5)


def main():
    rospy.init_node("MoveBackward")
    MoveBackward()
    rospy.spin()


if __name__ == "__main__":
    main()
