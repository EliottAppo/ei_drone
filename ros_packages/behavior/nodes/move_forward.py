#! /usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32


class MoveForward(Behavior):
    def __init__(self):
        super().__init__("MoveForward")
        self.pub_linear_x = rospy.Publisher("/linear_x", Float32, queue_size=1)

    def on_status_on(self):
        self.pub_linear_x.publish(0.2)


def main():
    rospy.init_node("MoveForward")
    MoveForward()
    rospy.spin()


if __name__ == "__main__":
    main()
