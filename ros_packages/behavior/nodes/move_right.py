#! /usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32


class MoveRight(Behavior):
    def __init__(self):
        super().__init__("MoveRight")
        self.pub_linear_y = rospy.Publisher("/linear_y", Float32, queue_size=1)

    def on_status_on(self):
        self.pub_linear_y.publish(-0.5)


def main():
    rospy.init_node("MoveRight")
    MoveRight()
    rospy.spin()


if __name__ == "__main__":
    main()
