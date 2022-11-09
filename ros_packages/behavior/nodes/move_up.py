#! /usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32


class MoveUp(Behavior):
    def __init__(self):
        super().__init__("MoveUp")
        self.pub_linear_z = rospy.Publisher("/linear_z", Float32, queue_size=1)

    def on_status_on(self):
        self.pub_linear_z.publish(0.5)


def main():
    rospy.init_node("MoveUp")
    MoveUp()
    rospy.spin()


if __name__ == "__main__":
    main()
