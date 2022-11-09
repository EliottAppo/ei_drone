#! /usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32


class TurnRight(Behavior):
    def __init__(self):
        super().__init__("TurnRight")
        self.pub_angular_z = rospy.Publisher(
            "/angular_z", Float32, queue_size=1)

    def on_status_on(self):
        self.pub_angular_z.publish(-0.3)


def main():
    rospy.init_node("TurnRight")
    TurnRight()
    rospy.spin()


if __name__ == "__main__":
    main()
