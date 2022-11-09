#! /usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32


class TurnLeft(Behavior):
    def __init__(self):
        super().__init__("TurnLeft")
        self.pub_angular_z = rospy.Publisher(
            "/angular_z", Float32, queue_size=1)

    def on_status_on(self):
        self.pub_angular_z.publish(0.3)


def main():
    rospy.init_node("TurnLeft")
    TurnLeft()
    rospy.spin()


if __name__ == "__main__":
    main()
