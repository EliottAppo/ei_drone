#!/usr/bin/env python3

import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32
from visual_processing.msg import VanishingPoint
from multiprocessing import Lock


class MoveForwardVp(Behavior):
    def __init__(self):
        super().__init__("MoveForwardVp")
        self.vp_exists = None
        self.vp_exists_mutex = Lock()
        self.status_on = False
        self.status_on_mutex = Lock()
        self.linear_x_vel = 0.2

        self.vanishing_point_sub = rospy.Subscriber(
            "/vanishing_point/point_result", VanishingPoint, self.vanishing_point_cb)
        self.linear_x_pub = rospy.Publisher("/linear_x", Float32, queue_size=1)

    def on_status_off(self):
        self.linear_x_pub.publish(0.0)

    def vanishing_point_cb(self, msg):
        with self.vp_exists_mutex:
            self.vp_exists = msg.vp_exists

    def loop(self):
        while not rospy.is_shutdown():
            if self.get_status():
                if self.vp_exists:
                    self.linear_x_pub.publish(self.linear_x_vel)
                else:
                    self.linear_x_pub.publish(0.0)


def main():
    rospy.init_node("MoveForwardVp")
    behavior = MoveForwardVp()
    behavior.loop()


if __name__ == "__main__":
    main()
