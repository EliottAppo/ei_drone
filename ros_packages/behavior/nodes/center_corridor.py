#!/usr/bin/env python3

from multiprocessing import Lock
import copy
import numpy as np
import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32
from visual_processing.msg import VanishingPoint


class CenterCorridor(Behavior):
    def __init__(self):
        super().__init__("CenterCorridor")
        self.angle = None
        self.angle_mutex = Lock()
        self.first_run = True
        self.vp_exists = False

        self.alpha = 1
        self.angle_threshold = 10
        self.linear_y_vel = 0.1
        self.linear_y_pub = rospy.Publisher("/linear_y", Float32, queue_size=1)
        self.centercorridor_sub = rospy.Subscriber(
            "/vanishing_point/point_result", VanishingPoint, self.centercorridor_cb)

    def get_angle_from_msg(self, msg):
        return np.array([abs(msg.left_angle), abs(msg.right_angle)])

    def get_angle(self):
        with self.angle_mutex:
            angle_copy = copy.deepcopy(self.angle)
        return angle_copy

    def centercorridor_cb(self, msg):
        if self.first_run:
            with self.angle_mutex:
                self.angle = self.get_angle_from_msg(msg)
                self.first_run = False
        else:
            if msg.vp_exists:
                old_angle = np.array(self.get_angle())
                new_angle = self.get_angle_from_msg(msg)
                with self.angle_mutex:
                    self.angle = self.alpha*new_angle + \
                        (1 - self.alpha)*old_angle
        self.vp_exists = msg.vp_exists

    def loop(self):
        while not rospy.is_shutdown():
            if self.get_status():
                if self.vp_exists:
                    if abs(self.angle[0] - self.angle[1]) > self.angle_threshold:
                        # Vanishing point on the left, then go to the left
                        if self.angle[0] > self.angle[1]:
                            self.linear_y_pub.publish(-self.linear_y_vel)
                        # Vanishing point on the right, then go to the right
                        elif self.angle[0] < self.angle[1]:
                            self.linear_y_pub.publish(self.linear_y_vel)
                else:
                    self.linear_y_pub.publish(0.0)


def main():
    rospy.init_node("CenterCorridor")
    behavior = CenterCorridor()
    behavior.loop()


if __name__ == "__main__":
    main()
