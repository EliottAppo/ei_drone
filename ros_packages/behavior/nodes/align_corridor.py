#!/usr/bin/env python3

from multiprocessing import Lock
import copy
import numpy as np
import rospy
from behavior_class import Behavior
from std_msgs.msg import Float32
from visual_processing.msg import VanishingPoint


class AlignCorridor(Behavior):
    def __init__(self):
        super().__init__("AlignCorridor")
        self.vp_exists = None
        self.vp_coordinates = np.zeros((2,))
        self.vp_coordinates_mutex = Lock()
        self.first_run = True

        img_width, img_height = 640, 368
        self.frame_center = np.array([img_width / 2 - 20, img_height / 2])

        # "self.alpha" represents the coefficient for the exponentially weighted moving average
        # applied on the vanishing point coordinates
        self.alpha = 0.95
        self.dist_threshold = 5
        self.angular_z_vel = 0.1
        self.linear_y_vel = 0.15

        self.vanishing_point_sub = rospy.Subscriber(
            "/vanishing_point/point_result", VanishingPoint, self.vanishing_point_cb)
        self.angular_z_pub = rospy.Publisher(
            "/angular_z", Float32, queue_size=1)
        self.linear_y_pub = rospy.Publisher(
            "/linear_y", Float32, queue_size=1)


    def get_vp_coordinates_from_msg(self, msg):
        return np.array([msg.vp_x, msg.vp_y])

    def get_vp_coordinates(self):
        with self.vp_coordinates_mutex:
            vp_coordinates_copy = copy.deepcopy(self.vp_coordinates)
        return vp_coordinates_copy

    def vanishing_point_cb(self, msg):
        if self.first_run:
            with self.vp_coordinates_mutex:
                self.vp_coordinates = self.get_vp_coordinates_from_msg(msg)
                self.first_run = False
        else:
            old_vp_coordinates = self.get_vp_coordinates()
            new_vp_coordinates = self.get_vp_coordinates_from_msg(msg)
            with self.vp_coordinates_mutex:
                self.vp_coordinates = self.alpha * new_vp_coordinates + \
                    (1 - self.alpha) * old_vp_coordinates
        self.vp_exists = msg.vp_exists

    def loop(self):
        while not rospy.is_shutdown():
            if self.get_status():
                if self.vp_exists:
                    vp_coordinates = self.get_vp_coordinates()
                    coord_center_dist = np.linalg.norm(vp_coordinates - self.frame_center)
                    
                    if coord_center_dist > self.dist_threshold:
                        # Vanishing point too much on the left, rotate in the counterclockwise sense
                        if vp_coordinates[0] < self.frame_center[0]:
                            self.angular_z_pub.publish(self.angular_z_vel)
                            self.linear_y_pub.publish(-self.linear_y_vel)
                        # Vanishing point too much on the right, rotate in the clockwise sense
                        elif vp_coordinates[0] > self.frame_center[0]:
                            self.angular_z_pub.publish(-self.angular_z_vel)
                            self.linear_y_pub.publish(self.linear_y_vel)


                else:
                    self.angular_z_pub.publish(0.0)


def main():
    rospy.init_node("AlignCorridor")
    behavior = AlignCorridor()
    behavior.loop()


if __name__ == "__main__":
    main()
