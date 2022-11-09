#! /usr/bin/env python3

import numpy as np
import cv2

import rospy
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
import visual


class OpticalFlowNode:
    """ Optical Flow Node """

    def __init__(self):
        """ Class constructor """
        self.image_pub = rospy.Publisher(
            "/optical_flow/drawing_result/compressed", CompressedImage, queue_size=1)
        self.door_pub = rospy.Publisher("/door_exists", Bool, queue_size=1)

        self.image_sub = rospy.Subscriber(
            "/bebop/image_raw/compressed", CompressedImage, self.on_image_cb, queue_size=1, buff_size=2**22)
        self.odom_sub = rospy.Subscriber(
            "/bebop/odom", Odometry, self.on_odom_cb, queue_size=1)

        self.prev_Y = None  # previous Y curve
        self.to_left = None  # indicates if drone is going to the left

        self.half_width = rospy.get_param("/optical_flow/half_width", 5)
        self.sigma = rospy.get_param("/optical_flow/sigma", 30)
        self.rho = rospy.get_param("/optical_flow/rho", 10)
        self.median_filter_window = rospy.get_param(
            "/optical_flow/median_filter_window", 100)
        self.maximum_shifts_mean = rospy.get_param(
            "/optical_flow/maximum_shifts_mean", 10)
        self.maximum_shifts_std = rospy.get_param(
            "/optical_flow/maximum_shifts_std", 5)
        self.minimum_Y_std = rospy.get_param("/optical_flow/minimum_Y_std", 20)

    def on_odom_cb(self, msg):
        """
            `odom_sub` subscriber callback
        """
        linear_y = msg.twist.twist.linear.y
        self.to_left = linear_y > 0

    def on_image_cb(self, msg):
        """ 
            `image_sub` subscriber callback
        """
        compressed_in = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)

        show_debug_drawings = self.image_pub.get_num_connections() > 0

        door_exists, Y = visual.check_if_door_exists(frame, self.prev_Y, self.to_left, self.half_width, self.sigma,
                                                     self.rho, self.median_filter_window, self.maximum_shifts_mean, self.maximum_shifts_std, self.minimum_Y_std, show_debug_drawings)

        # publishing processed image
        if show_debug_drawings:
            out_msg = CompressedImage()
            out_msg.header.stamp = rospy.Time.now()
            out_msg.format = "jpeg"
            out_msg.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()
            self.image_pub.publish(out_msg)

        # publishing door topic
        self.door_pub.publish(door_exists)

        # updating previous Y curve
        self.prev_Y = Y


if __name__ == "__main__":
    rospy.init_node("optical_flow")
    test = OpticalFlowNode()
    rospy.spin()
