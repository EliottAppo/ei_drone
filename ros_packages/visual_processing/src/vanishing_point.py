#! /usr/bin/env python3

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage
from visual_processing.msg import VanishingPoint
import visual


class VanishingPointNode:
    def __init__(self):
        self.image_pub = rospy.Publisher(
            "/vanishing_point/drawing_result/compressed", CompressedImage, queue_size=1)
        self.vp_pub = rospy.Publisher(
            "/vanishing_point/point_result", VanishingPoint,  queue_size=1)

        # TODO change image subscriber topic
        self.sub = rospy.Subscriber(
            "/bebop/image_raw/compressed",  CompressedImage, self.on_image,  queue_size=1, buff_size=2**22)
        # "/image_in/compressed",  CompressedImage, self.on_image,  queue_size=1, buff_size=2**22)

        self.min_len = rospy.get_param("/vanishing_point/min_len", 30)
        self.min_angle = rospy.get_param("/vanishing_point/min_angle", 15)
        self.max_angle = rospy.get_param("/vanishing_point/max_angle", 75)
        self.sci_value = rospy.get_param("/vanishing_point/sci_value", 0.3)
        self.min_dist = rospy.get_param("/vanishing_point/min_dist", 3)

    def on_image(self, msg):
        compressed_in = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)

        vanish_x, vanish_y, right_lines, left_lines, right_angle, left_angle = visual.get_vanish_point(
            frame, self.min_len, self.min_angle, self.max_angle, self.sci_value, self.min_dist)

        # publish vanish point
        out_vp_msg = VanishingPoint()
        out_vp_msg.vp_exists = vanish_x is not None and vanish_y is not None
        if out_vp_msg.vp_exists:
            out_vp_msg.vp_x = vanish_x
            out_vp_msg.vp_y = vanish_y
            out_vp_msg.left_angle = left_angle
            out_vp_msg.right_angle = right_angle

        self.vp_pub.publish(out_vp_msg)

        # draw and publish image with vanish point drawings
        if self.image_pub.get_num_connections() > 0:
            visual.draw_vanish_point(
                frame, vanish_x, vanish_y, right_lines, left_lines)

            # create msg to pub
            out_img = CompressedImage()
            out_img.header.stamp = rospy.Time.now()
            out_img.format = "jpeg"
            out_img.data = np.array(cv2.imencode(".jpg", frame)[1]).tostring()
            self.image_pub.publish(out_img)


if __name__ == "__main__":
    rospy.init_node("vanishing_point")
    VanishingPointNode()
    rospy.spin()
