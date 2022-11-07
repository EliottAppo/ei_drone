#! /usr/bin/env python3

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage

import visual_processing_lib


class Test:

    def __init__(self):
        self.image_pub = rospy.Publisher(
            "/draw_res/compressed", CompressedImage, queue_size=1)

        self.sub = rospy.Subscriber(
            "/bebop/image_raw/compressed",  CompressedImage, self.on_image,  queue_size=1, buff_size=2**22)
        # "/image_in/compressed",  CompressedImage, self.on_image,  queue_size=1, buff_size=2**22)

    def on_image(self, msg):
        compressed_in = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)

        cv2.line(frame, (0, 0), (50, 50), (255, 0, 0), 2)

        out_img = CompressedImage()
        out_img.header.stamp = rospy.Time.now()
        out_img.format = "jpeg"
        out_img.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        self.image_pub.publish(out_img)


if __name__ == '__main__':
    rospy.init_node('test')
    test = Test()
    rospy.spin()
