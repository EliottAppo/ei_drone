#! /usr/bin/env python3

import numpy as np
import cv2
import rospy
from sensor_msgs.msg import CompressedImage

from visual_processing_lib import draw, visual

import math


class Test:

    def __init__(self):
        self.REJECT_DEGREE_TH = 4.0
        self.image_pub = rospy.Publisher(
            "/draw_res/compressed", CompressedImage, queue_size=1)

        self.sub = rospy.Subscriber(
            "/bebop/image_raw/compressed",  CompressedImage, self.on_image,  queue_size=1, buff_size=2**22)
        # "/image_in/compressed",  CompressedImage, self.on_image,  queue_size=1, buff_size=2**22)

    def on_image(self, msg):
        compressed_in = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(compressed_in, cv2.IMREAD_COLOR)

        #### This ###
        # self.vanish_point(frame)
        #### OR ####
        self.vanish_point2(frame)

        #### FINALY ####
        # create msg to pub
        out_img = CompressedImage()
        out_img.header.stamp = rospy.Time.now()
        out_img.format = "jpeg"
        out_img.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        self.image_pub.publish(out_img)

    def vanish_point(self, img):
        min_len = 20
        min_angle = 15
        max_angle = 75
        sci_value = 0.3
        min_dist = 3

        lines = visual.detect_lines(img)

        if lines is not None:
            # filtering lines
            lines = visual.longer_than(lines, min_len)
            lines = visual.angle_inclination_between(
                lines, max_angle, min_angle)

            # draw.lines(img, lines, color=(0, 255, 255),
            #            thickness=2)

            # calculating intersection
            intersects = visual.intersect(lines)

            # if len(intersects) > 1:
            if len(intersects) > 1:

                # smallest confidence interval
                sci_x = visual.sci(intersects[:, 0], sci_value)
                sci_y = visual.sci(intersects[:, 1], sci_value)

                # determining vanishing point
                vp_x = int(np.mean(sci_x))
                vp_y = int(np.mean(sci_y))

                # filtering lines far away from vanishing point
                lines = visual.distance_to_vp_lower_than(
                    lines, (vp_x, vp_y), min_dist)

                # spliting lines between right and left lines
                right_lines, left_lines = visual.split_right_left(lines)

                # calculating right and left angles
                right_angle = visual.calculate_angle_from_lines(right_lines)
                left_angle = visual.calculate_angle_from_lines(left_lines)

                draw.lines(img, right_lines, color=(0, 255, 255),
                           thickness=2)  # drawing right lines
                draw.lines(img, left_lines, color=(0, 255, 0),
                           thickness=2)  # drawing left lines

                if not(np.isnan(right_angle) or np.isnan(left_angle)):
                    cv2.circle(img, (vp_x, vp_y), radius=4, color=(
                        255, 0, 0), thickness=2)  # drawing vanishing point

    def FilterLines(self, Lines):
        FinalLines = []

        for Line in Lines:
            [[x1, y1, x2, y2]] = Line

            # Calculating equation of the line: y = mx + c
            if x1 != x2:
                m = (y2 - y1) / (x2 - x1)
            else:
                m = 100000000
            c = y2 - m*x2
            # theta will contain values between -90 -> +90.
            theta = math.degrees(math.atan(m))

            # Rejecting lines of slope near to 0 degree or 90 degree and storing others
            if self.REJECT_DEGREE_TH <= abs(theta) <= (90 - self.REJECT_DEGREE_TH):
                l = math.sqrt((y2 - y1)**2 + (x2 - x1) **
                              2)    # length of the line
                FinalLines.append([x1, y1, x2, y2, m, c, l])

        # Removing extra lines
        # (we might get many lines, so we are going to take only longest 15 lines
        # for further computation because more than this number of lines will only
        # contribute towards slowing down of our algo.)
        if len(FinalLines) > 15:
            FinalLines = sorted(FinalLines, key=lambda x: x[-1], reverse=True)
            FinalLines = FinalLines[:15]

        return FinalLines

    def GetLines(self, Image):
        # Converting to grayscale
        GrayImage = cv2.cvtColor(Image, cv2.COLOR_BGR2GRAY)
        # Blurring image to reduce noise.
        BlurGrayImage = cv2.GaussianBlur(GrayImage, (5, 5), 1)
        # Generating Edge image
        EdgeImage = cv2.Canny(BlurGrayImage, 40, 255)

        # Finding Lines in the image
        Lines = cv2.HoughLinesP(EdgeImage, 1, np.pi / 180, 50, 10, 15)

        # Check if lines found and exit if not.
        if Lines is None:
            print("Not enough lines found in the image for Vanishing Point detection.")
            return

        # Filtering Lines wrt angle
        FilteredLines = self.FilterLines(Lines)

        return FilteredLines

    def GetVanishingPoint(self, Lines):
        # We will apply RANSAC inspired algorithm for this. We will take combination
        # of 2 lines one by one, find their intersection point, and calculate the
        # total error(loss) of that point. Error of the point means root of sum of
        # squares of distance of that point from each line.
        VanishingPoint = None
        MinError = 100000000000

        for i in range(len(Lines)):
            for j in range(i+1, len(Lines)):
                m1, c1 = Lines[i][4], Lines[i][5]
                m2, c2 = Lines[j][4], Lines[j][5]

                if m1 != m2:
                    x0 = (c1 - c2) / (m2 - m1)
                    y0 = m1 * x0 + c1

                    err = 0
                    for k in range(len(Lines)):
                        m, c = Lines[k][4], Lines[k][5]
                        m_ = (-1 / m)
                        c_ = y0 - m_ * x0

                        x_ = (c - c_) / (m_ - m)
                        y_ = m_ * x_ + c_

                        l = math.sqrt((y_ - y0)**2 + (x_ - x0)**2)

                        err += l**2

                    err = math.sqrt(err)

                    if MinError > err:
                        MinError = err
                        VanishingPoint = [x0, y0]

        return VanishingPoint

    def vanish_point2(self, img):
        Lines = self.GetLines(img)

        if not Lines or not len(Lines):
            return
        # Get vanishing point
        VanishingPoint = self.GetVanishingPoint(Lines)

        # Checking if vanishing point found
        if VanishingPoint is not None:

            # Drawing lines and vanishing point
            for Line in Lines:
                cv2.line(img, (Line[0], Line[1]),
                         (Line[2], Line[3]), (0, 255, 0), 2)
            cv2.circle(img, (int(VanishingPoint[0]), int(
                VanishingPoint[1])), 10, (0, 0, 255), -1)


if __name__ == '__main__':
    rospy.init_node('test')
    test = Test()
    rospy.spin()
