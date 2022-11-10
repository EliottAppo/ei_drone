import numpy as np
import cv2
import processing
import draw

MIN_LEN = 20
MIN_ANGLE = 15
MAX_ANGLE = 75
SCI_VALUE = 0.3
MIN_DIST = 3


def vanish_point(img,
                 min_len=MIN_LEN,
                 min_angle=MIN_ANGLE,
                 max_angle=MAX_ANGLE,
                 sci_value=SCI_VALUE,
                 min_dist=MIN_DIST):
    lines = processing.detect_lines(img)

    if lines is not None:
        # filtering lines
        lines = processing.longer_than(lines, min_len)
        lines = processing.angle_inclination_between(
            lines, max_angle, min_angle)

        # calculating intersection
        intersects = processing.intersect(lines)

        if len(intersects) > 1:

            # smallest confidence interval
            sci_x = processing.sci(intersects[:, 0], sci_value)
            sci_y = processing.sci(intersects[:, 1], sci_value)

            # determining vanishing point
            vp_x = int(np.mean(sci_x))
            vp_y = int(np.mean(sci_y))

            # filtering lines far away from vanishing point
            lines = processing.distance_to_vp_lower_than(
                lines, (vp_x, vp_y), min_dist)

            # spliting lines between right and left lines
            right_lines, left_lines = processing.split_right_left(lines)

            # calculating right and left angles
            right_angle = processing.calculate_angle_from_lines(right_lines)
            left_angle = processing.calculate_angle_from_lines(left_lines)

            draw.lines(img, right_lines, color=(0, 255, 255),
                       thickness=2)  # drawing right lines
            draw.lines(img, left_lines, color=(0, 255, 0),
                       thickness=2)  # drawing left lines

            if not(np.isnan(right_angle) or np.isnan(left_angle)):
                cv2.circle(img, (vp_x, vp_y), radius=4, color=(
                    255, 0, 0), thickness=2)  # drawing vanishing point

                return vp_x, vp_y

    return None, None
