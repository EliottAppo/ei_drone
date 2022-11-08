import numpy as np
import cv2
from visual_processing_lib import processing, draw


MIN_LEN = 20
MIN_ANGLE = 15
MAX_ANGLE = 75
SCI_VALUE = 0.3
MIN_DIST = 3


def get_vanish_point(img,
                     min_len=MIN_LEN,
                     min_angle=MIN_ANGLE,
                     max_angle=MAX_ANGLE,
                     sci_value=SCI_VALUE,
                     min_dist=MIN_DIST):
    """
    Get vanish point from `img`
    """
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

            # draw.lines(img, right_lines, color=(0, 255, 255),
            #            thickness=2)  # drawing right lines
            # draw.lines(img, left_lines, color=(0, 255, 0),
            #            thickness=2)  # drawing left lines

            vanish_x, vanish_y = None, None

            if not(np.isnan(right_angle) or np.isnan(left_angle)):
                vanish_x, vanish_y = vp_x, vp_y
                # cv2.circle(img, (vp_x, vp_y), radius=4, color=(
                #     255, 0, 0), thickness=2)  # drawing vanishing point

            return vanish_x, vanish_y, right_lines, left_lines

    return None, None, None, None


def draw_vanish_point(img, vanish_x, vanish_y, right_lines, left_lines):
    """
    Draw vanish point from `img`
    """
    if right_lines is not None and left_lines is not None:
        draw.lines(img, right_lines, color=(0, 255, 255),
                   thickness=2)  # drawing right lines
        draw.lines(img, left_lines, color=(0, 255, 0),
                   thickness=2)  # drawing left lines

    if vanish_x is not None and vanish_y is not None:
        cv2.circle(img, (vanish_x, vanish_y), radius=4, color=(
            255, 0, 0), thickness=2)  # drawing vanishing point
